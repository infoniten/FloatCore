// TCP-транспорт для host-сборки.
//
// Одно соединение за раз — этого достаточно: VESC Tool подключается одним
// клиентом. После разрыва сервер снова ждёт подключения, состояние Refloat
// при этом не сбрасывается (важно для проверки «изменения переживают reconnect»).

#include "transport.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

typedef struct {
    int listen_fd;
    int client_fd;
    uint16_t port;
} TcpImpl;

static bool tcp_accept(Transport *t) {
    TcpImpl *s = (TcpImpl *) t->impl;

    struct sockaddr_in addr;
    socklen_t len = sizeof(addr);
    int fd = accept(s->listen_fd, (struct sockaddr *) &addr, &len);
    if (fd < 0) {
        return false;
    }

    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    s->client_fd = fd;
    printf("[transport] клиент подключён: %s:%u\n", inet_ntoa(addr.sin_addr),
           (unsigned) ntohs(addr.sin_port));
    fflush(stdout);
    return true;
}

static int tcp_recv(Transport *t, uint8_t *buf, size_t cap) {
    TcpImpl *s = (TcpImpl *) t->impl;
    if (s->client_fd < 0) {
        return -1;
    }
    ssize_t n = recv(s->client_fd, buf, cap, 0);
    if (n == 0) {
        return -1;  // клиент закрыл соединение
    }
    if (n < 0) {
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        }
        return -1;
    }
    return (int) n;
}

static bool tcp_send(Transport *t, const uint8_t *data, size_t len) {
    TcpImpl *s = (TcpImpl *) t->impl;
    if (s->client_fd < 0) {
        return false;
    }
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = send(s->client_fd, data + sent, len - sent, 0);
        if (n <= 0) {
            if (n < 0 && errno == EINTR) {
                continue;
            }
            return false;
        }
        sent += (size_t) n;
    }
    return true;
}

static void tcp_disconnect(Transport *t) {
    TcpImpl *s = (TcpImpl *) t->impl;
    if (s->client_fd >= 0) {
        close(s->client_fd);
        s->client_fd = -1;
        printf("[transport] клиент отключён\n");
        fflush(stdout);
    }
}

static bool tcp_is_connected(Transport *t) {
    return ((TcpImpl *) t->impl)->client_fd >= 0;
}

static void tcp_destroy(Transport *t) {
    TcpImpl *s = (TcpImpl *) t->impl;
    tcp_disconnect(t);
    if (s->listen_fd >= 0) {
        close(s->listen_fd);
    }
    free(s);
    free(t);
}

Transport *tcp_transport_create(uint16_t port) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        perror("socket");
        return NULL;
    }

    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);

    if (bind(fd, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
        perror("bind");
        close(fd);
        return NULL;
    }
    if (listen(fd, 1) < 0) {
        perror("listen");
        close(fd);
        return NULL;
    }

    TcpImpl *impl = calloc(1, sizeof(TcpImpl));
    Transport *t = calloc(1, sizeof(Transport));
    if (!impl || !t) {
        free(impl);
        free(t);
        close(fd);
        return NULL;
    }

    impl->listen_fd = fd;
    impl->client_fd = -1;
    impl->port = port;

    t->name = "TCP";
    t->impl = impl;
    t->accept = tcp_accept;
    t->recv = tcp_recv;
    t->send = tcp_send;
    t->disconnect = tcp_disconnect;
    t->destroy = tcp_destroy;
    t->is_connected = tcp_is_connected;
    return t;
}
