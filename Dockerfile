# Воспроизводимая среда для host-тестов Refloat.
#
# Ничего специфичного для ESP32 здесь нет: образ доказывает, что неизменённые
# исходники Refloat собираются и проходят сценарии на обычном Linux/x86.
FROM debian:bookworm-20250630-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        make \
        python3 \
        git \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src
COPY . /src

# Submodule должен быть проинициализирован на хосте либо смонтирован внутрь:
#   git submodule update --init
RUN test -f refloat-upstream/src/main.c \
    || (echo "refloat-upstream пуст: выполните 'git submodule update --init'" && exit 1)

RUN make -C tests/host

CMD ["make", "-C", "tests/host", "test"]
