#include "custom_app.h"

#include "commands.h"
#include "packet.h"
#include "vesc_buffer.h"

#include <string.h>

size_t custom_app_encode(const uint8_t *data, size_t len, uint8_t *out, size_t cap) {
    if (len + 1 > cap || len + 1 > VESC_PACKET_MAX_PL_LEN) {
        return 0;
    }
    size_t ind = 0;
    vb_append_uint8(out, COMM_CUSTOM_APP_DATA, &ind);
    if (len > 0) {
        memcpy(out + ind, data, len);
        ind += len;
    }
    return ind;
}
