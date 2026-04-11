#include "si7021.h"
#include "i2c.h"
#include "slice.h"

enum {
    MEASURE_HUMIDITY_HOLD_MODE = 0xE5,
    MEASURE_HUMIDITY_NO_HOLD_MODE = 0xF5,

    MEASURE_TEMP_HOLD_MODE = 0xE3,
    MEASURE_TEMP_NO_HOLD_MODE = 0xF3,
} SI7021_CMD_TABLE;

enum {
    SI7021_DEV_ADDRESS = 0x40,
};

si7021_handle_t si7021_handle_create(si7021_ctx_t *ctx, i2c_handle_t i2c) {
    if ((i2c == NULL) || (ctx == NULL)) {
        return NULL;
    }

    ctx->i2c = i2c;

    return ctx;
}

uint32_t si7021_temp_get(si7021_handle_t h) {
    return 0;
}

uint32_t si7021_humid_get(si7021_handle_t h) {
    uint8_t write_data[1] = {
        // MEASURE_TEMP_HOLD_MODE,
        MEASURE_HUMIDITY_HOLD_MODE,
    };

    uint8_t read_data[2];
    i2c_burst_write_read(
            h->i2c,
            SI7021_DEV_ADDRESS,
            slice_view(write_data, sizeof(write_data)),
            slice_mut_view(read_data, sizeof(read_data))
    );

    uint32_t d = read_data[1] | (read_data[0] << 8);
    d = ((125 * d) / 65536) - 6;

    return d;
}
