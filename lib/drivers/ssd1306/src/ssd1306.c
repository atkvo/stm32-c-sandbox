#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include "ssd1306.h"

typedef enum stream_types {
    STREAM_TYPE_CMD = 0x0,
    STREAM_TYPE_DATA = 0x40,
} stream_types_t;

// @todo: list all command codes
enum cmd_codes {
    CMD_SET_ADDRESS_MODE     = 0x20,
    CMD_SET_CONTRAST         = 0x81,

    CMD_DISPLAY_ON_RESUME    = 0xA4,
    CMD_DISPLAY_ON_RESET     = 0xA5,

    CMD_SET_DISPLAY_NORMAL   = 0xA6,
    CMD_SET_DISPLAY_INVERSE  = 0xA7,

    CMD_DISPLAY_OFF          = 0xAE,
    CMD_DISPLAY_ON           = 0xAF,

    // Scrolling commands
    CMD_SCROLL_HORIZONTAL_R  = 0x26, // Params: Dummy 0, Page Start Addr, Time Interval, Page End
    CMD_SCROLL_HORIZONTAL_L  = 0x27, // Params: Dummy 0, Page Start Addr, Time Interval, Page End
    CMD_SCROLL_DEACTIVATE    = 0x2E,
    CMD_SCROLL_ACTIVATE      = 0x2F,

    // Address setting commands
    CMD_SET_LOWER_COL_BASE   = 0x00,
    CMD_SET_HIGHER_COL_BASE  = 0x10,
    CMD_SET_MEMORY_ADDR_MODE = 0x20,
    CMD_SET_COL_ADDRESS      = 0x21, // Params: Col Start, Col End
    CMD_SET_PAGE_ADDRESS     = 0x22, // Params: Page Start, Page End

};

static inline void send_stream(ssd1306_handle_t handle, stream_types_t type, slice_t cmd) {
    // @info This function assumes handle has already been validated
    const uint8_t stream_type = type;

    slice_t chain[2] = {
        { .ptr = &stream_type, .len = 1 },
        cmd,
    };

    i2c_write_v(handle->i2c,
            handle->dev_addr,
            chain,
            2);
}

ssd1306_handle_t ssd1306_handle_create(
        ssd1306_ctx_t *ctx,
        i2c_handle_t i2c_handle,
        uint8_t dev_addr,
        ssd1306_display_info_t disp_info,
        slice_mutable_t display_buffer) {

    if (ctx == NULL) {
        return NULL;
    }

    ssd1306_handle_t handle = ctx;
    handle->i2c = i2c_handle;
    handle->dev_addr = dev_addr;
    handle->disp_info = disp_info;
    handle->ram = display_buffer;

    return handle;
}

void ssd1306_init(ssd1306_handle_t handle) {

    // taken from https://gist.github.com/pulsar256/564fda3b9e8fc6b06b89
    const uint8_t init_stream[] = {
        CMD_DISPLAY_OFF,

        0xD4, // Set Display Clock Divide Ratio / OSC Frequency
        0x80, // Display Clock Divide Ratio / OSC Frequency

        0xA8, // Set Multiplex Ratio
        0x3F, // Multiplex Ratio for 128x64 (64-1)

        0xD3, // Set Display Offset
        0x00, // Display Offset

        0x40, // Set Display Start Line

        0x8D, // Set Charge Pump
        0x14, // Charge Pump (0x10 External, 0x14 Internal DC/DC)

        0xA1, // Set Segment Re-Map
        0xC8, // Set Com Output Scan Direction

        0xDA, // Set COM Hardware Configuration
        0x12, // COM Hardware Configuration

        0x81, // Set Contrast
        0xCF, // Contrast

        0xD9, // Set Pre-Charge Period
        0xF1, // Set Pre-Charge Period (0x22 External, 0xF1 Internal)

        0xDB, // Set VCOMH Deselect Level
        0x40, // VCOMH Deselect Level

        CMD_DISPLAY_ON_RESET, // Set all pixels OFF
        CMD_DISPLAY_ON_RESUME, // Set all pixels OFF
        CMD_SET_DISPLAY_NORMAL, // Set display not inverted

        CMD_SET_ADDRESS_MODE, // Set horizontal address mode
        0, // Auto increment column address and page address

        CMD_DISPLAY_ON, // Set display On
    };

    send_stream(handle,
            STREAM_TYPE_CMD,
            (slice_t) {
                .ptr = init_stream,
                .len = sizeof(init_stream)
            });
}

static inline void reset_cursor(ssd1306_handle_t h) {
    const uint8_t reset_cursor[] = {
        CMD_SET_COL_ADDRESS,
        0,
        h->disp_info.columns - 1,
        CMD_SET_PAGE_ADDRESS,
        0x0,
        (h->disp_info.rows / 8) - 1,
    };

    send_stream(h, STREAM_TYPE_CMD, slice_view(reset_cursor, sizeof(reset_cursor)));
}

void ssd1306_update(ssd1306_handle_t h) {
    if (h == NULL) {
        return;
    }

    reset_cursor(h);

    send_stream(h, STREAM_TYPE_DATA, slice_view(h->ram.ptr, h->ram.len));
}

void ssd1306_update_nb(ssd1306_handle_t h) {
    if (h == NULL) {
        return;
    }

    reset_cursor(h);

    send_stream(h, STREAM_TYPE_DATA, slice_view(h->ram.ptr, h->ram.len));
}

void ssd1306_display_state_set(ssd1306_handle_t h, bool on) {
    const uint8_t cmd[] = {
        on ? CMD_DISPLAY_ON : CMD_DISPLAY_OFF,
    };

    send_stream(h, STREAM_TYPE_CMD, (slice_t) { .ptr = cmd, .len = sizeof(cmd) });
}

void ssd1306_scroll_state_set(ssd1306_handle_t h, bool on) {
    const uint8_t cmd[] = {
        on ? CMD_SCROLL_ACTIVATE : CMD_SCROLL_DEACTIVATE,
    };

    send_stream(h, STREAM_TYPE_CMD, (slice_t) { .ptr = cmd, .len = sizeof(cmd) });
}

void ssd1306_scroll_mode_set(ssd1306_handle_t h, ssd1306_scroll_dir_t dir) {
    // Assume that *only* these two values will be provided
    // and assert that they are 0 and 1 to optimize with the
    // I2C command sent to device.
    static_assert(SSD1306_SCROLL_MODE_LEFT == 0);
    static_assert(SSD1306_SCROLL_MODE_RIGHT == 1);

    const uint8_t cmd[] = {
        dir + CMD_SCROLL_HORIZONTAL_L
    };

    send_stream(h, STREAM_TYPE_CMD, (slice_t) { .ptr = cmd, .len = sizeof(cmd) });
}
