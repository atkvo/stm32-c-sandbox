#include "ssd1306.h"

enum stream_types {
    STREAM_CMD = 0x0,
    STREAM_DATA = 0x40,
};

enum cmd_codes
{
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

void ssd1306_init(i2c_handle_t handle, uint8_t dev_addr) {

    // taken from https://gist.github.com/pulsar256/564fda3b9e8fc6b06b89
    const uint8_t init_stream[] = {
        0xAE,

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
        CMD_DISPLAY_ON, // Set display On
    };

    i2c_burst_write(handle, dev_addr, STREAM_CMD, init_stream, sizeof(init_stream));
}
