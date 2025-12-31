#include <assert.h>
#include <stddef.h>

#include "i2c.h"
#include "bit_utils.h"
#include "gpio.h"
#include "stm32f411xe.h"

struct i2c_ctx {
    volatile I2C_TypeDef* reg_ptr;
    uint8_t resource_idx;
};

static inline void i2c_start(i2c_handle_t);
static inline void i2c_stop(i2c_handle_t);
static inline void i2c_stop_no_wait(i2c_handle_t);
static inline void i2c_addr_wait(i2c_handle_t, uint8_t address, bool is_read);
static inline void i2c_addr_flag_clear(i2c_handle_t h);
static inline void i2c_write(i2c_handle_t, uint8_t data);
static inline bool i2c_read_byte(i2c_handle_t, uint8_t *out);
static inline void i2c_ack_enable(i2c_handle_t);
static inline void i2c_ack_disable(i2c_handle_t);

#define I2C_MAX_INSTANCES (3)
static struct i2c_ctx i2c_pool[I2C_MAX_INSTANCES];

/**
 * @brief Initialize hardware (Clocks, GPIOs, Baud rate, Duty cycle)
 */
i2c_handle_t i2c_acquire(uint8_t i2c_idx) {
    if (i2c_idx >= I2C_MAX_INSTANCES) {
        return NULL;
    }

    static_assert((I2C2_BASE - I2C1_BASE) == (I2C3_BASE - I2C2_BASE),
            "I2C reg gap calculation requires equal gaps between I2C mmio reg sets");
    enum { I2C_REG_SIZE = (I2C2_BASE - I2C1_BASE) };
    i2c_pool[i2c_idx].resource_idx = i2c_idx;
    i2c_pool[i2c_idx].reg_ptr = (volatile I2C_TypeDef*)(I2C1_BASE + (i2c_idx * I2C_REG_SIZE));

    return &i2c_pool[i2c_idx];
}

/**
 * @brief Configures I2C device
 */
void i2c_init(i2c_handle_t handle, gpio_pin_handle_t scl, gpio_pin_handle_t sda) {
    if ((handle == NULL) || (scl == NULL) || (sda == NULL)) {
        return;
    }

    // Turn on I2C peripheral clock
    const uint8_t RCC_I2C_BASE_BIT_INDEX = 21;
    RCC->APB1ENR |= BIT(RCC_I2C_BASE_BIT_INDEX + handle->resource_idx);

    // Configure GPIO pins
    gpio_pin_config_t gpio_cfg;
    gpio_cfg.mode = GPIO_MODE_ALTERNATE;
    gpio_cfg.pupd = GPIO_PUPD_PULLUP;
    gpio_cfg.af.output_mode = GPIO_OUTPUT_MODE_OPEN_DRAIN;
    gpio_cfg.af.speed = GPIO_SPEED_FAST;
    gpio_cfg.af.af_mode = 4;

    gpio_pin_configure(scl, gpio_cfg);
    gpio_pin_configure(sda, gpio_cfg);

    volatile I2C_TypeDef* i2c_ptr = handle->reg_ptr;

    // toggle reset
    i2c_ptr->CR1 |= I2C_CR1_SWRST;
    i2c_ptr->CR1 &= ~I2C_CR1_SWRST;

    i2c_ptr->CR1 &= ~(I2C_CR1_PE); // Disable peripheral before configuring clock

    // configure clock
    i2c_ptr->CR2 &= ~(I2C_CR2_FREQ_Msk);
    const uint32_t freq_mhz = SystemCoreClock / 1000000; // convert to MHz
    i2c_ptr->CR2 |= freq_mhz << I2C_CR2_FREQ_Pos;

    // get nanosecond period
    // period in mhz = us
    // 1000 ns / us
    // period_ns = 1000 (ns/us) * (f us) = (1000 * f) ns
    const uint32_t period_ns = (1000) / freq_mhz;

    // See datasheet for details (Section 18.6.8 (I2C_CCR))
    const uint32_t t_RSCL_NS = 300;
    const uint32_t t_WSCLL_NS = 4000;
    // CCR = t_rSCL + t_wSCLH / t_PCLK1
    const uint32_t ccr = (t_RSCL_NS + t_WSCLL_NS ) / period_ns;

    i2c_ptr->CCR =  ccr & I2C_CCR_CCR_Msk;
    i2c_ptr->TRISE = (t_RSCL_NS / period_ns) + 1;

    i2c_ptr->CR1 |= I2C_CR1_PE;
}

/**
 * @brief Simple byte write to a device register
 */
void i2c_write_reg(i2c_handle_t handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    if (handle == NULL) {
        return;
    }
    i2c_start(handle);
    i2c_addr_wait(handle, dev_addr, false);
    i2c_addr_flag_clear(handle);
    i2c_write(handle, reg_addr);
    i2c_write(handle, data);
    i2c_stop(handle);
}

/**
 * @brief Simple byte read from a device register
 */
uint8_t i2c_read_reg(i2c_handle_t handle, uint8_t dev_addr, uint8_t reg_addr) {
    if (handle == NULL) {
        return 0;
    }

    uint8_t data;
    i2c_burst_read(handle, dev_addr, reg_addr, &data, 1);
    return data;

}

/**
 * @brief Burst write
 */
void i2c_burst_write(i2c_handle_t handle, uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint16_t len) {
    if (handle == NULL) {
        return;
    }

    i2c_start(handle);
    i2c_addr_wait(handle, dev_addr, false);
    i2c_addr_flag_clear(handle);

    for (uint32_t i = 0; i < len; i++)
    {
        i2c_write(handle, data[i]);
    }

    i2c_stop(handle);
}

/**
 * @brief Burst read
 */
void i2c_burst_read(i2c_handle_t handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    if (handle == NULL) {
        return;
    }

    i2c_start(handle);
    i2c_addr_wait(handle, dev_addr, false);
    i2c_addr_flag_clear(handle);
    i2c_write(handle, reg_addr);

    i2c_start(handle);
    i2c_addr_wait(handle, dev_addr, true);
    // address flag cleared based on number of bytes read

    if (len == 1) {
        // Special case for single byte - must disable the ACK
        // before clearing the address flag to prevent extra
        // data bytes from slave
        i2c_ack_disable(handle);
        i2c_addr_flag_clear(handle);
        i2c_stop_no_wait(handle);
        i2c_read_byte(handle, &data[0]);
    }
    else {
        i2c_addr_flag_clear(handle);
        for (uint16_t i = 0; i < len; i++) {
            if (i == (len - 1)) {
                // @note this would be repeated for 1 byte read also,
                // but should not be an issue
                i2c_ack_disable(handle);
                i2c_stop(handle);
            }

            i2c_read_byte(handle, &data[i]);
        }
    }

    i2c_ack_enable(handle);
}

static inline void i2c_start(i2c_handle_t h)
{
    while (h->reg_ptr->SR2 & I2C_SR2_BUSY);
    h->reg_ptr->CR1 |= I2C_CR1_START;
    while (!(h->reg_ptr->SR1 & I2C_SR1_SB));
}

static inline void i2c_ack_disable(i2c_handle_t h) {
    h->reg_ptr->CR1 &= ~I2C_CR1_ACK;
}

static inline void i2c_ack_enable(i2c_handle_t h) {
    h->reg_ptr->CR1 |= I2C_CR1_ACK;
}

static inline void i2c_stop(i2c_handle_t h)
{
    h->reg_ptr->CR1 |= I2C_CR1_STOP;
    uint32_t timeout = 1000; // @todo: configurable
    while ((h->reg_ptr->SR2 & I2C_SR2_BUSY) && --timeout);
}

static inline void i2c_stop_no_wait(i2c_handle_t h)
{
    h->reg_ptr->CR1 |= I2C_CR1_STOP;
}

static inline void i2c_addr_wait(i2c_handle_t h, uint8_t address, bool is_read)
{
    h->reg_ptr->DR = address << 1 | (is_read ? 1 : 0);
    while (!(h->reg_ptr->SR1 & I2C_SR1_ADDR));
}

static inline void i2c_addr_flag_clear(i2c_handle_t h)
{
    (void)h->reg_ptr->SR1;
    (void)h->reg_ptr->SR2;
}

static inline void i2c_write(i2c_handle_t h, uint8_t data)
{
    while (!(h->reg_ptr->SR1 & (I2C_SR1_TXE)));
    h->reg_ptr->DR = data;
    while (!(h->reg_ptr->SR1 & (I2C_SR1_BTF)));
}

static inline bool i2c_read_byte(i2c_handle_t h, uint8_t *out) {
    while (!(h->reg_ptr->SR1 & I2C_SR1_RXNE));
    *out = h->reg_ptr->DR;

    return true;
}
