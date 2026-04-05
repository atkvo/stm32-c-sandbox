#pragma once

#ifndef HAL_DMA_H
#define HAL_DMA_H

#include <stddef.h>
#include <stdint.h>

/* @brief DMA interface
 */

struct dma_ctx;

typedef struct dma_ctx* dma_handle_t;
typedef void (*dma_cb_t)(void *data);

typedef enum {
    DMA_REQ_I2C1_TX,
    DMA_REQ_I2C2_TX,
    DMA_REQ_I2C3_TX,

    DMA_REQ_COUNT
} dma_request_t;

typedef enum {
    DMA_PERIPH_TO_MEM,
    DMA_MEM_TO_PERIPH,
    DMA_MEM_TO_MEM,
} dma_direction_t;

typedef struct {
    dma_direction_t dir;
    uint32_t data_width;
} dma_cfg_t;

dma_handle_t dma_stream_take(dma_request_t req);
void dma_stream_release(dma_handle_t handle);
void dma_configure(dma_handle_t handle, dma_cfg_t *cfg);
void dma_start(dma_handle_t handle, const uint32_t *src, uint32_t *dest, size_t len);
void dma_stop(dma_handle_t handle);
void dma_register_callback(dma_handle_t handle, dma_cb_t cb, void *user_data);
void dma_get_status(dma_handle_t handle);
void dma_get_remaining(dma_handle_t handle);

#endif
