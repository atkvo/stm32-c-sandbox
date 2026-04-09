#include "dma.h"
#include "pool_mgr.h"
#include "stm32f411xe.h"

#define DMA_COUNT 2
#define DMA_STREAMS_PER 8
#define DMA_TOTAL_CONTEXTS (DMA_COUNT * DMA_STREAMS_PER)


typedef struct dma_map {
    DMA_TypeDef *dma;
    DMA_Stream_TypeDef *stream;
} dma_map_t;

typedef struct dma_ctx {
    dma_map_t map; // @todo: potential optimization - point to lookup table
    dma_cb_t cb;
    void *data;
} dma_ctx_t;

enum {
    DMA_PERIPH_1 = 0,
    DMA_PERIPH_2 = 1,
};

static dma_ctx_t dma_contexts[DMA_TOTAL_CONTEXTS];
static pool_manager_t dma_pool = POOL_MGR_INIT(dma_contexts, dma_ctx_t, DMA_TOTAL_CONTEXTS);

enum {
    DMA_ERR_INVALID_DMA_MAP = -1,
};
static int32_t dma_to_pool_idx(DMA_TypeDef *dma, DMA_Stream_TypeDef *stream) {
    int32_t dma_idx = 0;
    uint32_t base_stream = 0;

    /* Simple check since there are only 2 DMAs - skipping arithmetic */
    if (dma == DMA1) {
        dma_idx = 0;
        base_stream = DMA1_Stream0_BASE;
    }
    else if (dma == DMA2) {
        dma_idx = 1;
        base_stream = DMA2_Stream0_BASE;
    }
    else {
        return DMA_ERR_INVALID_DMA_MAP;
    }

    if ((uint32_t)stream < base_stream) {
        return DMA_ERR_INVALID_DMA_MAP;
    }

    uint32_t stream_idx = ((uint32_t)stream - base_stream) / sizeof(DMA_Stream_TypeDef);
    size_t pool_idx = (dma_idx * DMA_STREAMS_PER) + stream_idx;
    if (pool_idx >= DMA_TOTAL_CONTEXTS) {
        return DMA_ERR_INVALID_DMA_MAP;
    }

    return pool_idx;
}

static dma_ctx_t* get_dma_context(DMA_TypeDef *dma, DMA_Stream_TypeDef *stream) {
    /* no bounds checking */
    size_t dma_idx = dma_to_pool_idx(dma, stream);

    if (dma_idx >= 0) {
        return &dma_contexts[dma_idx];
    }
    else {
        return NULL;
    }
}

static const dma_map_t dma_lookup[DMA_REQ_COUNT] = {
    [DMA_REQ_I2C1_TX] = { .dma = DMA1, .stream = DMA1_Stream1 },
};

void DMA1_Stream1_IRQHandler(void) {
    dma_ctx_t *ctx = get_dma_context(DMA1, DMA1_Stream1);
    if (ctx->map.dma->LISR & DMA_LISR_TCIF1)
    {
        if (ctx->cb) {
            ctx->cb(ctx->data);
        }
        ctx->map.dma->LIFCR |= DMA_LIFCR_CTCIF1;
    }
}

dma_handle_t dma_stream_take(dma_request_t req) {
    const dma_map_t *map = &dma_lookup[req];
    int32_t idx = dma_to_pool_idx(map->dma, map->stream);
    if (idx < 0) {
        return NULL;
    }

    dma_handle_t ctx = pool_mgr_take(&dma_pool, idx);
    if (ctx) {
        ctx->map = *map;
    }
    return ctx;
}

void dma_stream_release(dma_handle_t handle) {
    if (handle->map.dma == DMA1) {
        RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA1EN;
    }
    else {
        RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA2EN;
    }

    handle->cb = NULL;
    handle->data = NULL;
    handle->map.stream->CR &= ~DMA_SxCR_EN;
    pool_mgr_release(&dma_pool, handle);
}

void dma_start(dma_handle_t handle, const uint32_t *src, uint32_t *dest, size_t len) {
    if (handle == NULL) {
        return;
    }

    /* Disable stream */
    handle->map.stream->CR &= ~DMA_SxCR_EN;
    while (handle->map.stream->CR & DMA_SxCR_EN);

    /* Configure memory address and length */
    handle->map.stream->M0AR = (uint32_t)src;
    handle->map.stream->PAR = (uint32_t)dest;
    handle->map.stream->NDTR = len;

    /* Clear interrupts */
    handle->map.dma->HIFCR = UINT32_MAX;

    /* Enable stream */
    handle->map.stream->CR |= DMA_SxCR_EN;
}

void dma_configure(dma_handle_t handle, dma_cfg_t *cfg) {
    if (handle == NULL) {
        return;
    }

    if (handle->map.dma == DMA1) {
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    }
    else {
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    }

    handle->map.stream->CR = 0;
    while (handle->map.stream->CR & DMA_SxCR_EN);

    const size_t ch_bit = (0 << 25);
    handle->map.stream->CR |= ch_bit |
        DMA_SxCR_MINC |
        DMA_SxCR_TCIE |
        // DMA_SxCR_TEIE; // @todo: add error handling
        DMA_SxCR_DIR_0;

    NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}

void dma_register_callback(dma_handle_t handle, dma_cb_t cb, void *user_data) {
    if (handle == NULL) {
        return;
    }

    handle->cb = cb;
    handle->data = user_data;
}
