#pragma once

#define BIT_MASK_CREATE(bits) ((1 << (bits)) - 1)
#define BIT(idx) ((1 << (idx)))
#define BIT_IS_SET(x, idx) ((x) & (1 << (BIT((idx)))))

