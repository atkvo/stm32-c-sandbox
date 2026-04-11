#include "task_sensor.h"
#include "antos.h"
#include "si7021.h"

sensor_task_ctx_t task_sensor_create_ctx(si7021_handle_t sensor) {
    return (sensor_task_ctx_t) {
        .sensor = sensor,
        .humidity = 0xDEADBEEF,
    };
}

void task_sensor(sensor_task_ctx_t *ctx) {
    ctx->humidity = si7021_humid_get(ctx->sensor);

    ant_task_schedule_next(1500);
}
