#include "task_sensor.h"
#include "antos.h"
#include "si7021.h"

task_sensor_ctx_t task_sensor_create_ctx(si7021_handle_t sensor) {
    return (task_sensor_ctx_t) {
        .sensor = sensor,
        .humidity = 0xDEADBEEF,
    };
}

void task_sensor(task_sensor_ctx_t *ctx) {
    /* @note: technically humidity command already measures temperature
     * so we can optimize to use a get humid + temp API call to get both values
     * quickly instead of issuing a get temperature command
     */
    ctx->humidity = si7021_humid_get(ctx->sensor);
    ctx->temperature = si7021_temp_get(ctx->sensor);

    ant_task_schedule_next(1500);
}
