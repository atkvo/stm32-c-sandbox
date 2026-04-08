#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

#include "antos.h"
#include "slice.h"

ant_task_status_t task_a(void* ctx);
ant_task_status_t task_b(void* ctx);

void* tick(void *arg) {
    while (1) {
        ant_tick_handler();
        sleep(1);
    }
}

int main() {
    enum { TOTAL_TASKS = 2 };
    static uint8_t ant_mem[ANT_REQUIRED_MEM(TOTAL_TASKS)];
    ant_init(slice_mut_view(ant_mem, sizeof(ant_mem)), TOTAL_TASKS, 1);

    ant_register_task(task_a, NULL);
    ant_register_task(task_b, NULL);

    // Create a thread to handle the system ticks
    pthread_t tick_thread;
    if (pthread_create(&tick_thread, NULL, tick, NULL) != 0) {
        fprintf(stderr, "Error creating tick thread\n");
        return 1;
    }

    ant_run();
}

ant_task_status_t task_a(void* ctx) {
    static int count = 0;
    printf("task a: %d\n", count++);

    ant_task_schedule_next(1000);
    return ANT_TASK_OK;
}

ant_task_status_t task_b(void* ctx) {
    static int count = 0;
    printf("task b: %d\n", count++);

    ant_task_schedule_next(2000);
    return ANT_TASK_OK;
}
