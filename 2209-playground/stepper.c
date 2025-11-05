#include <hardware/gpio.h>
#include <stdio.h>
#include <pico/stdlib.h>

#include "stepper.h"

const int direction = 5;
const int step = 6;
const int enable = 7;
const int idx = 8;
const int diag = 9;

const int ms1 = 14;
const int ms2 = 15;

int micro_steps = 8;

const bool away = false;
const bool toward = true;


void init_stepper() {
    gpio_init(direction);
    gpio_set_dir(direction, GPIO_OUT);
    gpio_put(direction, false);

    gpio_init(step);
    gpio_set_dir(step, GPIO_OUT);
    gpio_put(step, false);

    gpio_init(idx);
    gpio_set_dir(idx, GPIO_IN);

    gpio_init(diag);
    gpio_set_dir(diag, GPIO_IN);

    gpio_init(enable);
    gpio_set_dir(enable, GPIO_OUT);
    gpio_put(enable, false);

    gpio_init(ms1);
    gpio_set_dir(ms1, GPIO_OUT);
    gpio_put(ms1, false);

    gpio_init(ms2);
    gpio_set_dir(ms2, GPIO_OUT);
    gpio_put(ms2, false);
}

void enable_motor() {
    gpio_put(enable, false);
}

void disable_motor() {
    gpio_put(enable, true);
}

void step_motor() {
    gpio_put(step, false);
    sleep_us(100);
    gpio_put(step, true);
    sleep_us(100);
    if (gpio_get(diag)) {
        printf("help!\n");
    }
}

void rotate(bool dir) {
    gpio_put(direction, dir);

    for (int i = 0; i < 200; i++) {
        for (int m = 0; m < micro_steps; m++) {
            step_motor();
        }
    }
} 