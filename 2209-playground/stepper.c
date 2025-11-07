#include <hardware/gpio.h>
#include <stdio.h>
#include <pico/stdlib.h>

#include "stepper.h"
#include "pio-2209.pio.h"

#include "hardware/pio.h"
#include "hardware/clocks.h"

const int direction = 5;
const int step = 6;
const int enable = 7;
const int idx = 8;
const int diag = 9;
const int uart = 10;

const int ms1 = 14;
const int ms2 = 15;

int micro_steps = 8;

const bool away = false;
const bool toward = true;

PIO pio = pio0;
uint sm = 0;

static inline void uart_tx_program_init(PIO pio, uint sm, uint offset, uint pin_tx, uint baud) {
    // Tell PIO to initially drive output-high on the selected pin, then map PIO
    // onto that pin with the IO muxes.
    pio_sm_set_pins_with_mask64(pio, sm, 1ull << pin_tx, 1ull << pin_tx);
    pio_sm_set_pindirs_with_mask64(pio, sm, 1ull << pin_tx, 1ull << pin_tx);
    pio_gpio_init(pio, pin_tx);

    gpio_pull_up(pin_tx);

    pio_sm_config c = uart_tx_program_get_default_config(offset);

    // OUT shifts to right, no autopull
    sm_config_set_out_shift(&c, true, false, 32);

    // We are mapping both OUT and side-set to the same pin, because sometimes
    // we need to assert user data onto the pin (with OUT) and sometimes
    // assert constant values (start/stop bit)
    sm_config_set_out_pins(&c, pin_tx, 1);
    sm_config_set_sideset_pins(&c, pin_tx);

    sm_config_set_in_pins(&c, pin_tx); // for WAIT, IN
    // Shift to right, autopush enabled
    sm_config_set_in_shift(&c, true, true, 8);

    // SM transmits 1 bit per 8 execution cycles.
    float div = (float)clock_get_hz(clk_sys) / (8 * baud);
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}


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

    uint offset = pio_add_program(pio, &uart_tx_program);
    printf("Loaded program at %d\n", offset);

    uart_tx_program_init(pio, sm, offset, uart, 500000);

    pio_sm_set_enabled(pio, sm, true);
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

void move(bool dir, int distance, int delay) {
    gpio_put(direction, dir);

    int micro_steps_taken = 0;
    int cycles = 0;

    for (int i = 0; i < distance * micro_steps; i++) {
        step_motor();
        for (int j = 0; j < delay; j++) {

        }
        if (gpio_get(idx)) {
            cycles++;
        }
        micro_steps_taken++;
    }

    int full_steps = micro_steps_taken / micro_steps;
    int rotations = full_steps / 200;

    printf("microsteps: %d\n", micro_steps_taken);
    printf("cycles: %d\n", cycles);
    printf("full steps: %d\n", full_steps);
    printf("rotations: %d\n", rotations);
}

void dance() {
    move(away, 1000, 10);
    move(toward, 1000, 1);
}

int uart2209 = 8;

void uart_idle(int uart) {
    gpio_put(uart, true);
}

void uart_put_byte(int uart, uint8_t payload, uint8_t* crc) {
    gpio_put(uart, false); // start

    for (int bit = 0; bit < 8; bit++) {

        uint next_bit = payload & 0x1;
        bool transmit = next_bit == 0x1;
        if (crc) {
            *crc = (*crc << 1) | ((*crc >> 7) ^ ((*crc >> 1) & 0x1) ^ (*crc & 0x1) ^ next_bit);
        }
        gpio_put(uart, transmit);

        payload >>= 1;
    }

    gpio_put(uart, true); // stop

}

static inline void uart_tx_program_putc(PIO pio, uint sm, char c) {
    pio_sm_put_blocking(pio, sm, (uint32_t)c);
}

static inline uint8_t uart_rx_program_getc(PIO pio, uint sm) {
    // 8-bit read from the uppermost byte of the FIFO, as data is left-justified
    io_rw_8 *rxfifo_shift = (io_rw_8*)&pio->rxf[sm] + 3;
    while (pio_sm_is_rx_fifo_empty(pio, sm))
        tight_loop_contents();
    return (uint8_t)*rxfifo_shift;
}

void uart_put_byte_pio(PIO pio, uint sm, uint8_t payload, uint8_t* crc) {

    uint8_t working = payload;

    for (int bit = 0; bit < 8; bit++) {

        uint next_bit = working & 0x1;
        bool transmit = next_bit == 0x1;
        if (crc) {
            *crc = (*crc << 1) | ((*crc >> 7) ^ ((*crc >> 1) & 0x1) ^ (*crc & 0x1) ^ next_bit);
        }
        working >>= 1;
    }

    uart_tx_program_putc(pio, sm, payload);

}

void send_read_request_pio(uint8_t address) {
    uint8_t wire_address = address & 0x7;

    uint8_t crc = 0;
    uart_put_byte_pio(pio, sm, 0x05, &crc);
    uart_put_byte_pio(pio, sm, 0x00, &crc);
    uart_put_byte_pio(pio, sm, wire_address, &crc);
    printf("crc: %d\n", crc);
    uart_put_byte_pio(pio, sm, crc, NULL);

    uint8_t reply[8];

    for (int i = 0; i < 8; i++) {

        reply[i] = uart_rx_program_getc(pio, sm);
        printf("result: %d\n", reply[i]);
    }
}