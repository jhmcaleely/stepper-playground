#include <stdio.h>
#include <assert.h>
#include <pico/stdlib.h>

#include "stepper.h"
#include "uart-request-reply.pio.h"

#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <hardware/clocks.h>
#include <hardware/uart.h>

const int direction = 5;
const int step = 6;
const int enable = 7;
const int idx = 8;
const int diag = 9;
const int tmc2209uart = 10;

const int uart_rx = 21;
const int uart_tx = 20;

const int ms1 = 14;
const int ms2 = 15;

int micro_steps = 8;

const bool away = false;
const bool toward = true;

uint program_offset = 0;

uint global_baudrate = 100000;

const uint8_t preamble = 0x05;

PIO pio = pio0;
uint sm = 0;

uint8_t tmc2209crc_accumulate(uint8_t payload, uint8_t crc);

static inline void uart_request_reply_program_init(PIO pio, uint sm, uint offset, uint pin_io, uint baud) {
    // Tell PIO to initially drive output-high on the selected pin, then map PIO
    // onto that pin with the IO muxes.
    pio_sm_set_pins_with_mask64(pio, sm, 1ull << pin_io, 1ull << pin_io);
    pio_sm_set_pindirs_with_mask64(pio, sm, 1ull << pin_io, 1ull << pin_io);
    pio_gpio_init(pio, pin_io);

//    gpio_pull_up(pin_io);

    pio_sm_config c = uart_request_reply_program_get_default_config(offset);

    // OUT shifts to right, no autopull
    sm_config_set_out_shift(&c, true, false, 8);

    // We are mapping both OUT and side-set to the same pin, because sometimes
    // we need to assert user data onto the pin (with OUT) and sometimes
    // assert constant values (start/stop bit)
    sm_config_set_out_pins(&c, pin_io, 1);
    sm_config_set_sideset_pins(&c, pin_io);

    sm_config_set_in_pins(&c, pin_io); // for WAIT, IN
    sm_config_set_set_pins(&c, pin_io, 1);
//    sm_config_set_jmp_pin(&c, pin_io); // for JMP, strict
    // Shift to right, autopush enabled
    sm_config_set_in_shift(&c, true, true, 8);

    // SM transmits 1 bit per 8 execution cycles.
    float div = (float)clock_get_hz(clk_sys) / (8 * baud);
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}


void init_uart_hw() {
    uart_init(uart1, global_baudrate);

    gpio_set_function(uart_tx, UART_FUNCSEL_NUM(uart1, uart_tx));
    gpio_set_function(uart_rx, UART_FUNCSEL_NUM(uart1, uart_rx));
}

void init_stepper() {
    gpio_init(direction);
    gpio_set_dir(direction, GPIO_OUT);
    gpio_put(direction, true);

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

    program_offset = pio_add_program(pio, &uart_request_reply_program);
    printf("Loaded program at %d\n", program_offset);

    uart_request_reply_program_init(pio, sm, program_offset, tmc2209uart, global_baudrate);

    pio_sm_set_enabled(pio, sm, true);

    init_uart_hw();
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

static inline void uart_tx_program_putc(PIO pio, uint sm, char c) {
    pio_sm_put_blocking(pio, sm, (uint32_t)c);
}

static inline uint8_t uart_rx_program_getc(PIO pio, uint sm) {

    uint32_t rxdata;
    // 8-bit read from the uppermost byte of the FIFO, as data is left-justified
    io_rw_8 *rxfifo_shift = (io_rw_8*)&rxdata + 3;
    while (pio_sm_is_rx_fifo_empty(pio, sm))
        tight_loop_contents();
    rxdata = pio_sm_get(pio, sm);
    printf("rx contains %08x\n", rxdata);
    return (uint8_t)*rxfifo_shift;
}


uint8_t tmc2209crc_accumulate(uint8_t payload, uint8_t crc) {
    for (size_t j = 0; j < 8; j++) {
        if ((crc >> 7) ^ (payload & 0x01)) {
            crc = (crc << 1) ^ 0x07;
        } else {
            crc = (crc << 1);
        }
        payload = payload >> 1;
    }
    return crc;
}


bool validate_reply(uint8_t* reply, uint8_t address_request) {

    uint8_t reply_crc = 0;
    for (int i = 0; i < 7; i++) {
        reply_crc = tmc2209crc_accumulate(reply[i], reply_crc);
    }
    if (reply[7] != reply_crc) {
        printf("invalid crc\n");
        return false;
    }
    if ((reply[0] & 0x0f) != preamble) {
        printf("invalid preamble\n");
        return false;
    }
    if (reply[1] != 0xFF) {
        printf("invalid master address\n");
        return false;
    }
    if ((reply[2] & 0x80) != 0x0) {
        printf("invalid request address\n");
        return false;
    }
    if ((reply[2] & 0x7f) != address_request) {
        printf("incorrect request address\n");
        return false;
    }
    return true;
}

void uart_tmc2209_put_byte(uint8_t byte, uint8_t* crc) {
    if (crc) {
        *crc = tmc2209crc_accumulate(byte, *crc);
    }
#if 1
    uart_putc_raw(uart1, byte);
    uint8_t loop = uart_getc(uart1);
    assert(byte == loop);
#else
    uart_tx_program_putc(pio, sm, byte);
#endif
}

uint8_t uart_tmc2209_get_byte() {
#if 1
    return uart_getc(uart1);
#else
    return uart_rx_program_getc(pio, sm);
#endif
}

bool read_register(uint8_t reg, uint32_t* value) {

    uint8_t wire_address = reg & 0x7;
    uint8_t message[4] = { preamble, 0x00, wire_address ,0 };

    for (size_t m = 0; m < 3; m++) {
        uart_tmc2209_put_byte(message[m], &message[3]);
    }
    uart_tmc2209_put_byte(message[3], NULL);

    uint8_t reply_message[8];
    for (int i = 0; i < 8; i++) {
        reply_message[i] = uart_tmc2209_get_byte();
    }

    bool valid = validate_reply(reply_message, reg);
    if (valid) {
        *value  = reply_message[3] << 24;
        *value |= reply_message[4] << 16;
        *value |= reply_message[5] << 8;
        *value |= reply_message[6];
    }
    sleep_ms(1);
    return valid;
}

void write_register(uint8_t reg, uint32_t value) {

    uint8_t wire_address = reg & 0x7;
    uint8_t message[8] = { preamble, 0x00, wire_address | 0x80
                         , (value & 0xff000000) >> 24
                         , (value & 0x00ff0000) >> 16
                         , (value & 0x0000ff00) >> 8
                         , (value & 0x000000ff)
                         , 0 };

    for (size_t m = 0; m < 7; m++) {
        uart_tmc2209_put_byte(message[m], &message[7]);
    }
    uart_tmc2209_put_byte(message[7], NULL);

    sleep_ms(1);
}

bool accounted_write(uint8_t reg, uint32_t value) {

    uint32_t dgram_value;
    bool read_valid = read_register(IFCNT, &dgram_value);
    uint8_t dgrams = dgram_value & 0xff;
    if (read_valid) {
        write_register(reg, value);
    }
    read_valid |= read_register(IFCNT, &dgram_value);
    if (read_valid) {
        uint8_t dgrams_after = dgram_value & 0xff;
        return ++dgrams == dgrams_after;
    }
    return false;
}

void assert_stepper_api() {
    uint32_t value;
    if (read_register(IOIN, &value)) {
        assert((value & 0xff000000) == 0x21000000);
    } else {
        assert(!"read failure");
    }
}