#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"

#include "pio-2209.pio.h"

#include "stepper.h"

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    uart_tx_program_init(pio, sm, offset, pin, 50000);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
}



int main()
{
    stdio_init_all();

    init_stepper();
    enable_motor();

    rotate(away);
    rotate(toward);

    dance();

    // PIO Blinking example
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &uart_tx_program);
    printf("Loaded program at %d\n", offset);
    
    #ifdef PICO_DEFAULT_LED_PIN
    blink_pin_forever(pio, 0, offset, PICO_DEFAULT_LED_PIN, 3);
    #else
    blink_pin_forever(pio, 0, offset, 6, 3);
    #endif
    // For more pio examples see https://github.com/raspberrypi/pico-examples/tree/master/pio


    send_read_request(0x00);

    while (true) {
        printf("Hello, world!\n");
        sleep_ms(1000);
    }
}
