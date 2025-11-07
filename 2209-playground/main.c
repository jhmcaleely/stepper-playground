#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"

#include "stepper.h"


int main()
{
    stdio_init_all();
    printf("Init complete\n");

    sleep_ms(100);

    init_stepper();
    enable_motor();

    rotate(away);
    rotate(toward);

//    dance();

    send_read_request_pio(0x00);

    while (true) {
        printf("Hello, world!\n");
        sleep_ms(1000);
    }
}
