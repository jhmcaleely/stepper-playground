#include <stdio.h>
#include "pico/stdlib.h"

#include "stepper.h"


int main()
{
    stdio_init_all();

    printf("Init complete\n");

    init_stepper();
    enable_motor();

//    rotate(away);
//    rotate(toward);

//    dance();

    send_read_request_uart_hw(0x06);

    while (true) {
        send_read_request_uart_hw(0x06);
        printf("Hello, world!\n");
        sleep_ms(1000);
    }
}
