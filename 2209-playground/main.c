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

    disable_motor();

    while (true) {

        uint32_t reg;
        if (read_register(GCONF, &reg)) {
            uint32_t gconf = reg & 0x3FF;
            uint32_t pdn_diable = (gconf & 0x40) >> 6;
            printf("GCONF %x, PDN_DISABLE %x\n", gconf, pdn_diable);

            if (accounted_write(GCONF, gconf | 0x40)) {
                printf("written GCONF\n");
            }
        }

        if (read_register(GCONF, &reg)) {
            uint32_t gconf = reg & 0x3FF;
            uint32_t pdn_diable = (gconf & 0x40) >> 6;
            printf("GCONF %x, PDN_DISABLE %x\n", gconf, pdn_diable);
        }

//    dance();

        uint32_t value;
        if (read_register(IOIN, &value)) {
            if ((value & 0xff000000) == 0x21000000) {
                printf("API as expected\n");
            } else {
                printf("unexpected API version\n");
            }
        } else {
            printf("read failure\n");
        }
        sleep_ms(1000);
    }
}
