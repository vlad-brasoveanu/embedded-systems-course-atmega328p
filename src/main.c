#include "gimbal.h"
#include "gpio.h"
#include "bsp.h"
#include "nano.h"

int main(void)
{
    /* NOTE: Timer0 ISR (system tick) is enabled inside Timer0_Init(), which
     * is the first thing Gimbal_Init() calls. Interrupts (sei) are enabled
     * there as well. LSM6DSO FIFO is polled – no external INT pin required. */

    /* Halt with LED on if any sensor fails WHO_AM_I or I2C handshake */
    if (!Gimbal_Init()) {
        GPIO_Init(LED_BUILTIN, GPIO_OUTPUT);
        GPIO_Write(LED_BUILTIN, GPIO_HIGH);
        while (1);  /* halt – check wiring */
    }

    while (1) {
        Gimbal_Update();
    }
}
