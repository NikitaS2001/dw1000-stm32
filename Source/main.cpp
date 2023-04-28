#include "shell/shell.h"
#include "telem/telem.h"
#include "uwb/uwb.h"

#include "dwm1000/dwm1000.h"

#include "deca_lib/deca_device_api.h"
#include "deca_lib/deca_port.h"

#include <FreeRTOS.h>
#include <task.h>

#include <stdio.h>

void MainTask(void* pvParameter)
{
    /* Start with board specific hardware init. */
    peripherals_init();

    printf("Initializing DWM1000...\r\n");

    /* Reset and initialise DW1000.
     * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
     * performance. */

    reset_DW1000();

    /* Target specific drive of RSTn line into DW1000 low for a period. */
    spi_set_rate_low();

    if (dwt_initialise(DWT_LOADUCODE) == -1)
    {
        printf("DWM1000 init is failed!\r\n");

        // Blink LED when init is failed
        while (true)
        {
            GPIO_SetBits(GPIOC, GPIO_Pin_13);
            vTaskDelay(1000 / portTICK_PERIOD_MS);

            GPIO_ResetBits(GPIOC, GPIO_Pin_13);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }

    spi_set_rate_high();

    /* Configure DW1000. */
    dwt_configure(&config);
    dwt_setleds(1);

    /* Apply default antenna delay value. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    printf("Init pass!\r\n");

    ShellInit();

    xTaskCreate(
        UwbTask,
        "UwbTask",
        configMINIMAL_STACK_SIZE * 10,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL);

#ifndef BEACON

    xTaskCreate(
        TelemTask,
        "TelemTask",
        configMINIMAL_STACK_SIZE,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL);

#endif

    vTaskDelete( NULL );
}

int main(void)
{
    xTaskCreate(
        MainTask,
        "MainTask",
        configMINIMAL_STACK_SIZE,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL);

    vTaskStartScheduler();
}
