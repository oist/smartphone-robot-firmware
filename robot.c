#include <stdio.h>
#include "pico/stdlib.h"
#include "robot.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "max77976.h"
#include "max77857.h"
#include "wrm483265_10f5_12v_g.h"
#include "ncp3901.h"
#include "sn74ahc125rgyr.h"
#include "max77958.h"

int main()
{
    on_start();

    // Wait forever
    while (1)
    {
        sample_adc_inputs();
        printf("sampling ..\n");
        sleep_ms(500);
    }

    return 0;
}

void on_start(){
    stdio_init_all();
    // Waiting to make sure I can catch it within minicom
    sleep_ms(3000);
    i2c_start();
    adc_init();
    wrm483265_10f5_12v_g_init(WIRELESS_CHG_EN);
    ncp3901_init(GPIO_WIRELESS_AVAILABLE, GPIO_OTG);
    max77976_init(BATTERY_CHARGER_INTERRUPT_PIN);
    sn74ahc125rgyr_init(SN74AHC125RGYR_GPIO);
    max77958_init(MAX77958_INTB);
    // Be sure to do this last
    sn74ahc125rgyr_on_end_of_start(SN74AHC125RGYR_GPIO);
}

void i2c_start()
{
    // I2C Initialisation. Using it at 400Khz.

    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(I2C_SDA0, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL0, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA0);
    gpio_pull_up(I2C_SCL0);

    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(I2C_SDA1, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL1, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA1);
    gpio_pull_up(I2C_SCL1);
}

void blink_led(uint8_t blinkCnt, int onTime, int offTime)
{
#ifndef PICO_DEFAULT_LED_PIN
#warning blink example requires a board with a regular LED
#else
    const uint8_t LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    uint8_t i = 0;
    while (i < blinkCnt)
    {
        printf("Hello blink\n");
        gpio_put(LED_PIN, 1);
        sleep_ms(onTime);
        gpio_put(LED_PIN, 0);
        sleep_ms(offTime);
        i++;
    }
#endif
}

//---------------------------------------------------------------------
// Initialization Methods
//---------------------------------------------------------------------

void max77642_init() {}

void quad_encoders_init() {}

//---------------------------------------------------------------------
// Shutdown Methods
//---------------------------------------------------------------------

void max77642_shutdown() {}

//---------------------------------------------------------------------
// Run Loop Methods
//---------------------------------------------------------------------
void sample_adc_inputs(){
    ncp3901_adc0();
}

void drv8830drcr_set_moto_lvl(){
}

//---------------------------------------------------------------------
// Interrupt Callbacks
//---------------------------------------------------------------------

void quad_encoders_callback(){
    // GPIO12-15 monitor past and current states to determine counts
}
