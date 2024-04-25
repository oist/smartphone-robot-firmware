#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "robot.h"
#include <string.h>
#include "hardware/adc.h"
#include "STWLC38JRM.h"

static uint8_t send_buf[3] = {0};
static uint8_t return_buf[3] = {0}; // Will read full buffer from registers 0x52 to 0x71

// Enables the external wireless charging module
void STWLC38JRM_init(uint enable_pin, uint vrect_pin){
    gpio_init(enable_pin);
    gpio_set_dir(enable_pin, GPIO_OUT);
    // resets the STWLC38JRM
    gpio_put(enable_pin, 1);
    sleep_ms(100);
    // turns the STWLC38JRM back on
    gpio_put(enable_pin, 0);

    // init ADC1 as analog input (sampling should be done in main loop)
    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(vrect_pin);
}

void STWLC38JRM_shutdown(uint enable_pin) {
    gpio_put(enable_pin, 0);
}

uint16_t STWLC38JRM_adc1()
{
    // Select ADC input 1 (GPIO27)
    adc_select_input(1);

    // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
    uint16_t result = adc_read();
    return result;
    // Save this value, add to a buffer, or merge with some moving avg.
    // rp2040_log("Raw value: 0x%03x, voltage: %f V\n", result, result * conversion_factor);
}

void STWLC38_get_ept_reasons(){
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    send_buf[0] = 0x01;
    send_buf[1] = 0x27;
    i2c_write_error_handling(i2c0, STWLC38_ADDR, send_buf, 2, true);
    i2c_read_error_handling(i2c0, STWLC38_ADDR, return_buf, 3, false);
    rp2040_log("STWLC38 EPT Reason: 0x0127: 0x%02x, 0x0128: 0x%02x, 0x0129: 0x%02x", return_buf[0], return_buf[1], return_buf[2]);
}
