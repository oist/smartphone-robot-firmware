#ifndef ROBOT_
#define ROBOT_

#include "hardware/i2c.h"

void on_start();
void i2c_start();
void bq27742_g1_init();
void max77642_init();
void max77857_init();
void sn74ahc125rgyr_init();
void quad_encoders_init();
void blink_led(uint8_t blinkCnt, int onTime, int offTime);
void sample_adc_inputs();
void init_queues();
void i2c_read_error_handling(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, size_t len, bool nostop);
void i2c_write_error_handling(i2c_inst_t *i2c, uint8_t addr, const uint8_t *src, size_t len, bool nostop);
void call_queue_try_add(void *func, int32_t arg);
void results_queue_try_add(void *func, int32_t arg);

typedef struct
{
    void *func;
    int32_t data;
} queue_entry_t;

#define CONVERSION_FACTOR _u(3).3f / (1 << 12)
#define GPIO_WIRELESS_AVAILABLE _u(4) // GPIO4
#define GPIO_OTG _u(5) // GPIO5
#define BATTERY_CHARGER_INTERRUPT_PIN _u(6) // GPIO6
#define SN74AHC125RGYR_GPIO _u(16) // GPIO16
#define MAX77958_INTB _u(7) // GPIO7

#define DRV8830_FAULT1 _u(10) // GPIO10
#define DRV8830_FAULT2 _u(11) // GPIO11

// I2C defines
// Use I2C0 on GPIO0 (SDA) and GPIO1 (SCL) running at 400KHz.
#define I2C_SDA0 _u(0)
#define I2C_SCL0 _u(1)
#define I2C_SDA1 _u(2)
#define I2C_SCL1 _u(3)
#define I2C_TIMEOUT _u(1000000) // 1 second timeout

#define ADC0 _u(26)

#define WIRELESS_CHG_EN _u(9) // WRM483265-10F5-12V-G (U1) enable pin via GPIO9
#define WIRELESS_CHG_AVAILABLE _u(4)
#define USB_VOLTAGE _u(26)
#define CHARGER_INT _u(6)
#define MODE _u(0x16) // CHG_CNFG_00

// Quad encoder pins
#define ENCODER_1_CHANNEL_A _u(12)
#define ENCODER_1_CHANNEL_B _u(13)
#define ENCODER_2_CHANNEL_A _u(14)
#define ENCODER_2_CHANNEL_B _u(15)

#endif
