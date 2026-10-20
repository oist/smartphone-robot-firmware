#ifndef HEALTH_H
#define HEALTH_H

#include "pico/types.h"
#include "hardware/i2c.h"

// Why the board last reset, as reported in RP2040_STATE.Health.reset_cause.
#define HEALTH_RESET_WATCHDOG   (1u << 0)  // the main loop hung for WATCHDOG_TIMEOUT_MS
#define HEALTH_RESET_POR        (1u << 1)  // power-on or brown-out
#define HEALTH_RESET_RUN        (1u << 2)  // RUN pin
#define HEALTH_RESET_PSM        (1u << 3)  // debugger / PSM restart
#define HEALTH_RESET_SELF       (1u << 4)  // health_self_reset(): a stop could not reach the motors

// Boot stages, recorded in a watchdog scratch register as on_start() goes, so
// the next boot can say where the previous one died. 0xFF: reached the loop.
enum {
    BOOT_STAGE_EARLY_STOP = 1,
    BOOT_STAGE_LOG,
    BOOT_STAGE_STDIO,
    BOOT_STAGE_I2C,
    BOOT_STAGE_WIRELESS,
    BOOT_STAGE_NCP3901,
    BOOT_STAGE_CHARGER,
    BOOT_STAGE_BUFFERS,
    BOOT_STAGE_PD,
    BOOT_STAGE_GAUGE,
    BOOT_STAGE_DRV8830,
    BOOT_STAGE_ENCODERS,
    BOOT_STAGE_LOOP = 0xFF,
};

#pragma pack(push, 1)
typedef struct {
    uint8_t reset_cause;      // HEALTH_RESET_* bits for the reset before this boot
    uint8_t prev_boot_stage;  // stage the previous boot had reached (0: unknown, e.g. after power-on)
    uint8_t boots;            // boots since power-on (wraps)
    uint16_t motor_write_failures;  // since boot
    uint8_t bus_recoveries;   // I2C bus recoveries since boot (saturates)
    uint8_t motor_link_ok;    // 1 if the last write to both motor drivers succeeded
    uint8_t self_resets;      // consecutive self-resets before this boot
} HEALTH_STATE;
#pragma pack(pop)

void health_boot(void);
void health_stage(uint8_t stage);
void health_loop_tick(void);
void health_fill(HEALTH_STATE *out);

// One write to the motor drivers: both succeeded or not. Returns true when the
// caller should recover the bus (several failures in a row).
bool health_motor_write(bool ok);
void health_bus_recovered(void);
// A stop could not be delivered even after recovering the bus: reset, so the
// boot path coasts the motors. Gives up after a few in a row, so a dead bus
// cannot reset-loop the board forever.
void health_self_reset(void);

// I2C access is shared by both cores (core 1 runs interrupt handlers that
// read the fault and charger chips); every transaction takes this lock.
void i2c_lock(void);
bool i2c_lock_timeout_ms(uint32_t ms);
void i2c_unlock(void);
// Clock a stuck slave off SDA (nine SCL pulses, then a STOP) and re-init.
void i2c_bus_recover(i2c_inst_t *i2c, uint sda, uint scl);

#endif
