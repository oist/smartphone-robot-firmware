// Board health: why it reset, how far the previous boot got, and the state of
// the I2C link to the motor drivers -- reported to the host in every state
// packet (RP2040_STATE.Health). Also the I2C lock and bus recovery.
//
// Written after 2026-10-16, when a base under motor load reset, boot-looped
// and once sat with a wheel driving that no stop could reach, and none of it
// left a trace: the boot log goes out before the host opens the port.

#include "health.h"
#include "pico/stdlib.h"
#include "pico/mutex.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"
#include "hardware/structs/vreg_and_chip_reset.h"

// watchdog scratch 0..3 are ours (the SDK's watchdog_reboot uses 4..7). They
// survive every reset except power-on / brown-out, which clears them.
#define SCRATCH_STAGE 0
#define SCRATCH_BOOTS 1
#define SCRATCH_SELF  2
#define MAGIC 0xB0070000u
#define MAGIC_MASK 0xFFFF0000u

// Consecutive failed motor writes before recovering the bus, and recoveries
// without a success before resetting.
#define FAILS_BEFORE_RECOVERY 3
#define RECOVERIES_BEFORE_RESET 3
#define MAX_SELF_RESETS 3
// A boot that has run the loop this long clears the self-reset count.
#define HEALTHY_AFTER_MS 10000

static HEALTH_STATE health;
static uint32_t consecutive_fails;
static uint32_t recoveries_since_success;
static bool self_resets_cleared;

auto_init_recursive_mutex(i2c_mutex);

static uint32_t scratch_get(int i, uint32_t fallback) {
    uint32_t v = watchdog_hw->scratch[i];
    return (v & MAGIC_MASK) == MAGIC ? (v & 0xFFFF) : fallback;
}

static void scratch_set(int i, uint32_t v) {
    watchdog_hw->scratch[i] = MAGIC | (v & 0xFFFF);
}

void health_boot(void) {
    uint32_t chip_reset = vreg_and_chip_reset_hw->chip_reset;
    uint8_t cause = 0;
    // A watchdog reset does not touch CHIP_RESET, which then still describes
    // the power-on before it; the watchdog's own reason register is cleared
    // by any chip-level reset, so it decides which of the two to read.
    if (watchdog_hw->reason) {
        cause |= HEALTH_RESET_WATCHDOG;
    } else {
        if (chip_reset & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_POR_BITS) cause |= HEALTH_RESET_POR;
        if (chip_reset & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_RUN_BITS) cause |= HEALTH_RESET_RUN;
        if (chip_reset & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_PSM_RESTART_BITS) cause |= HEALTH_RESET_PSM;
    }
    uint32_t self = scratch_get(SCRATCH_SELF, 0);
    // health_self_reset() sets bit 15 just before rebooting.
    if (self & 0x8000) cause |= HEALTH_RESET_SELF;
    health.reset_cause = cause;
    health.self_resets = self & 0xFF;
    scratch_set(SCRATCH_SELF, self & 0xFF);
    health.prev_boot_stage = scratch_get(SCRATCH_STAGE, 0);
    health.boots = scratch_get(SCRATCH_BOOTS, 0) + 1;
    scratch_set(SCRATCH_BOOTS, health.boots);
    health.motor_link_ok = 1;
}

void health_stage(uint8_t stage) {
    scratch_set(SCRATCH_STAGE, stage);
}

void health_loop_tick(void) {
    if (!self_resets_cleared && to_ms_since_boot(get_absolute_time()) > HEALTHY_AFTER_MS) {
        self_resets_cleared = true;
        scratch_set(SCRATCH_SELF, 0);
    }
}

void health_fill(HEALTH_STATE *out) {
    *out = health;
}

bool health_motor_write(bool ok) {
    health.motor_link_ok = ok;
    if (ok) {
        consecutive_fails = 0;
        recoveries_since_success = 0;
        return false;
    }
    if (health.motor_write_failures < 0xFFFF) health.motor_write_failures++;
    return ++consecutive_fails >= FAILS_BEFORE_RECOVERY;
}

void health_bus_recovered(void) {
    consecutive_fails = 0;
    if (health.bus_recoveries < 0xFF) health.bus_recoveries++;
    if (++recoveries_since_success >= RECOVERIES_BEFORE_RESET) {
        health_self_reset();
    }
}

void health_self_reset(void) {
    uint32_t n = scratch_get(SCRATCH_SELF, 0) & 0xFF;
    if (n >= MAX_SELF_RESETS) {
        // Resetting has not helped; keep running and keep retrying instead.
        recoveries_since_success = 0;
        return;
    }
    scratch_set(SCRATCH_SELF, 0x8000 | (n + 1));
    watchdog_reboot(0, 0, 0);
    while (true) tight_loop_contents();
}

void i2c_lock(void) {
    recursive_mutex_enter_blocking(&i2c_mutex);
}

bool i2c_lock_timeout_ms(uint32_t ms) {
    return recursive_mutex_enter_timeout_ms(&i2c_mutex, ms);
}

void i2c_unlock(void) {
    recursive_mutex_exit(&i2c_mutex);
}

void i2c_bus_recover(i2c_inst_t *i2c, uint sda, uint scl) {
    i2c_lock();
    i2c_deinit(i2c);
    gpio_init(sda);
    gpio_set_dir(sda, GPIO_IN);
    gpio_pull_up(sda);
    gpio_init(scl);
    gpio_set_dir(scl, GPIO_OUT);
    gpio_put(scl, 1);
    busy_wait_us(5);
    // A slave stopped mid-byte holds SDA low until it has clocked out the rest
    // of it: up to nine clocks frees it.
    for (int i = 0; i < 9 && !gpio_get(sda); i++) {
        gpio_put(scl, 0);
        busy_wait_us(5);
        gpio_put(scl, 1);
        busy_wait_us(5);
    }
    // STOP: SDA low -> high while SCL is high.
    gpio_put(sda, 0);
    gpio_set_dir(sda, GPIO_OUT);
    busy_wait_us(5);
    gpio_set_dir(sda, GPIO_IN);
    busy_wait_us(5);
    i2c_init(i2c, 400 * 1000);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(scl);
    i2c_unlock();
}
