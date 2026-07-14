#!/usr/bin/env python3
import argparse
import os
import re
import select
import subprocess
import sys
import termios
import time


DEFAULT_DEVICE = "/dev/ttyACM0"
DEFAULT_OUTPUT = "/tmp/firmware-uart.log"
DIAG_END_MARKERS = (
    "EXT_MAX77958_I2C1_TEST: end",
    "DRV8830_SCOPE_TEST END",
)
CC_STATE_NAMES = {
    0: "NO_CONNECTION",
    1: "SINK_ATTACHED",
    2: "SOURCE_ATTACHED",
    3: "AUDIO_ACCESSORY",
    4: "DEBUG_SOURCE",
    5: "ERROR",
    6: "DISABLED",
    7: "DEBUG_SINK",
}

LIVE_FILTERS = (
    "on_start complete",
    "DRV8830_SCOPE_TEST",
    "motor fault",
    "Fault values",
    "MAX77958_DIAG:",
    "CCStat: ccstat changed",
    "Power source ready",
    "PD Message:",
    "opcode_read:",
)

BAUD_RATES = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
}


def configure_serial(fd, baud):
    try:
        baud_const = BAUD_RATES[baud]
    except KeyError:
        raise ValueError(f"unsupported baud rate: {baud}") from None

    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    attrs[4] = baud_const
    attrs[5] = baud_const
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)


def open_serial(device, baud):
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    configure_serial(fd, baud)
    return fd


def drain_serial(fd, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        readable, _, _ = select.select([fd], [], [], 0.05)
        if not readable:
            continue
        try:
            os.read(fd, 4096)
        except BlockingIOError:
            pass


def start_flash(enabled):
    if not enabled:
        return None

    print("Running make flash while UART capture is active...", flush=True)
    return subprocess.Popen(["make", "flash"])


def capture_uart(fd, output_path, timeout, stop_markers, flash_process):
    deadline = time.monotonic() + timeout
    chunks = []
    marker_seen = False

    with open(output_path, "wb") as output:
        while time.monotonic() < deadline:
            readable, _, _ = select.select([fd], [], [], 0.1)
            if readable:
                try:
                    chunk = os.read(fd, 4096)
                except BlockingIOError:
                    chunk = b""

                if chunk:
                    output.write(chunk)
                    output.flush()
                    chunks.append(chunk)
                    text = b"".join(chunks).decode("utf-8", "replace")
                    if any(marker in text for marker in stop_markers):
                        marker_seen = True
                        break

            if flash_process is not None and flash_process.poll() is not None:
                if time.monotonic() >= deadline:
                    break

    return b"".join(chunks).decode("utf-8", "replace"), marker_seen


def line_is_relevant(line):
    return any(token in line for token in LIVE_FILTERS)


def parse_role_line(line):
    state_match = re.search(r"state=(\d+)", line)
    power_match = re.search(r"pcb_power=([A-Z_]+)", line)
    data_match = re.search(r"pcb_data=([A-Z_]+)", line)
    ready_match = re.search(r"pd_ready=(\d+)", line)

    return {
        "state": int(state_match.group(1)) if state_match else None,
        "pcb_power": power_match.group(1) if power_match else None,
        "pcb_data": data_match.group(1) if data_match else None,
        "pd_ready": int(ready_match.group(1)) if ready_match else None,
    }


def consume_uart_lines(fd, output, deadline, line_callback):
    chunks = []
    pending = ""

    while time.monotonic() < deadline:
        readable, _, _ = select.select([fd], [], [], 0.1)
        if not readable:
            continue

        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            chunk = b""

        if not chunk:
            continue

        output.write(chunk)
        output.flush()
        chunks.append(chunk)

        pending += chunk.decode("utf-8", "replace")
        lines = pending.splitlines(keepends=True)
        pending = ""
        if lines and not lines[-1].endswith(("\n", "\r")):
            pending = lines.pop()

        for raw_line in lines:
            line = raw_line.strip()
            if not line:
                continue
            if line_is_relevant(line):
                print(line, flush=True)
            if line_callback(line):
                return b"".join(chunks).decode("utf-8", "replace"), True

    return b"".join(chunks).decode("utf-8", "replace"), False


def wait_for_startup(fd, output, timeout):
    print("Waiting for on_start complete before interactive cycles...", flush=True)
    deadline = time.monotonic() + timeout

    def saw_startup(line):
        return "on_start complete" in line

    return consume_uart_lines(fd, output, deadline, saw_startup)


def wait_for_detach(fd, output, step_timeout):
    status = {
        "vbus_off": False,
        "detached": False,
        "last_role": None,
    }
    deadline = time.monotonic() + step_timeout

    def saw_detach(line):
        if "MAX77958_DIAG: setting VBUS GPIO4/GPIO5 off" in line:
            status["vbus_off"] = True

        if " pcb_power=" in line or "state=" in line:
            role = parse_role_line(line)
            status["last_role"] = line
            if role["state"] == 0 or role["pcb_power"] == "UNKNOWN":
                status["detached"] = True

        return status["detached"] and status["vbus_off"]

    text, ok = consume_uart_lines(fd, output, deadline, saw_detach)
    return text, ok, status


def wait_for_reattach(fd, output, step_timeout):
    status = {
        "vbus_on": False,
        "desired": False,
        "last_role": None,
    }
    deadline = time.monotonic() + step_timeout

    def saw_desired(line):
        if "MAX77958_DIAG: setting VBUS GPIO4/GPIO5 on" in line:
            status["vbus_on"] = True

        if " pcb_power=" in line:
            role = parse_role_line(line)
            status["last_role"] = line
            status["desired"] = (
                role["state"] == 2
                and role["pcb_power"] == "SOURCE"
                and role["pcb_data"] == "UFP_DEVICE"
                and role["pd_ready"] == 1
            )

        return status["desired"] and status["vbus_on"]

    text, ok = consume_uart_lines(fd, output, deadline, saw_desired)
    return text, ok, status


def desired_baseline_from_text(text):
    status = {
        "vbus_on": False,
        "desired": False,
        "last_role": None,
    }
    lines = text.splitlines()
    last_on_index = -1
    last_off_index = -1

    for index, line in enumerate(lines):
        if "MAX77958_DIAG: setting VBUS GPIO4/GPIO5 on" in line:
            last_on_index = index
        if "MAX77958_DIAG: setting VBUS GPIO4/GPIO5 off" in line:
            last_off_index = index
        if " pcb_power=" in line:
            status["last_role"] = line
            role = parse_role_line(line)
            status["desired"] = (
                role["state"] == 2
                and role["pcb_power"] == "SOURCE"
                and role["pcb_data"] == "UFP_DEVICE"
                and role["pd_ready"] == 1
            )

    status["vbus_on"] = last_on_index > last_off_index
    return status["desired"] and status["vbus_on"], status


def run_interactive_cycles(fd, output_path, timeout, flash_process, cycles, step_timeout):
    chunks = []
    marker_seen = False

    with open(output_path, "wb") as output:
        startup_text, startup_seen = wait_for_startup(fd, output, timeout)
        chunks.append(startup_text.encode("utf-8", "replace"))

        if not startup_seen:
            print(f"ERROR: on_start complete was not seen within {timeout:.1f}s", flush=True)
            text = b"".join(chunks).decode("utf-8", "replace")
            return text, marker_seen, 1

        baseline_ok, baseline_status = desired_baseline_from_text(startup_text)
        if not baseline_ok:
            print(f"Waiting {step_timeout:.1f}s for post-startup attached baseline...", flush=True)
            baseline_text, baseline_ok, baseline_status = wait_for_reattach(fd, output, step_timeout)
            chunks.append(baseline_text.encode("utf-8", "replace"))

        if not baseline_ok:
            print(f"ERROR: startup baseline did not reach SOURCE + UFP + READY + VBUS on within {step_timeout:.1f}s")
            if baseline_status["last_role"]:
                print(f"last role line: {baseline_status['last_role']}")
            print(f"vbus_on_seen: {'yes' if baseline_status['vbus_on'] else 'no'}")
            text = b"".join(chunks).decode("utf-8", "replace")
            return text, marker_seen, 1

        print("Startup baseline confirmed: SOURCE + UFP + READY + VBUS on.")

        for cycle in range(1, cycles + 1):
            input(f"\nCycle {cycle}/{cycles}: press Enter when ready to detach. ")
            print("Detach the phone now.", flush=True)
            detach_text, detach_ok, detach_status = wait_for_detach(fd, output, step_timeout)
            chunks.append(detach_text.encode("utf-8", "replace"))
            if not detach_ok:
                print(f"ERROR: cycle {cycle} detach did not reach NO_CONNECTION + VBUS off within {step_timeout:.1f}s")
                if detach_status["last_role"]:
                    print(f"last role line: {detach_status['last_role']}")
                print(f"vbus_off_seen: {'yes' if detach_status['vbus_off'] else 'no'}")
                text = b"".join(chunks).decode("utf-8", "replace")
                return text, marker_seen, 1
            print(f"Cycle {cycle}/{cycles}: detach confirmed.")

            input(f"Cycle {cycle}/{cycles}: press Enter when ready to reattach. ")
            print("Reattach the phone now.", flush=True)
            attach_text, attach_ok, attach_status = wait_for_reattach(fd, output, step_timeout)
            chunks.append(attach_text.encode("utf-8", "replace"))
            if not attach_ok:
                print(f"ERROR: cycle {cycle} reattach did not reach SOURCE + UFP + READY + VBUS on within {step_timeout:.1f}s")
                if attach_status["last_role"]:
                    print(f"last role line: {attach_status['last_role']}")
                print(f"vbus_on_seen: {'yes' if attach_status['vbus_on'] else 'no'}")
                text = b"".join(chunks).decode("utf-8", "replace")
                return text, marker_seen, 1
            print(f"Cycle {cycle}/{cycles}: reattach confirmed.")

    text = b"".join(chunks).decode("utf-8", "replace")
    return text, marker_seen, 0


def last_match(lines, pattern):
    regex = re.compile(pattern)
    for line in reversed(lines):
        match = regex.search(line)
        if match:
            return line, match
    return None, None


def parse_last_int(lines, pattern):
    _, match = last_match(lines, pattern)
    return int(match.group(1)) if match else None


def parse_last_str(lines, pattern):
    _, match = last_match(lines, pattern)
    return match.group(1) if match else None


def summarize(text, output_path, marker_seen, flash_process):
    lines = text.splitlines()
    print(f"\nLog written to: {output_path}")

    if flash_process is not None:
        status = flash_process.poll()
        if status is None:
            status = flash_process.wait()
        print(f"make flash exit code: {status}")

    print(f"capture completion marker seen: {'yes' if marker_seen else 'no'}")
    print(f"on_start complete seen: {'yes' if any('on_start complete' in line for line in lines) else 'no'}")

    drv8830_scope_lines = [line for line in lines if "DRV8830_SCOPE_TEST" in line]
    if drv8830_scope_lines:
        drv8830_pre_clear_lines = [line for line in drv8830_scope_lines if "fault_pre_clear" in line]
        drv8830_post_clear_lines = [line for line in drv8830_scope_lines if "fault_post_clear" in line]
        drv8830_nonfault_status_lines = [line for line in drv8830_scope_lines if "nonfault_status_irq=1" in line]
        drv8830_active_fault_lines = [line for line in drv8830_scope_lines if "active_fault=1" in line]
        drv8830_uvlo_only_lines = [
            line for line in drv8830_pre_clear_lines
            if "UVLO=1" in line and "FAULT=0" in line
        ]
        drv8830_real_uvlo_lines = [
            line for line in drv8830_pre_clear_lines
            if "UVLO=1" in line and "FAULT=1" in line
        ]
        drv8830_motor_fault_lines = [
            line for line in lines
            if "Left motor fault" in line or "Right motor fault" in line
        ]

        print("DRV8830 scope test:")
        print(f"  lines observed: {len(drv8830_scope_lines)}")
        print(f"  non-fault status IRQs: {len(drv8830_nonfault_status_lines)}")
        print(f"  active fault IRQs: {len(drv8830_active_fault_lines)}")
        print(f"  UVLO-only pre-clear reads: {len(drv8830_uvlo_only_lines)}")
        print(f"  true UVLO pre-clear reads: {len(drv8830_real_uvlo_lines)}")
        print(f"  generic motor fault logs: {len(drv8830_motor_fault_lines)}")
        print(f"  final pre-clear: {drv8830_pre_clear_lines[-1] if drv8830_pre_clear_lines else 'not found'}")
        print(f"  final post-clear: {drv8830_post_clear_lines[-1] if drv8830_post_clear_lines else 'not found'}")

    external_lines = [line for line in lines if "EXT_MAX77958_I2C1_TEST" in line]
    if external_lines:
        external_status_lines = [line for line in external_lines if " t=" in line and " CC0=" in line]
        external_gpio_lines = [line for line in external_lines if " t=" in line and " GPIO " in line]
        external_event_lines = [line for line in external_lines if " t=" not in line]
        print("external MAX77958 I2C1 test:")
        for line in external_event_lines:
            print(f"  {line}")
        print(f"  final status: {external_status_lines[-1] if external_status_lines else 'not found'}")
        print(f"  final GPIO: {external_gpio_lines[-1] if external_gpio_lines else 'not found'}")

        external_states = []
        for line in external_status_lines:
            match = re.search(r"state=(\d+)", line)
            if match:
                external_states.append(int(match.group(1)))
        if external_states:
            print(f"  CC states observed: {sorted(set(external_states))}")
            print(f"  source attach observed: {'yes' if 2 in external_states else 'no'}")

    _, cc_ctrl = last_match(lines, r"CC_CTRL1 readback = (0x[0-9a-fA-F]+)")
    if cc_ctrl:
        print(f"CC_CTRL1 readback: {cc_ctrl.group(1)}")
    else:
        print("CC_CTRL1 readback: not found")

    cc0_lines = [line for line in lines if re.search(r"MAX77958_DIAG t=\d+ms CC0=", line)]
    cc1_lines = [line for line in lines if "MAX77958_DIAG" in line and " CC1=" in line]
    pd_lines = [line for line in lines if "MAX77958_DIAG" in line and " PD0=" in line]
    state_eval_lines = [line for line in lines if "MAX77958_DIAG: state eval" in line]
    role_lines = [
        line for line in lines
        if "MAX77958_DIAG:" in line
        and " pcb_power=" in line
        and " pcb_data=" in line
        and " pd_ready=" in line
    ]
    pr_swap_lines = [line for line in lines if "MAX77958_DIAG: requesting PR_SWAP" in line]
    dr_swap_lines = [line for line in lines if "MAX77958_DIAG: requesting DR_SWAP" in line]
    vbus_on_lines = [line for line in lines if "MAX77958_DIAG: setting VBUS GPIO4/GPIO5 on" in line]
    vbus_off_lines = [line for line in lines if "MAX77958_DIAG: setting VBUS GPIO4/GPIO5 off" in line]
    gpio_lines = [line for line in lines if "MAX77958_DIAG" in line and " GPIO53=" in line]

    for label, selected in (
        ("final CC0", cc0_lines),
        ("final CC1", cc1_lines),
        ("final PD", pd_lines),
        ("final GPIO", gpio_lines),
    ):
        print(f"{label}: {selected[-1] if selected else 'not found'}")

    print(f"final role line: {role_lines[-1] if role_lines else 'not found'}")
    print(f"final state eval: {state_eval_lines[-1] if state_eval_lines else 'not found'}")
    print(f"PR_SWAP requests observed: {len(pr_swap_lines)}")
    print(f"DR_SWAP requests observed: {len(dr_swap_lines)}")
    print(f"VBUS on commands observed: {len(vbus_on_lines)}")
    print(f"VBUS off commands observed: {len(vbus_off_lines)}")

    states = []
    for line in cc0_lines + role_lines:
        match = re.search(r"state=(\d+)", line)
        if match:
            states.append(int(match.group(1)))

    if states:
        nonzero_states = sorted(set(state for state in states if state != 0))
        source_seen = 2 in states
        print(f"CC states observed: {sorted(set(states))}")
        print(f"source attach observed: {'yes' if source_seen else 'no'}")
        if nonzero_states and not source_seen:
            print(f"nonzero CC states observed: {nonzero_states}")
    else:
        print("CC states observed: none")

    final_state = parse_last_int(cc0_lines, r"state=(\d+)")
    if final_state is None:
        final_state = parse_last_int(role_lines, r"state=(\d+)")
    final_data = parse_last_int(pd_lines, r"data=(\d+)")
    final_psrdy = parse_last_int(pd_lines, r"psrdy=(\d+)")
    if final_data is None:
        final_data = parse_last_int(state_eval_lines, r"data=(\d+)")
    if final_psrdy is None:
        final_psrdy = parse_last_int(state_eval_lines, r"psrdy=(\d+)")
    final_pcb_power = parse_last_str(role_lines, r"pcb_power=([A-Z_]+)")
    final_pcb_data = parse_last_str(role_lines, r"pcb_data=([A-Z_]+)")
    final_pd_ready = parse_last_int(role_lines, r"pd_ready=(\d+)")
    final_gpio_match = last_match(gpio_lines, r"g4=(\d+)/(\d+) g5=(\d+)/(\d+)")[1]

    if final_state is not None or final_data is not None or final_psrdy is not None or final_gpio_match or vbus_on_lines or vbus_off_lines or final_pcb_power:
        source_attached = final_state == 2
        pcb_source = final_pcb_power == "SOURCE" if final_pcb_power is not None else source_attached
        pcb_ufp = final_pcb_data == "UFP_DEVICE" if final_pcb_data is not None else final_data == 0
        pd_ready = final_pd_ready == 1 if final_pd_ready is not None else final_psrdy == 1
        vbus_enabled = None
        if final_gpio_match:
            g4_dir, g4_out, g5_dir, g5_out = (int(group) for group in final_gpio_match.groups())
            vbus_enabled = g4_dir == 1 and g4_out == 1 and g5_dir == 1 and g5_out == 1
        elif vbus_on_lines or vbus_off_lines:
            last_on_index = max((index for index, line in enumerate(lines) if line in vbus_on_lines), default=-1)
            last_off_index = max((index for index, line in enumerate(lines) if line in vbus_off_lines), default=-1)
            vbus_enabled = last_on_index > last_off_index

        print(
            "final role: "
            f"cc={CC_STATE_NAMES.get(final_state, 'UNKNOWN' if final_state is not None else 'not found')} "
            f"pcb_power={final_pcb_power or 'not found'} "
            f"pcb_data={final_pcb_data or 'not found'} "
            f"pd_ready={'yes' if pd_ready else 'no' if final_pd_ready is not None or final_psrdy is not None else 'not found'} "
            f"vbus_enabled={'yes' if vbus_enabled else 'no' if vbus_enabled is not None else 'not found'}"
        )
        print(
            "desired phone-control state: "
            f"{'yes' if pcb_source and pd_ready and pcb_ufp and vbus_enabled else 'no'}"
        )

    return 0 if text else 1


def main():
    parser = argparse.ArgumentParser(
        description="Capture firmware UART diagnostics from the debug probe serial port."
    )
    parser.add_argument("--device", default=DEFAULT_DEVICE, help=f"serial device, default {DEFAULT_DEVICE}")
    parser.add_argument("--baud", type=int, default=115200, help="UART baud rate, default 115200")
    parser.add_argument("--timeout", type=float, default=30.0, help="capture timeout in seconds, default 30")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help=f"log output path, default {DEFAULT_OUTPUT}")
    parser.add_argument("--flash", action="store_true", help="run make flash after opening UART")
    parser.add_argument("--no-drain", action="store_true", help="do not discard stale bytes before capture")
    parser.add_argument("--interactive-cycles", type=int, default=0, help="run prompted detach/reattach cycles after on_start complete")
    parser.add_argument("--step-timeout", type=float, default=5.0, help="seconds to wait for each interactive detach or reattach transition")
    args = parser.parse_args()

    try:
        fd = open_serial(args.device, args.baud)
    except OSError as exc:
        print(f"ERROR: could not open {args.device}: {exc}", file=sys.stderr)
        return 1
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    flash_process = None
    try:
        if not args.no_drain:
            drain_serial(fd, 0.25)

        flash_process = start_flash(args.flash)
        if args.interactive_cycles:
            text, marker_seen, interactive_status = run_interactive_cycles(
                fd,
                args.output,
                args.timeout,
                flash_process,
                args.interactive_cycles,
                args.step_timeout,
            )
        else:
            interactive_status = 0
            text, marker_seen = capture_uart(
                fd,
                args.output,
                args.timeout,
                DIAG_END_MARKERS,
                flash_process,
            )
    finally:
        os.close(fd)

    summary_status = summarize(text, args.output, marker_seen, flash_process)
    return interactive_status or summary_status


if __name__ == "__main__":
    raise SystemExit(main())
