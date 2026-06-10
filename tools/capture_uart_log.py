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
DEFAULT_OUTPUT = "/tmp/max77958-diag.log"
DIAG_END_MARKERS = (
    "MAX77958_DIAG: end status poll",
    "EXT_MAX77958_I2C1_TEST: end",
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


def last_match(lines, pattern):
    regex = re.compile(pattern)
    for line in reversed(lines):
        match = regex.search(line)
        if match:
            return line, match
    return None, None


def summarize(text, output_path, marker_seen, flash_process):
    lines = text.splitlines()
    print(f"\nLog written to: {output_path}")

    if flash_process is not None:
        status = flash_process.poll()
        if status is None:
            status = flash_process.wait()
        print(f"make flash exit code: {status}")

    print(f"diagnostic end marker seen: {'yes' if marker_seen else 'no'}")

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

    cc0_lines = [line for line in lines if "MAX77958_DIAG" in line and " CC0=" in line]
    cc1_lines = [line for line in lines if "MAX77958_DIAG" in line and " CC1=" in line]
    pd_lines = [line for line in lines if "MAX77958_DIAG" in line and " PD0=" in line]
    gpio_lines = [line for line in lines if "MAX77958_DIAG" in line and " GPIO53=" in line]

    for label, selected in (
        ("final CC0", cc0_lines),
        ("final CC1", cc1_lines),
        ("final PD", pd_lines),
        ("final GPIO", gpio_lines),
    ):
        print(f"{label}: {selected[-1] if selected else 'not found'}")

    states = []
    for line in cc0_lines:
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

    return 0 if text else 1


def main():
    parser = argparse.ArgumentParser(
        description="Capture MAX77958 UART diagnostics from the debug probe serial port."
    )
    parser.add_argument("--device", default=DEFAULT_DEVICE, help=f"serial device, default {DEFAULT_DEVICE}")
    parser.add_argument("--baud", type=int, default=115200, help="UART baud rate, default 115200")
    parser.add_argument("--timeout", type=float, default=30.0, help="capture timeout in seconds, default 30")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help=f"log output path, default {DEFAULT_OUTPUT}")
    parser.add_argument("--flash", action="store_true", help="run make flash after opening UART")
    parser.add_argument("--no-drain", action="store_true", help="do not discard stale bytes before capture")
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
        text, marker_seen = capture_uart(
            fd,
            args.output,
            args.timeout,
            DIAG_END_MARKERS,
            flash_process,
        )
    finally:
        os.close(fd)

    return summarize(text, args.output, marker_seen, flash_process)


if __name__ == "__main__":
    raise SystemExit(main())
