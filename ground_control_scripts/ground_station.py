#!/usr/bin/env python3
"""
Ground Control Station - Raspberry Pi
Communicates with CDH node via NRF24L01+
Requires SPI enabled: sudo raspi-config -> Interface Options -> SPI -> Enable
"""

import select
import struct
import sys
import termios
import time
import tty

from pyrf24 import RF24, RF24_PA_MAX, RF24_2MBPS, RF24_CRC_8
from rich.console import Group
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

# NRF24 settings (must match CDH node)
CE_PIN = 22
CSN_PIN = 0  # /dev/spidev0.0
CHANNEL = 76  # 2476 MHz
PAYLOAD_SIZE = 8

# Message types from CDH
MSG_BUTTON = 0x01
MSG_TEMP = 0x02
MSG_STATUS = 0x03

# Ground command types
GND_CMD_TOGGLE_CDH_LED = 0x01
GND_CMD_TOGGLE_EPS_LED = 0x02
GND_CMD_STATUS_REQ = 0x03

# Status polling interval
STATUS_INTERVAL = 5.0  # seconds

# Initialize radio
radio = RF24(CE_PIN, CSN_PIN, 10000000)  # 10 MHz SPI speed

# UI state
last_latency_ms = None
last_status = None
last_rx_msg = None
last_tx_msg = None


def setup():
    if not radio.begin():
        print("ERROR: NRF24 radio not responding. Check wiring and SPI.")
        return False

    radio.setPALevel(RF24_PA_MAX)
    radio.setDataRate(RF24_2MBPS)
    radio.setChannel(CHANNEL)
    radio.setPayloadSize(PAYLOAD_SIZE)
    radio.setAddressWidth(5)
    radio.setCRCLength(RF24_CRC_8)
    radio.setAutoAck(True)
    radio.setRetries(0, 3)  # 250us delay, 3 retries

    address = b"\xe7\xe7\xe7\xe7\xe7"
    radio.openReadingPipe(0, address)
    radio.openWritingPipe(address)

    radio.startListening()
    return True


def parse_message(payload):
    """Parse incoming payload from CDH node. Returns True if it was a status response."""
    global last_status, last_rx_msg
    msg_type = payload[0]

    if msg_type == MSG_BUTTON:
        state = payload[1]
        last_rx_msg = f"Button: {'PRESSED' if state else 'RELEASED'}"
        return False

    elif msg_type == MSG_TEMP:
        temp_raw = struct.unpack_from("<h", payload, 1)[0]
        temp_c = temp_raw / 10.0
        last_rx_msg = f"EPS Temperature: {temp_c:.1f} C"
        return False

    elif msg_type == MSG_STATUS:
        cdh_led = payload[1]
        eps_led_raw = payload[2]
        temp_raw = struct.unpack_from("<h", payload, 3)[0]
        temp_c = temp_raw / 10.0
        eps_alive = bool(payload[5])

        last_status = {
            "cdh_led": cdh_led,
            "eps_led": eps_led_raw if eps_alive else None,
            "eps_temp": temp_c if eps_alive else None,
            "eps_alive": eps_alive,
        }
        return True

    else:
        hex_str = " ".join(f"{b:02X}" for b in payload)
        last_rx_msg = f"Unknown: {hex_str}"
        return False


def send_message(data):
    """Send a message to CDH node."""
    global last_latency_ms, last_tx_msg

    payload = bytes(data[:PAYLOAD_SIZE]).ljust(PAYLOAD_SIZE, b"\x00")

    radio.stopListening()
    t_start = time.perf_counter()
    result = radio.write(payload)
    t_end = time.perf_counter()
    radio.startListening()

    latency_ms = (t_end - t_start) * 1000
    last_latency_ms = latency_ms
    hex_str = " ".join(f"{b:02X}" for b in payload)

    if result:
        last_tx_msg = f"{hex_str} ({latency_ms:.1f} ms)"
    else:
        last_tx_msg = f"{hex_str} [FAILED] ({latency_ms:.1f} ms)"

    return result


def get_key():
    """Non-blocking single key read."""
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def build_display():
    """Build the rich terminal UI."""
    # Status panel
    status_table = Table(show_header=False, box=None, padding=(0, 2))
    status_table.add_column("Key", style="bold", width=14)
    status_table.add_column("Value")

    if last_status:
        cdh_led = "[green]ON[/green]" if last_status["cdh_led"] else "[red]OFF[/red]"
        if last_status["eps_alive"]:
            eps_led = "[green]ON[/green]" if last_status["eps_led"] else "[red]OFF[/red]"
            eps_temp = f"{last_status['eps_temp']:.1f} C"
            eps_state = "[green]ALIVE[/green]"
        else:
            eps_led = "[yellow]--[/yellow]"
            eps_temp = "[yellow]--[/yellow]"
            eps_state = "[red]CAN DOWN[/red]"

        status_table.add_row("CDH LED", cdh_led)
        status_table.add_row("EPS LED", eps_led)
        status_table.add_row("EPS Temp", eps_temp)
        status_table.add_row("EPS Status", eps_state)
    else:
        status_table.add_row("", "[dim]Waiting for status...[/dim]")

    latency_str = f"{last_latency_ms:.1f} ms" if last_latency_ms is not None else "[dim]--[/dim]"
    status_table.add_row("Radio Latency", latency_str)

    status_panel = Panel(status_table, title="Status", border_style="blue")

    # Messages panel
    msg_table = Table(show_header=False, box=None, padding=(0, 2))
    msg_table.add_column("Key", style="bold", width=14)
    msg_table.add_column("Value")
    msg_table.add_row("Last RX", last_rx_msg or "[dim]--[/dim]")
    msg_table.add_row("Last TX", last_tx_msg or "[dim]--[/dim]")
    msg_panel = Panel(msg_table, title="Messages", border_style="cyan")

    # Controls panel
    controls = Text()
    controls.append("  [z]", style="bold yellow")
    controls.append(" Toggle EPS LED    ")
    controls.append("[x]", style="bold yellow")
    controls.append(" Toggle CDH LED    ")
    controls.append("[q]", style="bold yellow")
    controls.append(" Quit")
    controls_panel = Panel(controls, title="Controls", border_style="green")

    return Panel(
        Group(status_panel, msg_panel, controls_panel),
        title="[bold]Ground Control Station[/bold]",
        border_style="white",
    )


def main():
    if not setup():
        return

    # Set terminal to cbreak mode for single keypress reading
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())

        last_status_time = 0.0

        with Live(build_display(), refresh_per_second=10, screen=True) as live:
            while True:
                # Check for incoming radio data
                if radio.available():
                    buf = radio.read(PAYLOAD_SIZE)
                    parse_message(buf)
                    live.update(build_display())

                # Send status request every STATUS_INTERVAL seconds
                now = time.time()
                if now - last_status_time >= STATUS_INTERVAL:
                    last_status_time = now
                    send_message(bytes([GND_CMD_STATUS_REQ]))
                    live.update(build_display())

                # Handle keypress
                key = get_key()
                if key == "z":
                    send_message(bytes([GND_CMD_TOGGLE_EPS_LED]))
                    send_message(bytes([GND_CMD_STATUS_REQ]))
                    last_status_time = now
                    live.update(build_display())
                elif key == "x":
                    send_message(bytes([GND_CMD_TOGGLE_CDH_LED]))
                    send_message(bytes([GND_CMD_STATUS_REQ]))
                    last_status_time = now
                    live.update(build_display())
                elif key == "q":
                    break

                time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        radio.powerDown()


if __name__ == "__main__":
    main()
