import zmq
import serial
import serial.tools.list_ports
import json
import time
import glob
import argparse

# ==========================================
# SERIAL & BLUETOOTH CONFIGURATION
# ==========================================
DEFAULT_BAUD_RATE = 9600
TX_INTERVAL_SEC = 0.10      # 10 Hz (100 ms) continuous loop mirroring turret_dashboard.html
DEFAULT_ZMQ_PORT = 5555


def auto_detect_serial_port():
    """Finds the active Bluetooth RFCOMM or USB serial port automatically."""
    # Prioritize Linux Bluetooth serial ports (/dev/rfcomm0, etc.)
    rfcomm_ports = sorted(glob.glob("/dev/rfcomm*"))
    if rfcomm_ports:
        return rfcomm_ports[0]

    # Fallback to standard USB serial ports
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "rfcomm" in p.device or "USB" in p.device or "ACM" in p.device:
            return p.device
    return None


def main():
    parser = argparse.ArgumentParser(description="ZMQ-to-Bluetooth Serial Bridge for Badminton Turret")
    parser.add_argument("--serial-port", type=str, default=None, help="Serial device path (default: auto-detect /dev/rfcomm0)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD_RATE, help="Serial baud rate (default: 9600)")
    parser.add_argument("--zmq-port", type=int, default=DEFAULT_ZMQ_PORT, help="ZMQ PUB port to connect to (default: 5555)")
    parser.add_argument("--sim", action="store_true", help="Run in simulation mode without hardware serial port")
    args = parser.parse_args()

    # 1. Connect ZMQ Subscriber to Master Controller
    context = zmq.Context()
    sub_socket = context.socket(zmq.SUB)
    sub_socket.connect(f"tcp://localhost:{args.zmq_port}")
    sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
    print(f"[BT-BRIDGE] Subscribed to Master ZMQ on tcp://localhost:{args.zmq_port}")

    # 2. Connect to Bluetooth / Serial Port
    ser = None
    port_name = args.serial_port or auto_detect_serial_port() or "/dev/rfcomm0"

    if not args.sim:
        try:
            ser = serial.Serial(port_name, args.baud, timeout=0.1)
            time.sleep(1.5)  # Allow Arduino auto-reset over serial/BT to settle
            print(f"[BT-BRIDGE] Connected to Arduino over Bluetooth/Serial at {port_name} ({args.baud} baud)")
        except Exception as e:
            print(f"[BT-BRIDGE WARNING] Could not open '{port_name}': {e}")
            print("[BT-BRIDGE WARNING] Falling back to [SIM] dry-run mode so curriculum can continue.")
            ser = None
    else:
        print("[BT-BRIDGE] Running in [SIM] dry-run mode (no serial packets transmitted).")

    # 3. Turret State (Exact mirror of turret_dashboard.html initial state)
    state = {
        "p": 100,   # Pitch (70 - 130)
        "y": 90,    # Yaw (0 - 180)
        "s": 0,     # BLDC Speed (0 - 50)
        "f": 77,    # Feeder Servo Angle (default 77)
        "sh": 0     # Shoot Flag (0 = idle, 1 = fire)
    }

    last_tx_time = 0.0
    last_printed_state = None

    try:
        while True:
            # A. Check for incoming ZMQ state updates from Master Controller (Non-blocking)
            state_changed = False
            while True:
                try:
                    msg = sub_socket.recv_json(flags=zmq.NOBLOCK)
                    # Support both compact keys ('p','y','s','f','sh') and verbose keys
                    if "p" in msg or "pitch" in msg:
                        state["p"] = int(msg.get("p", msg.get("pitch", state["p"])))
                    if "y" in msg or "yaw" in msg:
                        state["y"] = int(msg.get("y", msg.get("yaw", state["y"])))
                    if "s" in msg or "speed" in msg:
                        state["s"] = int(msg.get("s", msg.get("speed", state["s"])))
                    if "f" in msg or "feed" in msg:
                        state["f"] = int(msg.get("f", msg.get("feed", state["f"])))
                    if "sh" in msg or "shoot_flag" in msg:
                        raw_sh = msg.get("sh", msg.get("shoot_flag", 0))
                        state["sh"] = 1 if raw_sh in (1, True, "1") else 0

                    state_changed = True
                except zmq.Again:
                    break

            # B. Transmit at 10Hz (100ms) OR immediately if shoot/state just changed
            now = time.time()
            if state_changed or (now - last_tx_time >= TX_INTERVAL_SEC):
                compact_packet = json.dumps(state, separators=(",", ":")) + "\n"

                if ser is not None and ser.is_open:
                    try:
                        ser.write(compact_packet.encode("utf-8"))
                    except serial.SerialException as e:
                        print(f"[BT-BRIDGE ERROR] Serial write failed: {e}")

                # Log state changes or active fire pulses cleanly without flooding stdout
                current_tuple = (state["p"], state["y"], state["s"], state["f"], state["sh"])
                if current_tuple != last_printed_state:
                    tag = "TX [LIVE]" if ser else "TX [SIM]"
                    fire_marker = " <--- FIRE!" if state["sh"] == 1 else ""
                    print(f"[BT-BRIDGE] {tag}: {compact_packet.strip()}{fire_marker}")
                    last_printed_state = current_tuple

                last_tx_time = now

            time.sleep(0.005)  # 5ms loop sleep to keep CPU usage near zero

    except KeyboardInterrupt:
        pass
    finally:
        # Safety stop before closing serial link
        if ser is not None and ser.is_open:
            try:
                stop_pkt = json.dumps({"p": 100, "y": 90, "s": 0, "f": 77, "sh": 0}, separators=(",", ":")) + "\n"
                ser.write(stop_pkt.encode("utf-8"))
                time.sleep(0.1)
                ser.close()
            except Exception:
                pass
        print("[BT-BRIDGE] Serial link closed.")


if __name__ == "__main__":
    main()