import zmq
import time
import random
import subprocess
import sys
import os
import signal
import argparse

# ==========================================
# CURRICULUM & SERVO TIMING CONFIGURATION
# ==========================================
PITCH_MIN = 100          # Minimum pitch angle in degrees (matches turret 70-130 range)
PITCH_MAX = 101         # Maximum pitch angle in degrees
PITCH_STEP = 1         # Step size in degrees

SPEED_MIN = 15         # Minimum BLDC ESC speed (%)
SPEED_MAX = 15          # Maximum BLDC ESC speed (%)
SPEED_STEP = 15         # Speed step (%)

CONSTANT_YAW = 90       # Straight ahead (constant indoor yaw)
FEEDER_REST_ANGLE = 71  # Resting angle of feeder servo (mirrored from dashboard)
SHOOT_HOLD_SEC = 0.25   # Crisp 250ms sh=1 pulse (2-3 packets at 10Hz; avoids double-triggering)
SPOOL_DELAY_SEC = 1.5   # Time given for pitch servo & BLDCs to settle on new config

ZMQ_CMD_PORT = 5555     # Master PUB -> Bluetooth Bridge SUB & Vision Pipeline SUB
ZMQ_TEL_PORT = 5556     # Vision Pipeline PUB -> Master SUB (shot confirmation telemetry)


def generate_curriculum(pitch_min, pitch_max, pitch_step, speed_min, speed_max, speed_step):
    """Generates a randomized Pitch x Speed grid to evenly distribute shuttle wear."""
    configs = []
    for pitch in range(pitch_min, pitch_max + 1, pitch_step):
        for speed in range(speed_min, speed_max + 1, speed_step):
            configs.append({"pitch": pitch, "speed": speed})
    random.shuffle(configs)
    return configs


def launch_child_processes(serial_port, sim_mode):
    """Spawns the Bluetooth Serial Bridge and RealSense Vision Pipeline as child processes."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    bridge_script = os.path.join(script_dir, "bluetooth_bridge.py")
    vision_script = os.path.join(script_dir, "turret_vision_pipeline.py")

    children = []

    # 1. Launch Bluetooth / Serial ZMQ Subscriber
    bridge_cmd = [sys.executable, bridge_script, "--zmq-port", str(ZMQ_CMD_PORT)]
    if serial_port:
        bridge_cmd.extend(["--serial-port", serial_port])
    if sim_mode:
        bridge_cmd.append("--sim")

    print(f"[MASTER] Starting Bluetooth Bridge: {' '.join(bridge_cmd)}")
    children.append(subprocess.Popen(bridge_cmd))

    # 2. Launch OpenCV / RealSense Vision Pipeline
    if os.path.exists(vision_script):
        vision_cmd = [sys.executable, vision_script]
        print(f"[MASTER] Starting Vision Pipeline: {' '.join(vision_cmd)}")
        children.append(subprocess.Popen(vision_cmd))
    else:
        print(f"[WARNING] '{vision_script}' not found in directory. Running controller + bridge only.")

    return children


def cleanup_processes(children, pub_socket=None):
    """Safely stops motors and terminates all child processes cleanly."""
    print("\n[MASTER] Shutting down system safely...")
    if pub_socket is not None:
        try:
            pub_socket.send_json({
                "command": "SET",
                "p": 100,
                "y": CONSTANT_YAW,
                "s": 0,
                "f": FEEDER_REST_ANGLE,
                "sh": 0
            })
            time.sleep(0.25)
        except Exception:
            pass

    for proc in children:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
    print("[MASTER] All child processes stopped. Goodbye!")


def main():
    parser = argparse.ArgumentParser(description="Master Curriculum & Process Controller for Badminton Turret")
    parser.add_argument("--serial-port", type=str, default=None, help="Bluetooth/Serial port (e.g., /dev/rfcomm0)")
    parser.add_argument("--sim", action="store_true", help="Run Bluetooth bridge in dry-run simulation mode")
    parser.add_argument("--pitch-min", type=int, default=PITCH_MIN, help="Minimum pitch angle (deg)")
    parser.add_argument("--pitch-max", type=int, default=PITCH_MAX, help="Maximum pitch angle (deg)")
    parser.add_argument("--pitch-step", type=int, default=PITCH_STEP, help="Pitch step size (deg)")
    parser.add_argument("--speed-min", type=int, default=SPEED_MIN, help="Minimum ESC speed (%%)")
    parser.add_argument("--speed-max", type=int, default=SPEED_MAX, help="Maximum ESC speed (%%)")
    parser.add_argument("--speed-step", type=int, default=SPEED_STEP, help="ESC speed step (%%)")
    args = parser.parse_args()

    # 1. Setup ZMQ Master Publisher (Commands) & Telemetry Subscriber (Vision Feedback)
    context = zmq.Context()
    pub_socket = context.socket(zmq.PUB)
    pub_socket.bind(f"tcp://*:{ZMQ_CMD_PORT}")

    tel_socket = context.socket(zmq.SUB)
    tel_socket.connect(f"tcp://localhost:{ZMQ_TEL_PORT}")
    tel_socket.setsockopt_string(zmq.SUBSCRIBE, "")

    # 2. Spawn child nodes (File 2: Bluetooth Bridge, File 3: Vision Pipeline)
    children = launch_child_processes(args.serial_port, args.sim)

    def signal_handler(sig, frame):
        cleanup_processes(children, pub_socket)
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("[MASTER] Waiting 4.0 seconds for RealSense & ZMQ nodes to initialize...")
    time.sleep(4.0)

    curriculum = generate_curriculum(
        args.pitch_min, args.pitch_max, args.pitch_step,
        args.speed_min, args.speed_max, args.speed_step
    )
    total_configs = len(curriculum)

    print("\n============================================================")
    print(f"  CURRICULUM READY: {total_configs} Randomized Points")
    print(f"  Pitch: {args.pitch_min}° to {args.pitch_max}° (step {args.pitch_step}°)")
    print(f"  Speed: {args.speed_min}% to {args.speed_max}% (step {args.speed_step}%)")
    print("============================================================")

    try:
        for idx, cfg in enumerate(curriculum):
            pitch = cfg["pitch"]
            speed = cfg["speed"]
            shots_at_point = 0

            print(f"\n>>> [Point {idx + 1}/{total_configs}] Setting PITCH = {pitch}° | SPEED = {speed}% | YAW = {CONSTANT_YAW}°")

            # Send SET packet (mirrors dashboard compact state: p, y, s, f, sh=0)
            set_payload = {
                "command": "SET",
                "p": pitch,
                "y": CONSTANT_YAW,
                "s": speed,
                "f": FEEDER_REST_ANGLE,
                "sh": 0
            }
            pub_socket.send_json(set_payload)

            print(f"    Spooling motors & aligning servo ({SPOOL_DELAY_SEC}s)...")
            time.sleep(SPOOL_DELAY_SEC)

            while True:
                cmd = input(
                    f"    [Point {idx + 1}/{total_configs} | Shots: {shots_at_point}] "
                    f"Press [ENTER] to SHOOT, [N + ENTER] for NEXT point, [Q + ENTER] to QUIT: "
                ).strip().lower()

                if cmd == "n":
                    print(f"    Moving on from Pitch={pitch}°, Speed={speed}% (Total shots logged: {shots_at_point}).")
                    break
                elif cmd == "q":
                    cleanup_processes(children, pub_socket)
                    return
                else:
                    # Flush any stale telemetry messages before firing
                    while True:
                        try:
                            tel_socket.recv_json(flags=zmq.NOBLOCK)
                        except zmq.Again:
                            break

                    shots_at_point += 1
                    print(f"    * FIRING SHOT #{shots_at_point} (P={pitch}°, S={speed}%) *")

                    # 1. Trigger SHOOT (sh=1) -> Mirrors dashboard press & notifies Vision Pipeline
                    shoot_payload = {
                        "command": "SHOOT",
                        "p": pitch,
                        "y": CONSTANT_YAW,
                        "s": speed,
                        "f": FEEDER_REST_ANGLE,
                        "sh": 1
                    }
                    pub_socket.send_json(shoot_payload)

                    # Hold sh=1 for SHOOT_HOLD_SEC so Arduino servo completes feeder stroke
                    time.sleep(SHOOT_HOLD_SEC)

                    # 2. Release SHOOT (sh=0) -> Mirrors dashboard release
                    release_payload = {
                        "command": "SET",
                        "p": pitch,
                        "y": CONSTANT_YAW,
                        "s": speed,
                        "f": FEEDER_REST_ANGLE,
                        "sh": 0
                    }
                    pub_socket.send_json(release_payload)

                    # 3. Wait briefly for Vision Pipeline telemetry confirmation (up to 4.2s)
                    print("    Tracking shuttle flight...", end="", flush=True)
                    wait_start = time.time()
                    got_result = False
                    while time.time() - wait_start < 4.2:
                        try:
                            tel = tel_socket.recv_json(flags=zmq.NOBLOCK)
                            if tel.get("status") == "shot_complete":
                                print(
                                    f"\r    [LOGGED] v0={tel.get('v0')} m/s | Angle={tel.get('angle')}° | "
                                    f"Start(z={tel.get('start_z_cam')}m, y={tel.get('start_y_cam')}m) | "
                                    f"Range(wrt cam)={tel.get('landing_dist_cam')}m"
                                )
                                got_result = True
                                break
                        except zmq.Again:
                            time.sleep(0.05)

                    if not got_result:
                        print("\r    [INFO] Shot cycle finished (no vision telemetry received or running headless).")

        print("\n[SUCCESS] Full curriculum completed! All data saved to 'shuttle_flight_data.csv'.")
    finally:
        cleanup_processes(children, pub_socket)


if __name__ == "__main__":
    main()
