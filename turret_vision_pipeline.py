import pyrealsense2 as rs
import numpy as np
import cv2
import time
import csv
import os
import zmq
import math

# ==========================================
# 1. CONFIGURATION & CALIBRATED GEOMETRY
# ==========================================
DEPTH_CAP_ACQ = 2.80             # Max depth (m) for initial shuttle lock in launch corridor
DEPTH_CAP_FLIGHT = 10.0          # Max depth (m) for visualization & baseline depth bounds
MIN_DEPTH_METERS = 0.30          # Ignore anything closer than 30cm to the lens
MIN_CONTOUR_AREA_ACQ = 20        # Minimum depth blob area (px) during launch acquisition
MAX_CONTOUR_AREA_ACQ = 2200      # Maximum depth blob area (px) during launch acquisition

# Fixed 2.0s Timer + Lower-Feed Change Comparison Parameters
FIXED_LANDING_TIMER_SEC = 3.0    # Wait 2.0s after SHOOT, then compare lower feed changes
ACQ_MAX_WINDOW_SEC = 0.65        # Max time spent locking initial 3D launch vector before timer wait
ROI_CLIP_LR = 0.3               # Clip 25% from both left and right edges for lower landing ROI
ROI_BOTTOM_FRAC = 0.35           # Use only the bottom 35% of the frame for the lower landing ROI
RGB_DIFF_THRESH = 22             # RGB channel diff threshold between t~0.5s ref and t=2.0s frame
DEPTH_DIFF_THRESH_M = 0.10       # Depth protrusion threshold (10cm closer than empty floor)
MIN_LANDED_AREA_RGB = 10         # Minimum RGB-only blob area (px) on the floor
MIN_LANDED_AREA_DEPTH = 5        # Minimum depth-supported blob area (px) for distant shuttles
MAX_LANDED_AREA_PX = 950         # Maximum blob area (px) to reject shadows/people

LIGHT_SAT_THRESH = 228           # Saturated ceiling light threshold (upper ceiling only)
DEPTH_FPS = 30                   # Matched 1:1 with HD 1280x720 COLOR_FPS for zero syncer frame drops
COLOR_FPS = 30                   # Full HD 1280x720 runs natively at 30 FPS on D435/D455
COLOR_WIDTH = 1280               # Fullf HD width (1st priority for maximum shuttle clarity)
COLOR_HEIGHT = 720               # Full HD height (1st priority for maximum shuttle clarity)
DEPTH_WIDTH = 848                # Depth width (aligned automatically to HD color stream)
DEPTH_HEIGHT = 480               # Depth height (aligned automatically to HD color stream)
RGB_MANUAL_EXPOSURE_US = 20000    # Fast 3.0 ms manual shutter (in 100us units -> 30) to eliminate motion blur
RGB_MANUAL_GAIN = 96             # Boost sensor gain to compensate for fast 3.0 ms shutter indoors
ROTATION_CW = True               # Clockwise 90-deg rotation from back of camera
CAMERA_HEIGHT = 1.32             # True lens height above floor in METERS
CAMERA_TILT_DEG = 3.96           # Upward camera pitch tilt in DEGREES
DISPLAY_SCALE = 0.65             # Preview window scale for 720x1280 HD side-by-side layout

# ==========================================
# 2. ZMQ COMMUNICATION SETUP
# ==========================================
ZMQ_SUB_PORT = 5555  # Receives commands from master controller (SET, SHOOT)
ZMQ_PUB_PORT = 5556  # Publishes launch vector & landing telemetry to master controller

context = zmq.Context()
sub_socket = context.socket(zmq.SUB)
sub_socket.connect(f"tcp://localhost:{ZMQ_SUB_PORT}")
sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")

pub_socket = context.socket(zmq.PUB)
pub_socket.bind(f"tcp://*:{ZMQ_PUB_PORT}")

# ==========================================
# 3. 3D GRAVITY-ALIGNED DEPROJECTION
# ==========================================
def get_3d_point(intrinsics, u_rot, v_rot, depth_val, rotated_cw=True, tilt_deg=0.0):
    """
    Converts a rotated pixel coordinate back to gravity-aligned 3D world space (meters):
      world_x: lateral left/right wrt camera
      world_y: true vertical up/down wrt floor/gravity (+ UP above lens, - DOWN below lens)
      world_z: true horizontal forward distance along the floor (+ forward)
    """
    if rotated_cw:
        u_orig = v_rot
        v_orig = (intrinsics.height - 1) - u_rot
    else:
        u_orig = u_rot
        v_orig = v_rot

    point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [u_orig, v_orig], depth_val)
    cam_x = point_3d[1]
    cam_y = -point_3d[0]
    cam_z = point_3d[2]

    tilt_rad = math.radians(tilt_deg)
    cos_t = math.cos(tilt_rad)
    sin_t = math.sin(tilt_rad)
    world_x = cam_x
    world_y = cam_y * cos_t + cam_z * sin_t
    world_z = -cam_y * sin_t + cam_z * cos_t
    return (world_x, world_y, world_z)


def main():
    # ==========================================
    # 4. DATASET CSV LOGGING SETUP
    # ==========================================
    csv_file = "shuttle_flight_data.csv"
    file_exists = os.path.isfile(csv_file)
    f_csv = open(csv_file, mode="a", newline="")
    csv_writer = csv.writer(f_csv)
    if not file_exists:
        csv_writer.writerow([
            "timestamp", "pitch_cmd", "yaw_cmd", "speed_cmd",
            "v0_mps", "launch_angle_deg", "start_z_cam_m", "start_y_cam_m",
            "camera_height_m", "landing_dist_cam_m"
        ])
        f_csv.flush()

    # ==========================================
    # 5. REALSENSE HARDWARE INITIALIZATION
    # ==========================================
    ctx = rs.context()
    if len(ctx.query_devices()) == 0:
        print("[ERROR] No Intel RealSense device detected on USB!")
        f_csv.close()
        return

    pipeline = rs.pipeline()
    # Prioritize Full HD 1280x720 RGB first for maximum sharpness & shuttle detail
    stream_candidates = [
        (COLOR_WIDTH, COLOR_HEIGHT, COLOR_FPS, DEPTH_WIDTH, DEPTH_HEIGHT, DEPTH_FPS),
        (960, 540, 60, 848, 480, 60),
        (848, 480, 60, 848, 480, 60),
    ]

    profile = None
    active_stream_cfg = None
    for idx, (cw, ch, cfps, dw, dh, dfps) in enumerate(stream_candidates):
        try:
            config = rs.config()
            config.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, dfps)
            config.enable_stream(rs.stream.color, cw, ch, rs.format.bgr8, cfps)
            profile = pipeline.start(config)
            active_stream_cfg = (cw, ch, cfps, dw, dh, dfps)
            print(f"[INFO] RealSense stream started: RGB {cw}x{ch}@{cfps}FPS | Depth {dw}x{dh}@{dfps}FPS")
            break
        except RuntimeError as e:
            print(f"[WARNING] Stream profile RGB {cw}x{ch}@{cfps}FPS rejected ({e}).")
            if idx == 0:
                # Reset hardware once only if the camera was stuck in a zombie state
                devices = ctx.query_devices()
                if len(devices) > 0:
                    print("[INFO] Resetting RealSense hardware once to clear any zombie state...")
                    devices[0].hardware_reset()
                    time.sleep(3.5)

    if profile is None:
        print("[ERROR] Could not start Intel RealSense pipeline on any supported profile.")
        f_csv.close()
        return

    # Configure Depth Sensor for High Accuracy & Max Laser Power
    device = profile.get_device()
    depth_sensor = device.first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    try:
        if depth_sensor.supports(rs.option.visual_preset):
            depth_sensor.set_option(rs.option.visual_preset, 3)  # High Accuracy
        if depth_sensor.supports(rs.option.laser_power):
            max_laser = depth_sensor.get_option_range(rs.option.laser_power).max
            depth_sensor.set_option(rs.option.laser_power, max_laser)
    except Exception as e:
        print(f"[WARNING] Could not apply depth sensor options: {e}")

    # Configure RGB Color Sensor for Fast Manual Shutter (Zero Motion Blur + Constant Brightness)
    try:
        color_sensor = device.first_color_sensor()
        if color_sensor.supports(rs.option.enable_auto_exposure):
            color_sensor.set_option(rs.option.enable_auto_exposure, 0)
        if color_sensor.supports(rs.option.auto_exposure_priority):
            color_sensor.set_option(rs.option.auto_exposure_priority, 0)
        if color_sensor.supports(rs.option.exposure):
            exp_range = color_sensor.get_option_range(rs.option.exposure)
            target_exp = float(np.clip(RGB_MANUAL_EXPOSURE_US / 100.0, exp_range.min, exp_range.max))
            color_sensor.set_option(rs.option.exposure, target_exp)
        if color_sensor.supports(rs.option.gain):
            gain_range = color_sensor.get_option_range(rs.option.gain)
            target_gain = float(np.clip(RGB_MANUAL_GAIN, gain_range.min, gain_range.max))
            color_sensor.set_option(rs.option.gain, target_gain)
        if color_sensor.supports(rs.option.sharpness):
            sharp_range = color_sensor.get_option_range(rs.option.sharpness)
            color_sensor.set_option(rs.option.sharpness, min(sharp_range.max, 75.0))
        print(
            f"[INFO] RGB Manual Shutter Locked -> {active_stream_cfg[0]}x{active_stream_cfg[1]} @ {active_stream_cfg[2]}FPS | "
            f"Exposure: {RGB_MANUAL_EXPOSURE_US / 1000.0:.1f}ms | Gain: {RGB_MANUAL_GAIN}"
        )
    except Exception as e:
        print(f"[WARNING] Could not configure manual RGB shutter options: {e}")

    align = rs.align(rs.stream.color)
    color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intrinsics = color_stream.get_intrinsics()

    # Precompute tilted pinhole geometry constants (works dynamically for both 480x848 and HD 720x1280)
    rot_w = intrinsics.height if ROTATION_CW else intrinsics.width
    rot_h = intrinsics.width if ROTATION_CW else intrinsics.height
    focal_length = intrinsics.fx
    center_x = ((intrinsics.height - 1.0) - intrinsics.ppy) if ROTATION_CW else intrinsics.ppx
    center_y = intrinsics.ppx if ROTATION_CW else intrinsics.ppy
    tilt_rad = math.radians(CAMERA_TILT_DEG)
    true_horizon_y = center_y + focal_length * math.tan(tilt_rad)
    floor_min_y = true_horizon_y + (22.0 if rot_h > 1000 else 15.0)

    def project_floor_z_to_y(z_meters):
        """Projects a floor point at forward distance z_meters (m) to rotated image row y (px)."""
        z_safe = max(0.4, float(z_meters))
        delta_rad = math.atan2(CAMERA_HEIGHT, z_safe)
        ray_tan = math.tan(delta_rad + tilt_rad)
        return center_y + focal_length * ray_tan

    print(f"[INFO] RealSense Initialized (Rot Frame: {rot_w}x{rot_h} | Scale: {depth_scale:.8f} | Horizon Y: {true_horizon_y:.1f}px)")
    print(f"[INFO] Fixed {FIXED_LANDING_TIMER_SEC:.1f}s Timer + HD Lower-Feed Diff Tracker Active (Press 'd' for Depth, 'q' to quit).")

    window_name = "Turret Depth Tracker"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    # ==========================================
    # 6. TRACKING STATE MACHINE VARIABLES
    # ==========================================
    # States: IDLE -> ACQUIRE_LAUNCH -> TRACK_LAUNCH_VEC -> WAIT_2S_AND_DIFF -> DISPLAY_RESULT
    state = "IDLE"
    start_time = 0.0
    display_start_time = 0.0
    last_result_frame = None
    current_cmd = {"p": 100, "y": 90, "s": 0}

    # Initial 3D Launch Vector State
    trajectory_data = []          # [(t, world_x, world_y, world_z), ...]
    launch_trail_2d = []          # [(cx, cy), ...] during initial launch capture
    launch_published = False
    v0 = 0.0
    launch_angle_deg = 0.0
    start_z_cam = 0.0
    start_y_cam = 0.0

    pending_acq = None
    tracking_center = (0, 0)
    tracking_bbox = (40, 40)
    last_known_z = 1.0
    launch_lost_frames = 0

    # Lower-Feed Reference (captured mid-flight ~0.5s) vs. Post-Landing (captured at 2.0s)
    lower_base_top = max(0, int(round(floor_min_y - 10.0)))
    idle_lower_rgb = None
    idle_lower_depth = None

    ref_rgb_accum = None
    ref_depth_accum = None
    ref_depth_count = None
    ref_frames_captured = 0
    ref_lower_rgb = None
    ref_lower_depth = None
    ref_stab_win = None

    post_rgb_accum = None
    post_depth_accum = None
    post_depth_count = None
    post_frames_captured = 0
    last_vib_shift = (0.0, 0.0)
    hanning_256 = cv2.createHanningWindow((256, 256), cv2.CV_32F)

    # Floor Watch Corridor (Bottom 35% of HD frame, clipped 30% on left & right)
    floor_y_top = max(lower_base_top, int(round(floor_min_y)), int(rot_h * (1.0 - ROI_BOTTOM_FRAC)))
    floor_y_bot = rot_h - 50
    floor_x_left = int(rot_w * ROI_CLIP_LR)
    floor_x_right = int(rot_w * (1.0 - ROI_CLIP_LR))
    eg_corridor_box = (floor_x_left, floor_y_top, floor_x_right, floor_y_bot)

    landing_pixel = (0, 0)
    landing_dist_cam = 0.0

    # Background & Ceiling-Light Blackout Masks
    idle_bg_depth = None
    shoot_bg_depth = None
    light_core_accum = None
    light_blackout_mask = None
    light_guard_mask = None
    light_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (45, 45))
    halo_seed_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (35, 35))
    guard_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (19, 19))
    morph_open_kernel = np.ones((3, 3), np.uint8)
    rgb_speckle_kernel = np.ones((2, 2), np.uint8)
    morph_close_kernel = np.ones((3, 3), np.uint8)
    idle_frame_counter = 0

    show_depth_panel = True
    prev_color_frame_num = None
    total_hw_drops = 0
    shot_proc_times = []
    shot_hw_drops = 0
    rot_flag = cv2.ROTATE_90_CLOCKWISE if ROTATION_CW else cv2.ROTATE_90_COUNTERCLOCKWISE

    def build_stab_window(gray_lower):
        """Builds a 256x256 Hanning-windowed patch from left/right static edges for phaseCorrelate."""
        gh, gw = gray_lower.shape
        if gh < 256 or gw < 256:
            return None
        left_strip = gray_lower[:256, :128]
        right_strip = gray_lower[:256, gw - 128:gw]
        combined = np.hstack((left_strip, right_strip)).astype(np.float32)
        return combined * hanning_256

    def stabilize_to_ref(lower_rgb_clean, lower_d):
        """Aligns live lower-feed frame to ref_stab_win to remove sub-pixel wheel shake."""
        nonlocal last_vib_shift
        if ref_stab_win is None:
            last_vib_shift = (0.0, 0.0)
            return lower_rgb_clean, lower_d
        gray_live = cv2.cvtColor(lower_rgb_clean, cv2.COLOR_BGR2GRAY)
        curr_win = build_stab_window(gray_live)
        shift_x, shift_y = 0.0, 0.0
        if curr_win is not None:
            (dx, dy), resp = cv2.phaseCorrelate(ref_stab_win, curr_win)
            if resp > 0.15 and abs(dx) <= 18.0 and abs(dy) <= 18.0:
                shift_x, shift_y = float(dx), float(dy)
        last_vib_shift = (shift_x, shift_y)
        if abs(shift_x) > 0.05 or abs(shift_y) > 0.05:
            lh, lw = lower_d.shape
            warp_m = np.float32([[1.0, 0.0, -shift_x], [0.0, 1.0, -shift_y]])
            stab_rgb = cv2.warpAffine(
                lower_rgb_clean, warp_m, (lw, lh),
                flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE
            )
            stab_d = cv2.warpAffine(
                lower_d, warp_m, (lw, lh),
                flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_REPLICATE
            )
            return stab_rgb, stab_d
        return lower_rgb_clean, lower_d

    def finalize_launch_vector(frame_w, frame_h):
        """Computes initial (v0, angle) from 3D depth points and publishes launch telemetry."""
        nonlocal launch_published, v0, launch_angle_deg, start_z_cam, start_y_cam, eg_corridor_box

        if len(trajectory_data) >= 2:
            p0 = trajectory_data[0]
            p1 = trajectory_data[-1]
            dt = max(1e-4, p1[0] - p0[0])

            vx = (p1[1] - p0[1]) / dt
            vy = (p1[2] - p0[2]) / dt
            vz = (p1[3] - p0[3]) / dt

            cand_v0 = math.sqrt(vx**2 + vy**2 + vz**2)
            horiz_speed = math.sqrt(vx**2 + vz**2)
            cand_angle = math.degrees(math.atan2(vy, max(1e-4, horiz_speed)))

            v0 = float(np.clip(cand_v0, 3.5, 32.0))
            launch_angle_deg = float(np.clip(cand_angle, -10.0, 70.0))
            start_z_cam = p0[3]
            start_y_cam = p0[2]

            if not launch_published:
                launch_published = True
                print(
                    f"[DATA] Initial Vector ({len(trajectory_data)} 3D pts) -> V0: {v0:.2f} m/s | "
                    f"Angle: {launch_angle_deg:.1f}° | Start wrt Cam: (z={start_z_cam:.2f}m, y={start_y_cam:.2f}m)"
                )
                pub_socket.send_json({
                    "status": "launch_tracked",
                    "v0": round(v0, 2),
                    "angle": round(launch_angle_deg, 2),
                    "start_z_cam": round(start_z_cam, 3),
                    "start_y_cam": round(start_y_cam, 3)
                })

        y_top = max(lower_base_top, int(round(floor_min_y)), int(frame_h * (1.0 - ROI_BOTTOM_FRAC)))
        y_bot = frame_h - 10
        x_left = int(frame_w * ROI_CLIP_LR)
        x_right = int(frame_w * (1.0 - ROI_CLIP_LR))
        eg_corridor_box = (x_left, y_top, x_right, y_bot)

    # ==========================================
    # 7. MAIN 60 FPS TRACKING LOOP
    # ==========================================
    try:
        while True:
            # A. Poll ZMQ for Master Commands (Non-blocking)
            while True:
                try:
                    msg = sub_socket.recv_json(flags=zmq.NOBLOCK)
                    if "p" in msg or "pitch" in msg:
                        current_cmd["p"] = msg.get("p", msg.get("pitch", current_cmd["p"]))
                    if "y" in msg or "yaw" in msg:
                        current_cmd["y"] = msg.get("y", msg.get("yaw", current_cmd["y"]))
                    if "s" in msg or "speed" in msg:
                        current_cmd["s"] = msg.get("s", msg.get("speed", current_cmd["s"]))

                    if msg.get("command") == "SHOOT":
                        print(f"\n[EVENT] SHOOT command received: P:{current_cmd['p']} Y:{current_cmd['y']} S:{current_cmd['s']}")
                        state = "ACQUIRE_LAUNCH"
                        start_time = time.time()
                        trajectory_data = []
                        launch_trail_2d = []
                        launch_published = False
                        v0 = 0.0
                        launch_angle_deg = 0.0
                        start_z_cam = 0.0
                        start_y_cam = 0.0

                        pending_acq = None
                        launch_lost_frames = 0

                        ref_rgb_accum = None
                        ref_depth_accum = None
                        ref_depth_count = None
                        ref_frames_captured = 0
                        ref_lower_rgb = None
                        ref_lower_depth = None
                        ref_stab_win = None

                        post_rgb_accum = None
                        post_depth_accum = None
                        post_depth_count = None
                        post_frames_captured = 0
                        last_vib_shift = (0.0, 0.0)

                        landing_pixel = (0, 0)
                        landing_dist_cam = 0.0
                        shot_proc_times = []
                        shot_hw_drops = 0

                        if idle_bg_depth is not None:
                            valid_bg = np.where(idle_bg_depth > 0.1, idle_bg_depth, 99.0).astype(np.float32)
                            shoot_bg_depth = cv2.erode(valid_bg, np.ones((7, 7), np.uint8))
                        else:
                            shoot_bg_depth = None
                except zmq.Again:
                    break

            # B. Acquire Frames from RealSense (Skip heavy align.process only in IDLE)
            frames = pipeline.wait_for_frames(5000)
            proc_start = time.perf_counter()

            if state == "IDLE":
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                raw_color = np.asanyarray(color_frame.get_data())
                raw_color = cv2.rotate(raw_color, rot_flag)
                h, w = raw_color.shape[:2]

                raw_depth = np.asanyarray(depth_frame.get_data())
                raw_depth = cv2.rotate(raw_depth, rot_flag)
                if raw_depth.shape[:2] != (h, w):
                    raw_depth = cv2.resize(raw_depth, (w, h), interpolation=cv2.INTER_NEAREST)
                depth_meters = raw_depth.astype(np.float32) * depth_scale
            else:
                aligned_frames = align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                if not color_frame or not depth_frame:
                    continue
                raw_color = np.asanyarray(color_frame.get_data())
                raw_depth = np.asanyarray(depth_frame.get_data())
                raw_color = cv2.rotate(raw_color, rot_flag)
                raw_depth = cv2.rotate(raw_depth, rot_flag)
                depth_meters = raw_depth.astype(np.float32) * depth_scale
                h, w = depth_meters.shape

            # Track hardware USB frame drops
            curr_frame_num = color_frame.get_frame_number()
            if prev_color_frame_num is not None:
                gap = int(curr_frame_num - prev_color_frame_num)
                if gap > 1:
                    dropped = gap - 1
                    total_hw_drops += dropped
                    if state in ("ACQUIRE_LAUNCH", "TRACK_LAUNCH_VEC", "WAIT_2S_AND_DIFF"):
                        shot_hw_drops += dropped
                        print(f"[DEBUGGER WARNING] Dropped {dropped} frame(s)! (Frame #{curr_frame_num})")
            prev_color_frame_num = curr_frame_num

            # C. Detect & Black Out Ceiling Lights (Upper Ceiling Zone Only, Throttled in IDLE)
            if state == "IDLE":
                idle_frame_counter += 1
                if light_blackout_mask is None or (idle_frame_counter % 10 == 0):
                    curr_gray = cv2.cvtColor(raw_color, cv2.COLOR_BGR2GRAY)
                    if light_core_accum is None or light_core_accum.shape != curr_gray.shape:
                        light_core_accum = np.zeros_like(curr_gray, dtype=np.uint8)
                    ceiling_limit_y = int(h * 0.38)
                    ceiling_band = np.zeros_like(curr_gray, dtype=np.uint8)
                    ceiling_band[:ceiling_limit_y, :] = curr_gray[:ceiling_limit_y, :]

                    light_core_accum[ceiling_band >= LIGHT_SAT_THRESH] = 255
                    core_dilated = cv2.dilate(light_core_accum, halo_seed_kernel)
                    halo_mask = np.where((core_dilated > 0) & (ceiling_band >= 195), 255, 0).astype(np.uint8)
                    combined_lights = cv2.bitwise_or(light_core_accum, halo_mask)
                    light_blackout_mask = cv2.dilate(combined_lights, light_kernel)
                    light_guard_mask = cv2.dilate(light_blackout_mask, guard_kernel)

            if light_blackout_mask is not None:
                raw_color[light_blackout_mask > 0] = 0
                depth_meters[light_blackout_mask > 0] = 0.0

            # Build depth colormap visualization
            depth_viz = None
            if show_depth_panel:
                depth_viz = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_meters, alpha=255.0 / DEPTH_CAP_FLIGHT),
                    cv2.COLORMAP_JET
                )
                depth_viz[depth_meters <= 0.0] = [0, 0, 0]

            viz_frame = raw_color

            # Upper launch corridor bounds
            crop_top = 0
            crop_bottom = int(h * 0.52)
            crop_left = int(w * 0.20)
            crop_right = int(w * 0.76)

            # ==========================================
            # STATE 1: IDLE (Learn Static Room Depth, Fallback Lower Feed & Light Mask)
            # ==========================================
            if state == "IDLE":
                if idle_bg_depth is None or idle_bg_depth.shape != depth_meters.shape:
                    idle_bg_depth = depth_meters.copy()
                else:
                    valid_now = depth_meters > 0.1
                    empty_bg = idle_bg_depth <= 0.1
                    idle_bg_depth[valid_now & empty_bg] = depth_meters[valid_now & empty_bg]
                    both_valid = valid_now & (~empty_bg)
                    idle_bg_depth[both_valid] = np.minimum(
                        idle_bg_depth[both_valid] * 0.995 + depth_meters[both_valid] * 0.005,
                        depth_meters[both_valid] + 0.05
                    )

                idle_lower_rgb = raw_color[lower_base_top:h, :].copy()
                idle_lower_depth = depth_meters[lower_base_top:h, :].copy()

                rx1 = int(w * ROI_CLIP_LR)
                rx2 = int(w * (1.0 - ROI_CLIP_LR))
                ry1 = max(lower_base_top, int(round(floor_min_y)), int(h * (1.0 - ROI_BOTTOM_FRAC)))
                ry2 = h - 10
                eg_corridor_box = (rx1, ry1, rx2, ry2)

                cv2.rectangle(raw_color, (crop_left, crop_top), (crop_right, crop_bottom), (0, 255, 0), 1)
                cv2.line(raw_color, (0, int(round(floor_min_y))), (w, int(round(floor_min_y))), (100, 100, 100), 1)
                cv2.rectangle(raw_color, (rx1, ry1), (rx2, ry2), (0, 215, 255), 1)
                if depth_viz is not None:
                    cv2.rectangle(depth_viz, (crop_left, crop_top), (crop_right, crop_bottom), (0, 255, 0), 1)
                    cv2.rectangle(depth_viz, (rx1, ry1), (rx2, ry2), (0, 215, 255), 1)
                viz_frame = raw_color

            # ==========================================
            # STATE 2: ACQUIRE_LAUNCH (2-Frame Depth Lock in Launch Corridor)
            # ==========================================
            elif state == "ACQUIRE_LAUNCH":
                cv2.rectangle(raw_color, (crop_left, crop_top), (crop_right, crop_bottom), (0, 255, 0), 1)
                if depth_viz is not None:
                    cv2.rectangle(depth_viz, (crop_left, crop_top), (crop_right, crop_bottom), (0, 255, 0), 1)

                box_d = depth_meters[crop_top:crop_bottom, crop_left:crop_right]
                valid_range = (box_d >= MIN_DEPTH_METERS) & (box_d <= DEPTH_CAP_ACQ)

                if shoot_bg_depth is not None:
                    box_bg = shoot_bg_depth[crop_top:crop_bottom, crop_left:crop_right]
                    fg_depth_cond = valid_range & (box_d < (box_bg - 0.25))
                else:
                    fg_depth_cond = valid_range

                acq_mask = np.where(fg_depth_cond, 255, 0).astype(np.uint8)
                acq_mask = cv2.morphologyEx(acq_mask, cv2.MORPH_OPEN, morph_open_kernel)

                contours, _ = cv2.findContours(acq_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                candidates = []
                for c in contours:
                    area = cv2.contourArea(c)
                    if MIN_CONTOUR_AREA_ACQ <= area <= MAX_CONTOUR_AREA_ACQ:
                        rbx, rby, rbw, rbh = cv2.boundingRect(c)
                        gx = crop_left + rbx
                        gy = crop_top + rby
                        cx = gx + rbw // 2
                        cy = gy + rbh // 2
                        if gy < 18 or cy < 22:
                            continue
                        if light_guard_mask is not None and light_guard_mask[min(h - 1, cy), min(w - 1, cx)] > 0:
                            continue
                        roi_patch = box_d[rby:rby + rbh, rbx:rbx + rbw]
                        valid_z = roi_patch[(roi_patch >= MIN_DEPTH_METERS) & (roi_patch <= DEPTH_CAP_ACQ)]
                        if valid_z.size > 0:
                            z_med = float(np.percentile(valid_z, 30))
                            candidates.append((gx, gy, rbw, rbh, cx, cy, z_med))

                matched_now = None
                if candidates and pending_acq is not None:
                    best_dist = 9999.0
                    for cand in candidates:
                        gx, gy, rbw, rbh, cx, cy, z_med = cand
                        move_px = math.hypot(cx - pending_acq["cx"], cy - pending_acq["cy"])
                        dz = z_med - pending_acq["z"]
                        if 4.0 <= move_px <= 150.0 and -0.15 <= dz <= 0.65:
                            if move_px < best_dist:
                                best_dist = move_px
                                matched_now = cand

                if matched_now is not None:
                    gx, gy, rbw, rbh, cx, cy, z_med = matched_now
                    now_t = time.time() - start_time

                    pt0 = get_3d_point(
                        intrinsics, pending_acq["cx"], pending_acq["cy"], pending_acq["z"], ROTATION_CW, CAMERA_TILT_DEG
                    )
                    pt1 = get_3d_point(
                        intrinsics, cx, cy, z_med, ROTATION_CW, CAMERA_TILT_DEG
                    )
                    trajectory_data.append((pending_acq["t"], pt0[0], pt0[1], pt0[2]))
                    trajectory_data.append((now_t, pt1[0], pt1[1], pt1[2]))
                    launch_trail_2d.append((pending_acq["cx"], pending_acq["cy"]))
                    launch_trail_2d.append((cx, cy))

                    tracking_center = (cx, cy)
                    tracking_bbox = (max(36, rbw), max(36, rbh))
                    last_known_z = z_med
                    launch_lost_frames = 0

                    cv2.rectangle(raw_color, (gx, gy), (gx + rbw, gy + rbh), (0, 255, 0), 2)
                    state = "TRACK_LAUNCH_VEC"
                elif candidates:
                    candidates.sort(key=lambda item: item[6])
                    gx, gy, rbw, rbh, cx, cy, z_med = candidates[0]
                    pending_acq = {"cx": cx, "cy": cy, "z": z_med, "t": time.time() - start_time}
                    cv2.rectangle(raw_color, (gx, gy), (gx + rbw, gy + rbh), (255, 255, 0), 1)
                else:
                    pending_acq = None

                # Move into 2.0s timer wait if launch acquisition takes longer than ACQ_MAX_WINDOW_SEC
                if state == "ACQUIRE_LAUNCH" and (time.time() - start_time >= ACQ_MAX_WINDOW_SEC):
                    print("[WARNING] Launch depth lock missed -> Proceeding to 2.0s lower-feed timer.")
                    finalize_launch_vector(w, h)
                    state = "WAIT_2S_AND_DIFF"

                viz_frame = raw_color

            # ==========================================
            # STATE 3: TRACK_LAUNCH_VEC (Collect 3D Launch Points -> Enter 2.0s Timer)
            # ==========================================
            elif state == "TRACK_LAUNCH_VEC":
                z_cap_min = max(MIN_DEPTH_METERS, last_known_z - 0.35)
                z_cap_max = min(4.2, last_known_z + 0.75)

                box_w = max(92, int(tracking_bbox[0] * 2.2))
                box_h = max(92, int(tracking_bbox[1] * 2.2))
                x1 = max(0, tracking_center[0] - box_w // 2)
                y1 = max(0, tracking_center[1] - box_h // 2)
                x2 = min(w, tracking_center[0] + box_w // 2)
                y2 = min(h, tracking_center[1] + box_h // 2)

                detected = False
                if x2 > x1 and y2 > y1:
                    roi_d = depth_meters[y1:y2, x1:x2]
                    in_band = (roi_d >= z_cap_min) & (roi_d <= z_cap_max)
                    if shoot_bg_depth is not None:
                        roi_bg = shoot_bg_depth[y1:y2, x1:x2]
                        in_band = in_band & (roi_d < (roi_bg - 0.18))

                    d_mask = cv2.morphologyEx(np.where(in_band, 255, 0).astype(np.uint8), cv2.MORPH_OPEN, morph_open_kernel)
                    contours, _ = cv2.findContours(d_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    best_blob = None
                    best_score = 9999.0
                    for c in contours:
                        area = cv2.contourArea(c)
                        if 10 <= area <= MAX_CONTOUR_AREA_ACQ:
                            bx, by, bw, bh = cv2.boundingRect(c)
                            patch_d = roi_d[by:by + bh, bx:bx + bw]
                            valid_p = patch_d[(patch_d >= z_cap_min) & (patch_d <= z_cap_max)]
                            if valid_p.size == 0:
                                continue
                            cand_z = float(np.percentile(valid_p, 25))
                            cand_cx = x1 + bx + bw // 2
                            cand_cy = y1 + by + bh // 2
                            score = math.hypot(cand_cx - tracking_center[0], cand_cy - tracking_center[1]) + 45.0 * abs(cand_z - last_known_z)
                            if score < best_score:
                                best_score = score
                                best_blob = (cand_cx, cand_cy, bw, bh, cand_z)

                    if best_blob is not None:
                        cx, cy, bw, bh, z_val = best_blob
                        tracking_center = (cx, cy)
                        tracking_bbox = (max(30, bw), max(30, bh))
                        last_known_z = z_val
                        launch_trail_2d.append((cx, cy))
                        pt_3d = get_3d_point(intrinsics, cx, cy, z_val, ROTATION_CW, CAMERA_TILT_DEG)
                        trajectory_data.append((time.time() - start_time, pt_3d[0], pt_3d[1], pt_3d[2]))
                        detected = True
                        launch_lost_frames = 0

                if not detected:
                    launch_lost_frames += 1

                if len(launch_trail_2d) > 1:
                    cv2.polylines(raw_color, [np.array(launch_trail_2d, np.int32).reshape((-1, 1, 2))], False, (0, 255, 0), 2)

                if (
                    len(trajectory_data) >= 5
                    or (len(trajectory_data) >= 2 and launch_lost_frames >= 2)
                    or (time.time() - start_time >= ACQ_MAX_WINDOW_SEC)
                ):
                    finalize_launch_vector(w, h)
                    print(f"[INFO] Capturing clean mid-flight lower-feed reference -> Waiting for {FIXED_LANDING_TIMER_SEC:.1f}s timer...")
                    state = "WAIT_2S_AND_DIFF"

                viz_frame = raw_color

            # ==========================================
            # STATE 4: WAIT_2S_AND_DIFF (Mid-Flight Ref -> 2.0s Fixed Timer -> Lower-Feed Diff)
            # ==========================================
            elif state == "WAIT_2S_AND_DIFF":
                elapsed = time.time() - start_time
                if len(launch_trail_2d) > 1:
                    cv2.polylines(raw_color, [np.array(launch_trail_2d, np.int32).reshape((-1, 1, 2))], False, (0, 255, 0), 2)

                cx1, cy1, cx2, cy2 = eg_corridor_box
                cv2.line(raw_color, (0, int(round(true_horizon_y))), (w, int(round(true_horizon_y))), (100, 100, 100), 1)
                cv2.rectangle(raw_color, (cx1, cy1), (cx2, cy2), (0, 215, 255), 1)
                if depth_viz is not None:
                    cv2.rectangle(depth_viz, (cx1, cy1), (cx2, cy2), (0, 215, 255), 1)

                lower_live_sharp = raw_color[lower_base_top:h, :]
                lower_live_d = depth_meters[lower_base_top:h, :]

                # Step A: Capture 4-frame clean empty-floor reference right after launch
                # while the shuttle is high in the air above floor_min_y and feeder recoil has settled.
                if ref_frames_captured < 4 and elapsed < (FIXED_LANDING_TIMER_SEC - 0.35):
                    if ref_frames_captured == 0:
                        ref_rgb_accum = lower_live_sharp.astype(np.float32)
                        valid_d = lower_live_d > 0.1
                        ref_depth_accum = np.where(valid_d, lower_live_d, 0.0).astype(np.float32)
                        ref_depth_count = valid_d.astype(np.float32)
                        ref_stab_win = build_stab_window(cv2.cvtColor(lower_live_sharp, cv2.COLOR_BGR2GRAY))
                    else:
                        stab_rgb, stab_d = stabilize_to_ref(lower_live_sharp, lower_live_d)
                        ref_rgb_accum += stab_rgb.astype(np.float32)
                        valid_d = stab_d > 0.1
                        ref_depth_accum += np.where(valid_d, stab_d, 0.0).astype(np.float32)
                        ref_depth_count += valid_d.astype(np.float32)

                    ref_frames_captured += 1
                    if ref_frames_captured == 4:
                        ref_lower_rgb = np.clip(ref_rgb_accum / 4.0, 0, 255).astype(np.uint8)
                        ref_lower_depth = np.where(
                            ref_depth_count > 0,
                            ref_depth_accum / np.maximum(1.0, ref_depth_count),
                            99.0
                        ).astype(np.float32)
                        ref_stab_win = build_stab_window(cv2.cvtColor(ref_lower_rgb, cv2.COLOR_BGR2GRAY))

                # Step B: At t >= FIXED_LANDING_TIMER_SEC, capture 3 vibration-aligned frames and diff lower feed
                if elapsed >= FIXED_LANDING_TIMER_SEC:
                    if ref_lower_rgb is None:
                        if ref_frames_captured > 0:
                            ref_lower_rgb = np.clip(ref_rgb_accum / float(ref_frames_captured), 0, 255).astype(np.uint8)
                            ref_lower_depth = np.where(
                                ref_depth_count > 0,
                                ref_depth_accum / np.maximum(1.0, ref_depth_count),
                                99.0
                            ).astype(np.float32)
                        elif idle_lower_rgb is not None:
                            ref_lower_rgb = idle_lower_rgb.copy()
                            ref_lower_depth = np.where(idle_lower_depth > 0.1, idle_lower_depth, 99.0).astype(np.float32)

                    if ref_lower_rgb is not None:
                        stab_rgb, stab_d = stabilize_to_ref(lower_live_sharp, lower_live_d)
                        if post_frames_captured == 0:
                            post_rgb_accum = stab_rgb.astype(np.float32)
                            valid_d = stab_d > 0.1
                            post_depth_accum = np.where(valid_d, stab_d, 0.0).astype(np.float32)
                            post_depth_count = valid_d.astype(np.float32)
                        else:
                            post_rgb_accum += stab_rgb.astype(np.float32)
                            valid_d = stab_d > 0.1
                            post_depth_accum += np.where(valid_d, stab_d, 0.0).astype(np.float32)
                            post_depth_count += valid_d.astype(np.float32)
                        post_frames_captured += 1

                    # Once 3 frames at t >= 2.0s are averaged, compare against the t~0.5s reference
                    if post_frames_captured >= 3 or ref_lower_rgb is None:
                        if ref_lower_rgb is not None and post_rgb_accum is not None:
                            post_lower_rgb = np.clip(post_rgb_accum / float(post_frames_captured), 0, 255).astype(np.uint8)
                            post_lower_depth = np.where(
                                post_depth_count > 0,
                                post_depth_accum / np.maximum(1.0, post_depth_count),
                                99.0
                            ).astype(np.float32)

                            ry1 = max(0, cy1 - lower_base_top)
                            ry2 = min(ref_lower_rgb.shape[0], cy2 - lower_base_top)

                            if (ry2 - ry1) > 10 and cx2 > cx1:
                                ref_patch = ref_lower_rgb[ry1:ry2, cx1:cx2]
                                post_patch = post_lower_rgb[ry1:ry2, cx1:cx2]
                                abs_diff = cv2.absdiff(post_patch, ref_patch)
                                diff_max_chan = np.max(abs_diff, axis=2)

                                rgb_fg_raw = np.where(diff_max_chan >= RGB_DIFF_THRESH, 255, 0).astype(np.uint8)
                                rgb_fg = cv2.morphologyEx(rgb_fg_raw, cv2.MORPH_OPEN, rgb_speckle_kernel) > 0

                                ref_d_patch = ref_lower_depth[ry1:ry2, cx1:cx2]
                                post_d_patch = post_lower_depth[ry1:ry2, cx1:cx2]
                                depth_fg = (
                                    (post_d_patch >= 0.80)
                                    & (post_d_patch <= 9.50)
                                    & (post_d_patch < (ref_d_patch - DEPTH_DIFF_THRESH_M))
                                )

                                diff_mask = np.where(rgb_fg | depth_fg, 255, 0).astype(np.uint8)
                                diff_mask = cv2.morphologyEx(diff_mask, cv2.MORPH_CLOSE, morph_close_kernel)

                                contours, _ = cv2.findContours(diff_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                                candidates = []
                                max_area_hd = int(MAX_LANDED_AREA_PX * ((w / 480.0) ** 2))
                                for c in contours:
                                    area = cv2.contourArea(c)
                                    if area > max_area_hd:
                                        continue
                                    bx, by, bw, bh = cv2.boundingRect(c)
                                    has_depth = bool(np.any(depth_fg[by:by + bh, bx:bx + bw]))
                                    if has_depth and area < MIN_LANDED_AREA_DEPTH:
                                        continue
                                    if not has_depth and area < MIN_LANDED_AREA_RGB:
                                        continue

                                    peak_diff = float(np.max(diff_max_chan[by:by + bh, bx:bx + bw]))
                                    gx = cx1 + bx + bw // 2
                                    gy = cy1 + by + bh // 2

                                    # Score favors depth-confirmed blobs and strong contrast near center lane
                                    center_pen = 0.05 * abs(gx - (w // 2))
                                    area_pen = 0.04 * max(0.0, area - 240.0)
                                    score = (
                                        (35.0 if has_depth else 0.0)
                                        + 0.45 * peak_diff
                                        - center_pen
                                        - area_pen
                                    )
                                    candidates.append((score, gx, gy, bw, bh, area, has_depth))

                                if candidates:
                                    candidates.sort(key=lambda item: item[0], reverse=True)
                                    _, gx, gy, bw, bh, area, has_depth = candidates[0]
                                    landing_pixel = (gx, gy)

                                    # Project landing_pixel onto floor using tilted pinhole camera geometry
                                    ray_tan = (float(gy) - center_y) / focal_length
                                    total_down_angle = math.atan(ray_tan) - tilt_rad
                                    if total_down_angle > 0.02:
                                        landing_dist_cam = round(CAMERA_HEIGHT / math.tan(total_down_angle), 3)

                                    cv2.rectangle(
                                        raw_color,
                                        (gx - max(12, bw // 2), gy - max(12, bh // 2)),
                                        (gx + max(12, bw // 2), gy + max(12, bh // 2)),
                                        (0, 255, 255), 2
                                    )
                                    cv2.circle(raw_color, (gx, gy), 4, (0, 0, 255), -1)
                                    cv2.putText(
                                        raw_color,
                                        f"LANDED: {landing_dist_cam:.2f}m ({int(area)}px)",
                                        (max(10, gx - 75), max(int(floor_min_y) + 20, gy - 16)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2
                                    )

                        # Log shot row to CSV & publish completion telemetry to Master Controller
                        ts_str = time.strftime("%Y-%m-%d %H:%M:%S")
                        csv_writer.writerow([
                            ts_str, current_cmd["p"], current_cmd["y"], current_cmd["s"],
                            round(v0, 3), round(launch_angle_deg, 2),
                            round(start_z_cam, 3), round(start_y_cam, 3),
                            CAMERA_HEIGHT, landing_dist_cam
                        ])
                        f_csv.flush()

                        pub_socket.send_json({
                            "status": "shot_complete",
                            "v0": round(v0, 2),
                            "angle": round(launch_angle_deg, 2),
                            "start_z_cam": round(start_z_cam, 3),
                            "start_y_cam": round(start_y_cam, 3),
                            "landing_dist_cam": landing_dist_cam
                        })

                        print(
                            f"[DATA] Shot Complete @ {elapsed:.2f}s -> V0: {v0:.2f} m/s | "
                            f"Angle: {launch_angle_deg:.1f}° | Landing Range (wrt cam): {landing_dist_cam:.2f}m "
                            f"(px={landing_pixel})"
                        )
                        if shot_proc_times:
                            avg_ms = float(np.mean(shot_proc_times))
                            max_ms = float(np.max(shot_proc_times))
                            over_cnt = sum(1 for t_ms in shot_proc_times if t_ms > 16.67)
                            print(
                                f"[DEBUGGER] Shot Timing -> Avg: {avg_ms:.2f}ms | Max: {max_ms:.2f}ms "
                                f"(Budget: 16.67ms @ 60FPS) | Over-budget frames: {over_cnt}/{len(shot_proc_times)} | HW Drops: {shot_hw_drops}"
                            )

                        last_result_frame = raw_color.copy()
                        viz_frame = last_result_frame
                        display_start_time = time.time()
                        state = "DISPLAY_RESULT"
                else:
                    # Countdown banner while waiting for the 2.0s fixed timer
                    cv2.putText(
                        raw_color,
                        f"2.0s TIMER: {elapsed:.2f} / {FIXED_LANDING_TIMER_SEC:.2f}s (Ref: {ref_frames_captured}/4)",
                        (cx1 + 8, cy1 + 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 215, 255), 2
                    )
                    viz_frame = raw_color

            # ==========================================
            # STATE 5: DISPLAY_RESULT (1.2s Result Hold)
            # ==========================================
            elif state == "DISPLAY_RESULT":
                if time.time() - display_start_time > 1.2:
                    state = "IDLE"
                    viz_frame = raw_color
                else:
                    viz_frame = last_result_frame.copy() if last_result_frame is not None else raw_color

            # D. Render Side-by-Side RGB + Depth Preview & HUD
            proc_ms = (time.perf_counter() - proc_start) * 1000.0
            if state in ("ACQUIRE_LAUNCH", "TRACK_LAUNCH_VEC", "WAIT_2S_AND_DIFF"):
                shot_proc_times.append(proc_ms)

            inst_fps = min(60.0, 1000.0 / max(1.0, proc_ms))
            cv2.rectangle(viz_frame, (5, 5), (295, 82), (0, 0, 0), -1)
            fps_color = (0, 255, 0) if (proc_ms <= 16.67 and shot_hw_drops == 0) else (0, 165, 255)
            cv2.putText(viz_frame, f"FPS: {inst_fps:.1f} / 60 [{state}]", (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.48, fps_color, 1)
            cv2.putText(viz_frame, f"Proc: {proc_ms:.1f} / 16.7ms", (12, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.48, fps_color, 1)
            cv2.putText(viz_frame, f"Drops: {shot_hw_drops} (Total: {total_hw_drops})", (12, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 200, 255), 1)

            if show_depth_panel:
                if depth_viz is None:
                    depth_viz = np.zeros_like(viz_frame)
                hud_r = (
                    f"2.0S DIFF (VIB {last_vib_shift[0]:+.1f},{last_vib_shift[1]:+.1f}px)"
                    if state == "WAIT_2S_AND_DIFF"
                    else "DEPTH FEED (10m)"
                )
                cv2.putText(depth_viz, hud_r, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)
                display_img = np.hstack((viz_frame, depth_viz))
            else:
                display_img = viz_frame

            if DISPLAY_SCALE != 1.0:
                display_img = cv2.resize(
                    display_img, (0, 0), fx=DISPLAY_SCALE, fy=DISPLAY_SCALE, interpolation=cv2.INTER_AREA
                )

            cv2.imshow(window_name, display_img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("d"):
                show_depth_panel = not show_depth_panel

    except KeyboardInterrupt:
        print("\n[INFO] Shutting down Vision Pipeline...")
    finally:
        pipeline.stop()
        f_csv.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
