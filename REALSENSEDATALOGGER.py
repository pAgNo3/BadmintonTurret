import pyrealsense2 as rs
import numpy as np
import cv2
import zmq
import time
import math

# --- CONFIGURATION ---
DEPTH_CAP_METERS = 1.75
MIN_DEPTH_METERS = 0.15
MIN_CONTOUR_AREA = 50      # Ignores tiny IR sensor noise specks
MAX_CONTOUR_AREA = 4000    # Ignores ceilings, walls, and large bodies (Reduced from 15000)
TOP_CROP_PERCENTAGE = 0.10 # Percentage of the top of the frame to ignore (e.g., 0.10 = 10%)
DEPTH_FPS = 90 # Unleashing full 90 FPS depth tracking!
COLOR_FPS = 60 # RGB sensor maximum limit at 848x480
ROTATION_CW = True  # Clockwise rotation from the back of the camera

# ZMQ Setup
ZMQ_SUB_PORT = 5555 # Receives from controller (e.g., SHOOT command)
ZMQ_PUB_PORT = 5556 # Sends results back to controller
context = zmq.Context()
sub_socket = context.socket(zmq.SUB)
sub_socket.bind(f"tcp://*:{ZMQ_SUB_PORT}")
sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")

pub_socket = context.socket(zmq.PUB)
pub_socket.bind(f"tcp://*:{ZMQ_PUB_PORT}")

def get_3d_point(intrinsics, u_rot, v_rot, depth_val, rotated_cw=True):
    """ Converts a rotated pixel coordinate back to original camera 3D space """
    # Original image is 848 (width/u) x 480 (height/v)
    # Rotated screen is 480 (width/x) x 848 (height/y)
    if rotated_cw:
        u_orig = v_rot
        v_orig = 479 - u_rot 
    else:
        u_orig = 479 - v_rot
        v_orig = u_rot
    
    # Deprojeact pixel to 3D point (X, Y, Z) in meters using the RealSense API
    point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [u_orig, v_orig], depth_val)
    return point_3d

def main():
    # 0. Force Hardware Reset to clear Linux USB locks
    ctx = rs.context()
    devices = ctx.query_devices()
    if len(devices) > 0:
        print("[INFO] Sending Hardware Reset to clear zombie states...")
        devices[0].hardware_reset()
        print("[INFO] Waiting 3 seconds for USB to re-enumerate...")
        time.sleep(3)
    else:
        print("[ERROR] No RealSense devices found! Check cable.")
        return

    # 1. RealSense Setup
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Enable streams with maximum supported resolution and framerates
    config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, DEPTH_FPS)
    config.enable_stream(rs.stream.color, 848, 480, rs.format.rgb8, COLOR_FPS)
    
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()
    
    # Safely apply hardware settings (Catches "Device or resource busy" Linux errors)
    try:
        if depth_sensor.supports(rs.option.visual_preset):
            depth_sensor.set_option(rs.option.visual_preset, 3) # 3 = High Accuracy
        if depth_sensor.supports(rs.option.laser_power):
            depth_sensor.set_option(rs.option.laser_power, 360) # Max IR contrast for white shuttle
    except Exception as e:
        print(f"[WARNING] Could not set hardware options (Device busy). Using defaults. Error: {e}")
        
    depth_scale = depth_sensor.get_depth_scale()
    align = rs.align(rs.stream.color)
    intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

    print(f"[INFO] RealSense Initialized (Scale: {depth_scale}).")
    print(f"[INFO] ZMQ Listening on port {ZMQ_SUB_PORT}... Waiting for SHOOT command.")
    
    state = "IDLE" # States: IDLE, TRACKING_DEPTH, TRACKING_RGB
    trajectory_data = [] # Stores (time, X, Y, Z)
    shot_params = {}

    try:
        while True:
            # 3. ZMQ Listener (Non-blocking)
            try:
                msg = sub_socket.recv_json(flags=zmq.NOBLOCK)
                if msg.get("command") == "SHOOT":
                    print(f"\n[EVENT] SHOOT command received! P:{msg.get('pitch')} Y:{msg.get('yaw')} S:{msg.get('speed')}")
                    shot_params = msg
                    state = "TRACKING_DEPTH"
                    trajectory_data.clear()
                    start_time = time.time()
            except zmq.Again:
                pass

            # Fetch coherent frames with error handling
            try:
                frames = pipeline.wait_for_frames(5000)
            except RuntimeError:
                print("[WARNING] Frame timeout! USB bus is choked or camera is hung. Retrying...")
                continue

            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                continue

            # Convert to numpy arrays (Raw depth is 16-bit)
            raw_depth = np.asanyarray(depth_frame.get_data())
            raw_color = np.asanyarray(color_frame.get_data())
            
            # Rotate 90 degrees for your vertical mount and force contiguous memory layout
            if ROTATION_CW:
                raw_depth = np.rot90(raw_depth, k=-1).copy()
                raw_color = np.rot90(raw_color, k=-1).copy()
            else:
                raw_depth = np.rot90(raw_depth, k=1).copy()
                raw_color = np.rot90(raw_color, k=1).copy()

            # Convert 16-bit depth to float32 meters (computationally light via numpy broadcasting)
            depth_meters = raw_depth * depth_scale

            # --- CROP DEPTH FEED (TOP AND BOTTOM) ---
            # Zeros out the bottom half of the depth frame to ignore the floor/stands
            half_height = depth_meters.shape[0] // 2
            depth_meters[half_height:, :] = 0.0
            
            # Zeros out the top percentage of the depth frame to ignore ceiling edges
            top_crop_pixels = int(depth_meters.shape[0] * TOP_CROP_PERCENTAGE)
            depth_meters[:top_crop_pixels, :] = 0.0

            # --- Generate Depth Visualization ---
            # Colorizes depth (0m to DEPTH_CAP_METERS maps to OpenCV JET colors)
            viz_frame = cv2.applyColorMap(cv2.convertScaleAbs(depth_meters, alpha=255.0 / DEPTH_CAP_METERS), cv2.COLORMAP_JET)

            # 2. 4-Layer Array Creation [R, G, B, D]
            depth_channel = np.expand_dims(depth_meters, axis=2)
            rgbd_tensor = np.concatenate((raw_color, depth_channel), axis=2)

            # 4. Depth Trajectory Tracking
            if state == "TRACKING_DEPTH":
                # Create 8-bit binary mask based on depth thresholds
                depth_mask = cv2.inRange(depth_meters, MIN_DEPTH_METERS, DEPTH_CAP_METERS)
                
                # Visually dim out-of-range pixels to dark gray to highlight the active tracking slice
                viz_frame[depth_mask == 0] = [20, 20, 20]
                
                # Find the shuttle blob in the masked image
                contours, _ = cv2.findContours(depth_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                # Apply size constraints to filter out ceiling/walls and noise
                valid_contours = [c for c in contours if MIN_CONTOUR_AREA < cv2.contourArea(c) < MAX_CONTOUR_AREA]
                
                if valid_contours:
                    # Assume largest valid contour in depth range is the shuttle
                    largest_contour = max(valid_contours, key=cv2.contourArea)
                    
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    cx, cy = x + w//2, y + h//2
                    
                    # Get exact raw depth at center pixel (using the 16-bit converted float)
                    z_val = depth_meters[cy, cx]
                    
                    if z_val > 0:
                        # Convert to true 3D spatial coordinates
                        pt_3d = get_3d_point(intrinsics, cx, cy, z_val, ROTATION_CW)
                        
                        current_t = time.time() - start_time
                        trajectory_data.append((current_t, pt_3d[0], pt_3d[1], pt_3d[2]))
                        
                        # Draw bounding box on the depth visualization
                        cv2.rectangle(viz_frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                        cv2.circle(viz_frame, (cx, cy), 3, (0, 0, 255), -1)

                # Transition Condition: Timeout or Shuttle has left the depth zone
                if time.time() - start_time > 1.5 or (len(trajectory_data) > 0 and not contours):
                    print(f"[INFO] Depth Tracking Finished. Captured {len(trajectory_data)} frames.")
                    
                    # 5. Extract Initial Velocity (V0) and Launch Angles
                    if len(trajectory_data) >= 2:
                        t1, x1, y1, z1 = trajectory_data[0]
                        
                        # Find a point approx 20cm further down the Z axis
                        idx2 = -1
                        for i in range(1, len(trajectory_data)):
                            if trajectory_data[i][3] - z1 >= 0.20:
                                idx2 = i
                                break
                        
                        # If 20cm wasn't reached before leaving frame, use the last captured point
                        if idx2 == -1: idx2 = -1 
                        
                        t2, x2, y2, z2 = trajectory_data[idx2]
                        
                        dt = t2 - t1
                        if dt > 0:
                            vx = (x2 - x1) / dt
                            vy = (y2 - y1) / dt
                            vz = (z2 - z1) / dt
                            
                            v0 = math.sqrt(vx**2 + vy**2 + vz**2)
                            # Pitch angle (Elevation from horizontal)
                            launch_angle_deg = math.degrees(math.atan2(vy, math.sqrt(vx**2 + vz**2)))
                            
                            print(f"[RESULT] V0: {v0:.2f} m/s | Vertical Angle: {launch_angle_deg:.1f} deg")
                            
                            # Publish intermediate result to controller
                            pub_socket.send_json({
                                "status": "launch_tracked",
                                "v0": round(v0, 2),
                                "angle": round(launch_angle_deg, 2)
                            })
                    
                    # Move to next state (Step 6)
                    state = "TRACKING_RGB"
                    print("[INFO] Transitioning to RGB Landing Tracking...")

            # Show preview
            cv2.imshow("Turret Depth Tracker", viz_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
