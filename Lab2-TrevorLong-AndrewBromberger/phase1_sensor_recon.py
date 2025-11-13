#!/usr/bin/env python3
"""
Lab 2 - Phase 1: AprilTag Sensor Characterization

Objectives:
1. Implement coordinate frame transformations between AprilTag and drone frames
2. Characterize AprilTag detection performance at various distances
3. Analyze position measurement noise and detection reliability

Implement:
- Coordinate transformation functions
- Detection characterization analysis
- Performance metrics calculation
"""

import os
import time
import csv
import numpy as np
import matplotlib.pyplot as plt
from djitellopy import Tello
import cv2
from pupil_apriltags import Detector

# Configuration
OUTPUT_DIR = "Lab2-Phase1"
SAMPLE_RATE = 10  # Hz
TEST_DISTANCES = [1.0, 1.5, 2.0, 2.5, 3.0]  # meters
SAMPLES_PER_DISTANCE = 300  # 30 seconds at 10 Hz
SAMPLE_DT = 0.05  # 20 Hz sampling rate
SAMPLE_TIME = 30

# AprilTag setup
TAG_SIZE = 0.162  # meters
CAMERA_PARAMS = [921.170702, 919.018377, 459.904354, 351.238301]

def ensure_dir(path):
    """Create directory if it doesn't exist"""
    os.makedirs(path, exist_ok=True)

class CoordinateTransformer:
    """
    Coordinate transformations between AprilTag and drone frames.
    
    AprilTag Frame:
        x_tag: lateral displacement (positive = drone is to the right of tag)
        y_tag: vertical displacement (positive = drone is above tag)
        z_tag: distance from tag (positive = drone is away from tag)
    
    Drone Body Frame (for velocity commands):
        x_drone: forward/backward (positive = forward)
        y_drone: left/right (positive = right)
        z_drone: up/down (positive = up)
    """
    
    def __init__(self):
        pass
    
    def tag_to_drone_position(self, x_tag, y_tag, z_tag):
        """
        Transform AprilTag measurements to drone body frame.
        
        TODO: Implement the coordinate transformation.
        
        Consider:
        - How does distance from tag relate to forward/backward position?
        - Which axes map directly between frames?
        - Do any transformations require sign changes?
        
        Args:
            x_tag: Lateral position in tag frame [m]
            y_tag: Vertical position in tag frame [m]
            z_tag: Distance from tag [m]
        
        Returns:
            x_drone: Forward/backward position [m]
            y_drone: Left/right position [m]
            z_drone: Up/down position [m]
        """
        # TODO: Implement coordinate transformation
        # Hint: Consider the relationship between tag distance and drone forward position
        x_drone = -z_tag  # REPLACE THIS
        y_drone = x_tag  # REPLACE THIS
        z_drone = y_tag  # REPLACE THIS
        
        return x_drone, y_drone, z_drone
    
    def position_error_to_velocity_command(self, x_error, y_error, z_error, 
                                          kp_x=0.5, kp_y=0.5, kp_z=0.5):
        """
        Convert position errors to velocity commands using proportional control.
        
        TODO: Implement proportional control with velocity limiting.
        
        Args:
            x_error: Position error in x [m]
            y_error: Position error in y [m]
            z_error: Position error in z [m]
            kp_x, kp_y, kp_z: Proportional gains
        
        Returns:
            vx, vy, vz: Velocity commands [m/s], limited to ±0.3 m/s
        """
        # TODO: Implement proportional control
        # Hint: velocity = gain * error, then apply safety limits
        max_velocity = 0.3
        
        vx = -kp_x*x_error
        if vx > max_velocity:
            vx = max_velocity
        elif vx<-max_velocity:
            vx = -max_velocity

        vy = -kp_y*y_error
        if vy > max_velocity:
            vy = max_velocity
        elif vy<-max_velocity:
            vy = -max_velocity

        vz = -kp_z*z_error
        if vz > max_velocity:
            vz = max_velocity
        elif vz<-max_velocity:
            vz = -max_velocity

        return vx, vy, vz

def get_apriltag_detection(frame, detector):
    """
    Detect AprilTag in frame and return pose.
    
    Args:
        frame: Camera frame (BGR image)
        detector: AprilTag detector object
    
    Returns:
        dict with 'detected' (bool) and 'x', 'y', 'z' (float or nan)
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    detections = detector.detect(
        gray,
        estimate_tag_pose=True,
        camera_params=CAMERA_PARAMS,
        tag_size=TAG_SIZE
    )
    
    if len(detections) > 0:
        detection = detections[0]
        pose = detection.pose_t.flatten()
        
        return {
            'detected': True,
            'x': pose[0],
            'y': pose[1],
            'z': pose[2]
        }
    else:
        return {
            'detected': False,
            'x': np.nan,
            'y': np.nan,
            'z': np.nan
        }

def move_to_distance(tello, detector, target_distance, timeout=30.0):
    """
    Move drone to target distance from AprilTag using simple proportional control.
    
    Args:
        tello: Tello drone object
        detector: AprilTag detector
        target_distance: Desired distance from tag [m]
        timeout: Maximum time to reach target [s]
    """
    print(f"\nPositioning to {target_distance:.1f}m from tag...")
    
    transformer = CoordinateTransformer()
    start_time = time.time()
    settled_count = 0
    required_settled = 20
    
    while time.time() - start_time < timeout:
        frame = tello.get_frame_read().frame
        detection = get_apriltag_detection(frame, detector)
        
        if detection['detected']:
            current_distance = detection['z']
            error = target_distance - current_distance
            
            if abs(error) < 0.05:
                settled_count += 1
                if settled_count >= required_settled:
                    print(f"Reached target: {current_distance:.2f}m")
                    break
            else:
                settled_count = 0
            
            # Simple proportional control for distance only
            velocity_cmd = np.clip(0.5 * error, -0.3, 0.3)
            rc_cmd = int(-velocity_cmd * 100)
            
            tello.send_rc_control(0, rc_cmd, 0, 0)
        else:
            tello.send_rc_control(0, 0, 0, 0)
        
        time.sleep(0.1)
    
    tello.send_rc_control(0, 0, 0, 0)
    time.sleep(1.0)

def characterize_apriltag_detection():
    """
    Main characterization function. Collects detection data at multiple distances.
    
    Returns:
        csv_filename: Path to saved data file
    """
    ensure_dir(OUTPUT_DIR)
    
    tello = Tello()
    print("Connecting to Tello...")
    tello.connect()
    print("Connected!")
    
    print("Starting video stream...")
    tello.streamon()
    time.sleep(2)
    
    detector = Detector(families='tag36h11')
    controller = PIDController(kp=1,ki=0.05,kd=.01)
    
    try:
        battery = tello.get_battery()
        print(f"Battery level: {battery}%")
        if battery < 30:
            print("ERROR: Battery too low. Need at least 30%")
            return None
        
        input("Press Enter to takeoff and begin characterization...")
        
        print("\nTaking off...")
        tello.takeoff()
        time.sleep(2)
        
        print("Adjusting altitude...")
        move_to_altitude(tello,1.5,controller,SAMPLE_TIME)

        time.sleep(2)
        
        print("\nStarting AprilTag characterization")
        print(f"Test distances: {TEST_DISTANCES}")
        print(f"Samples per distance: {SAMPLES_PER_DISTANCE}")
        
        csv_filename = os.path.join(OUTPUT_DIR, "apriltag_characterization.csv")
        
        with open(csv_filename, 'w', newline='') as csvfile:
            fieldnames = ['test_distance', 'sample_num', 'detected', 
                         'x_tag', 'y_tag', 'z_tag', 'battery']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            
            for test_idx, test_distance in enumerate(TEST_DISTANCES):
                print(f"\nTest {test_idx+1}/{len(TEST_DISTANCES)}: {test_distance:.1f}m")
                
                move_to_distance(tello, detector, test_distance)
                
                print(f"Collecting {SAMPLES_PER_DISTANCE} samples...")
                detection_count = 0
                
                for sample_num in range(SAMPLES_PER_DISTANCE):
                    frame = tello.get_frame_read().frame
                    detection = get_apriltag_detection(frame, detector)
                    battery = tello.get_battery()
                    
                    if detection['detected']:
                        detection_count += 1
                    
                    row = {
                        'test_distance': test_distance,
                        'sample_num': sample_num,
                        'detected': 1 if detection['detected'] else 0,
                        'x_tag': detection['x'],
                        'y_tag': detection['y'],
                        'z_tag': detection['z'],
                        'battery': battery
                    }
                    writer.writerow(row)
                    
                    if sample_num % 50 == 0 and sample_num > 0:
                        rate = (detection_count / (sample_num + 1)) * 100
                        print(f"  Sample {sample_num}/{SAMPLES_PER_DISTANCE} | "
                              f"Detection: {rate:.1f}% | Battery: {battery}%")
                    
                    tello.send_rc_control(0, 0, 0, 0)
                    time.sleep(1.0 / SAMPLE_RATE)
                
                final_rate = (detection_count / SAMPLES_PER_DISTANCE) * 100
                print(f"Completed! Detection rate: {final_rate:.1f}%")
        
        print("\nData collection complete!")
        print(f"Data saved to: {csv_filename}")
        
        print("\nLanding...")
        tello.land()
        time.sleep(2)
        
        return csv_filename
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user!")
        try:
            tello.land()
        except:
            pass
        return None
        
    except Exception as e:
        print(f"\nERROR during characterization: {e}")
        import traceback
        traceback.print_exc()
        try:
            tello.land()
        except:
            pass
        return None
    finally:
        try:
            tello.streamoff()
            tello.end()
        except:
            pass

def analyze_characterization_data(csv_filename):
    """
    Analyze collected characterization data and generate metrics.
    
    TODO: Implement analysis functions to calculate:
    - Detection success rate per distance
    - Position noise (standard deviation) per distance
    - Maximum consecutive dropout duration per distance
    
    Args:
        csv_filename: Path to CSV data file
    """
    import pandas as pd
    
    print(f"\nAnalyzing characterization data from: {csv_filename}")
    
    df = pd.read_csv(csv_filename)
    
    results = {}
    
    for test_distance in df['test_distance'].unique():
        distance_data = df[df['test_distance'] == test_distance]
        
        # TODO: Calculate detection success rate
        # Hint: Count detected samples / total samples
        total_samples = len(distance_data)
        detected_samples = sum(distance_data['detected'].values)  # REPLACE THIS
        detection_rate = detected_samples/total_samples  # REPLACE THIS
        
        # TODO: Calculate position noise (standard deviation)
        # Hint: Use np.nanstd() to handle NaN values from undetected samples
        x_noise = np.nanstd(distance_data['x_tag'])  # REPLACE THIS
        y_noise = np.nanstd(distance_data['y_tag'])  # REPLACE THIS
        z_noise = np.nanstd(distance_data['z_tag'])  # REPLACE THIS
        
        # TODO: Calculate maximum consecutive dropout duration
        # Hint: Loop through detected array, count consecutive zeros
        detected_array = distance_data['detected'].values
        zero_len = 0
        max_dropout = 0
        for isamp in np.arange(0,total_samples):
            if detected_array[isamp] == 0:
                zero_len +=1
                if zero_len>max_dropout:
                    max_dropout=zero_len
            else:
                if zero_len>max_dropout:
                    max_dropout=zero_len
                zero_len = 0


        max_dropout_seconds = max_dropout*1/SAMPLE_RATE  # REPLACE THIS (convert to seconds)
        
        results[test_distance] = {
            'detection_rate': detection_rate,
            'x_noise': x_noise,
            'y_noise': y_noise,
            'z_noise': z_noise,
            'max_dropout': max_dropout_seconds
        }
        
        print(f"\nDistance: {test_distance:.1f}m")
        print(f"  Detection rate:  {detection_rate*100:6.1f}%")
        print(f"  Position noise:")
        print(f"    X: {x_noise*1000:6.2f} mm")
        print(f"    Y: {y_noise*1000:6.2f} mm")
        print(f"    Z: {z_noise*1000:6.2f} mm")
        print(f"  Max dropout:     {max_dropout_seconds:6.2f} s")
    
    create_characterization_plots(results)
    
    import json
    results_file = os.path.join(OUTPUT_DIR, 'characterization_results.json')
    with open(results_file, 'w') as f:
        json.dump({str(k): v for k, v in results.items()}, f, indent=2)
    
    print(f"\nResults saved to {results_file}")

def create_characterization_plots(results):
    """
    Generate characterization plots.
    
    Args:
        results: Dictionary of analysis results per distance
    """
    distances = sorted(results.keys())
    detection_rates = [results[d]['detection_rate'] * 100 for d in distances]
    x_noise = [results[d]['x_noise'] * 100 for d in distances]
    y_noise = [results[d]['y_noise'] * 100 for d in distances]
    z_noise = [results[d]['z_noise'] * 100 for d in distances]
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    
    ax1.plot(distances, detection_rates, 'bo-', linewidth=2, markersize=8)
    ax1.axhline(95, color='g', linestyle='--', label='95% threshold')
    ax1.set_xlabel('Distance from Tag [m]')
    ax1.set_ylabel('Detection Success Rate [%]')
    ax1.set_title('AprilTag Detection Reliability')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax1.set_ylim(0, 105)
    
    ax2.plot(distances, x_noise, 'r.-', label='X noise', markersize=8)
    ax2.plot(distances, y_noise, 'g.-', label='Y noise', markersize=8)
    ax2.plot(distances, z_noise, 'b.-', label='Z noise', markersize=8)
    ax2.set_xlabel('Distance from Tag [m]')
    ax2.set_ylabel('Position Noise Std Dev [cm]')
    ax2.set_title('AprilTag Pose Measurement Noise')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, 'apriltag_characterization.png'), 
                dpi=200, bbox_inches='tight')
    plt.close()
    
    print(f"Plots saved to {OUTPUT_DIR}/apriltag_characterization.png")


def move_to_altitude(tello, target_altitude, pid_controller, duration):
    """
    Move drone to target altitude using PID control for specified duration.
    """
    print(f"    Moving to {target_altitude:.1f}m for {duration:.0f}s...")

    data = {
        'mission_time': [],
        'target_altitude': [],
        'current_altitude': [],
        'error': [],
        'velocity_cmd': [],
        'proportional': [],
        'integral': [],
        'derivative': []
    }

    start_time = time.time()
    next_sample_time = start_time

    while time.time() - start_time < duration:
        current_time = time.time()

        if current_time >= next_sample_time:
            mission_time = current_time - start_time

            # Get sensor readings
            sensor_data = get_sensor_data(tello)
            current_altitude = get_current_altitude(sensor_data)

            # PID control
            velocity_cmd, p_term, i_term, d_term = pid_controller.update(target_altitude, current_altitude)

            # Send control command
            rc_cmd = velocity_to_rc(velocity_cmd)
            tello.send_rc_control(0, 0, rc_cmd, 0)

            # Log data
            data['mission_time'].append(mission_time)
            data['target_altitude'].append(target_altitude)
            data['current_altitude'].append(current_altitude)
            data['error'].append(target_altitude - current_altitude)
            data['velocity_cmd'].append(velocity_cmd)
            data['proportional'].append(p_term)
            data['integral'].append(i_term)
            data['derivative'].append(d_term)

            next_sample_time += SAMPLE_DT

        time.sleep(0.01)

    # Stop motion
    tello.send_rc_control(0, 0, 0, 0)
    final_altitude = get_current_altitude(get_sensor_data(tello))
    print(f"    Reached {final_altitude:.2f}m")

    return data

def get_current_altitude(sensor_data):
    """Get current altitude estimate, preferring sonar if valid"""
    if not np.isnan(sensor_data['height_sonar']) and sensor_data['height_sonar'] > 0.1:
        return sensor_data['height_sonar']
    elif not np.isnan(sensor_data['height_baro']):
        return sensor_data['height_baro']
    else:
        return 1.0  # Fallback estimate

def velocity_to_rc(velocity_mps):
    """Convert velocity in m/s to RC throttle command (-100 to +100)"""
    velocity_mps = max(-1.0, min(1.0, velocity_mps))
    rc_cmd = int(velocity_mps * 100)
    return max(-100, min(100, rc_cmd))


def get_sensor_data(tello):
    """Get sensor data from Tello"""
    try:
        height_sonar = tello.get_distance_tof() / 100.0  # cm -> m
        height_baro = tello.get_barometer() / 100.0  # cm -> m
        velocity_z = tello.get_speed_z() / 100.0  # cm/s -> m/s
        battery = tello.get_battery()  # already in %

        return {
            'height_sonar': height_sonar,
            'height_baro': height_baro,
            'velocity_z': velocity_z,
            'battery': battery
        }
    except Exception as e:
        print(f"Error reading sensors: {e}")
        return {
            'height_sonar': 0.0,
            'height_baro': 0.0,
            'velocity_z': 0.0,
            'battery': 0.0
        }


class PIDController:
    """Simple PID Controller for altitude control"""

    def __init__(self, kp=0.0, ki=0.0, kd=0.0, dt=0.05):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        # Internal state
        self.integral = 0.0
        self.previous_error = 0.0
        self.first_run = True

    def update(self, setpoint, measured_value):
        """
        Update PID controller with new measurement.

        TODO: Implement PID calculation
        1. Calculate error = setpoint - measured_value
        2. Calculate proportional term: P = kp * error
        3. Update and calculate integral term:
           - self.integral += error * self.dt  (accumulate error over time)
           - I = ki * self.integral
        4. Calculate derivative term:
           - If first run: D = 0 (no previous error to compare)
           - Otherwise: D = kd * (error - self.previous_error) / self.dt
           - Set self.first_run = False after first calculation
        5. Calculate total output = P + I + D
        6. Clamp output to [-1.0, 1.0] for safety
        7. Store current error as self.previous_error for next iteration
        8. Return: (output, P, I, D) for logging/plotting

        Args:
            setpoint: Desired altitude [m]
            measured_value: Current altitude [m]

        Returns:
            output: Control output (velocity command) [m/s]
            proportional: P term value
            integral: I term value
            derivative: D term value
        """
        # TODO: Implement PID calculation here (~15-20 lines)

        error = setpoint - measured_value  # TODO
        proportional = self.kp * error  # TODO
        self.integral += error * self.dt
        integral = self.ki * self.integral  # TODO
        if self.first_run:
            derivative = 0
        else:
            derivative = (error - self.previous_error) / self.dt  # TODO

        output = proportional + integral + derivative  # TODO

        if self.first_run: self.first_run = False

        if output > 1:
            output = 1
        elif output < -1:
            output = -1
        self.previous_error = error

        return output, proportional, integral, derivative

    def reset(self):
        """Reset PID internal state"""
        self.integral = 0.0
        self.previous_error = 0.0
        self.first_run = True

def main():
    print("Lab 2 - Phase 1: AprilTag Sensor Characterization")
    print("="*60)
    
    # csv_filename = characterize_apriltag_detection()
    csv_filename = 'Lab2-Phase1/apriltag_characterization.csv'
    
    if csv_filename:
        analyze_characterization_data(csv_filename)
        print("\nPhase 1 Complete!")
        print(f"Files saved in: {OUTPUT_DIR}/")
    else:
        print("\nCharacterization failed or was cancelled")

if __name__ == "__main__":
    main()