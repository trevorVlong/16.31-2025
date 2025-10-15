#!/usr/bin/env python3
"""
Lab 1 - Phase 2: Step Response Calibration

This script performs systematic step response testing to calibrate the drone's
altitude sensors and understand system dynamics. It commands specific vertical
velocities and measures the resulting altitude changes to build calibration models.
"""

import os
import time
import csv
import json
import numpy as np
import matplotlib.pyplot as plt
from djitellopy import Tello

# Configuration
OUTPUT_DIR = "Lab1-Phase2"
SAMPLE_DT = 0.05  # 20 Hz sampling rate

# Step velocity sequence: (velocity_cmd [m/s], duration [s])
STEP_SEQUENCE = [
    (0.0, 3.0),   # Initial hold
    (0.2, 3.5),   # Gentle up
    (0.0, 3.0),   # Hold
    (-0.15, 4.0), # Gentle down
    (0.0, 3.0),   # Hold
    (0.3, 3.0),   # Faster up
    (0.0, 3.0),   # Hold
    (-0.25, 3.0), # Faster down
    (0.0, 3.0),   # Final hold
]

def ensure_dir(path):
    os.makedirs(path, exist_ok=True)

def velocity_to_rc(velocity_mps):
    """Convert velocity in m/s to RC throttle command (-100 to +100)"""
    rc_cmd = int(velocity_mps * 100)
    return max(-100, min(100, rc_cmd))

# ============================================================================
# PROVIDED: Sensor reading function (reuse from Phase 1)
# ============================================================================
def get_sensor_data(tello):
    """
    Get sensor data from Tello using direct API methods.
    Returns values in SI units with proper error handling.
    """
    try:
        # Use direct API methods for reliable measurements
        height_sonar = tello.get_distance_tof() / 100.0  # cm -> m
        height_baro = tello.get_barometer() / 100.0       # cm -> m  
        velocity_z = tello.get_speed_z() / 100.0          # cm/s -> m/s
        battery = tello.get_battery()                     # already in %
        
        # For acceleration, still need to use state dict as no direct methods
        state = tello.get_current_state()
        def safe_get(key, default=0.0):
            try:
                val = state.get(key, default)
                if val is None:
                    return default
                return float(val) / 100.0  # cm/s^2 -> m/s^2
            except (ValueError, TypeError):
                return default
        
        ax = safe_get('agx', 0.0)
        ay = safe_get('agy', 0.0)
        az = safe_get('agz', 0.0)
        
        return {
            'height_sonar': height_sonar,
            'height_baro': height_baro,
            'velocity_z': velocity_z,
            'ax': ax,
            'ay': ay, 
            'az': az,
            'battery': battery
        }
        
    except Exception as e:
        print(f"Error reading sensors: {e}")
        return {
            'height_sonar': 0.0,
            'height_baro': 0.0,
            'velocity_z': 0.0,
            'ax': 0.0,
            'ay': 0.0,
            'az': 0.0,
            'battery': 0.0
        }

# ============================================================================
# PROVIDED: Data collection routine
# Study this to understand step response testing methodology
# ============================================================================
def run_step_calibration():
    """Execute the step response calibration test"""
    ensure_dir(OUTPUT_DIR)
    
    # Initialize drone
    tello = Tello()
    print("Connecting to Tello...")
    tello.connect()
    
    try:
        # Check battery
        battery = tello.get_battery()
        print(f"Battery level: {battery}%")
        if battery < 40:
            print("ERROR: Battery too low for calibration. Need at least 40%")
            return None
        
        print("Taking off...")
        tello.takeoff()
        time.sleep(3)  # Allow stabilization
        
        # Get initial readings
        initial_data = get_sensor_data(tello)
        initial_altitude = initial_data['height_sonar']
        if initial_altitude < 0.1:  # If sonar seems invalid
            initial_altitude = initial_data['height_baro']
        
        print(f"Initial altitude: {initial_altitude:.2f}m")
        print(f"Starting step response calibration...")
        
        # Calculate total test duration
        total_duration = sum(duration for _, duration in STEP_SEQUENCE)
        print(f"Test sequence duration: {total_duration:.1f} seconds")
        
        # Start mission and open CSV
        mission_start_time = time.time()
        csv_filename = os.path.join(OUTPUT_DIR, "step_calibration_data.csv")
        
        with open(csv_filename, 'w', newline='') as csvfile:
            # Define exact CSV columns
            fieldnames = [
                'mission_time', 'step_index', 'velocity_cmd', 'step_time_remaining',
                'height_sonar', 'height_baro', 'velocity_z',
                'ax', 'ay', 'az', 'battery'
            ]
            
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            
            # Execute step sequence
            next_sample_time = mission_start_time
            samples_collected = 0
            
            for step_index, (velocity_cmd, duration) in enumerate(STEP_SEQUENCE):
                print(f"\nStep {step_index + 1}/{len(STEP_SEQUENCE)}: {velocity_cmd:+.1f} m/s for {duration:.1f}s")
                
                step_start_time = time.time()
                step_end_time = step_start_time + duration
                rc_cmd = velocity_to_rc(velocity_cmd)
                
                # Execute this step
                while time.time() < step_end_time:
                    current_time = time.time()
                    
                    # Send RC command continuously
                    tello.send_rc_control(0, 0, rc_cmd, 0)
                    
                    # Sample at regular intervals
                    if current_time >= next_sample_time:
                        mission_time = current_time - mission_start_time
                        step_time_remaining = step_end_time - current_time
                        
                        sensor_data = get_sensor_data(tello)
                        
                        # Create row with EXACT fieldnames
                        row = {
                            'mission_time': mission_time,
                            'step_index': step_index,
                            'velocity_cmd': velocity_cmd,
                            'step_time_remaining': step_time_remaining,
                            'height_sonar': sensor_data['height_sonar'],
                            'height_baro': sensor_data['height_baro'],
                            'velocity_z': sensor_data['velocity_z'],
                            'ax': sensor_data['ax'],
                            'ay': sensor_data['ay'],
                            'az': sensor_data['az'],
                            'battery': sensor_data['battery']
                        }
                        
                        writer.writerow(row)
                        samples_collected += 1
                        
                        # Progress update every second
                        if samples_collected % 20 == 0:
                            current_alt = sensor_data['height_sonar'] if sensor_data['height_sonar'] > 0.1 else sensor_data['height_baro']
                            print(f"  t={mission_time:.1f}s: Alt={current_alt:.2f}m, Bat={sensor_data['battery']:.0f}%")
                        
                        next_sample_time += SAMPLE_DT
                    
                    time.sleep(0.01)
                
                # Stop movement between steps
                tello.send_rc_control(0, 0, 0, 0)
        
        # Stop all motion
        tello.send_rc_control(0, 0, 0, 0)
        print("Step sequence complete!")
        
        # Land
        print("Landing...")
        tello.land()
        time.sleep(2)
        
        print(f"Data saved to {csv_filename}")
        print(f"Total samples collected: {samples_collected}")
        
        return csv_filename
        
    except Exception as e:
        print(f"Error during calibration: {e}")
        try:
            tello.send_rc_control(0, 0, 0, 0)
            tello.land()
        except:
            pass
        return None
    finally:
        try:
            tello.end()
        except:
            pass
        print("Tello connection closed")

# ============================================================================
# TODO: Implement analysis functions
# This is where you analyze the step response data to build calibration models
# ============================================================================

def analyze_step_data(csv_filename):
    """Analyze the step response data and generate calibration results"""
    
    print(f"\nAnalyzing step calibration data from {csv_filename}...")
    
    # Load data
    import pandas as pd
    try:
        df = pd.read_csv(csv_filename)
    except Exception as e:
        print(f"Error loading CSV: {e}")
        return
    
    print(f"Loaded {len(df)} samples")
    
    # Extract arrays
    mission_time = df['mission_time'].values
    step_indices = df['step_index'].values
    velocity_cmds = df['velocity_cmd'].values
    height_sonar = df['height_sonar'].values
    height_baro = df['height_baro'].values
    velocity_measured = df['velocity_z'].values
    
    ensure_dir(OUTPUT_DIR)
    
    # ==================== PROVIDED: PLOTTING ====================
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8))
    
    # Commanded vs measured velocity
    ax1.plot(mission_time, velocity_cmds, 'r-', linewidth=2, label='Commanded vz [m/s]')
    ax1.plot(mission_time, velocity_measured*-100, 'b-', linewidth=1, alpha=0.8, label='Measured vz [m/s]')
    ax1.set_ylabel('Velocity [m/s]')
    ax1.set_title('Velocity Command vs Measured Response')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Height response
    ax2.plot(mission_time, height_sonar, 'g-', linewidth=1.5, label='Sonar Height [m]', alpha=0.8)
    ax2.plot(mission_time, height_baro, 'm-', linewidth=1.5, label='Barometer Height [m]', alpha=0.8)
    ax2.set_xlabel('Mission Time [s]')
    ax2.set_ylabel('Height [m]')
    ax2.set_title('Height Response from Both Sensors')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.set_ylim(0,np.max(height_sonar)*1.5)
    
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, 'step_response.png'), dpi=200, bbox_inches='tight')
    plt.close()
    
    # ==================== TODO: STEP-BY-STEP ANALYSIS ====================
    print(f"\nStep-by-step analysis:")
    
    calibration_table = []
    
    # TODO: For each step in the sequence, calculate actual vs expected height change
    # 1. Loop through unique step_index values: sorted(df['step_index'].unique())
    # 2. For each step:
    #    a. Extract data for that step: df[df['step_index'] == step_idx]
    #    b. Get velocity command (first value in step)
    #    c. Calculate duration = len(step_data) * SAMPLE_DT
    #    d. Calculate expected change = velocity_cmd * duration
    #    e. Measure actual change for both sensors:
    #       - Use first 10 samples for pre-height (median for robustness)
    #       - Use last 10 samples for post-height (median)
    #       - actual_change = post_height - pre_height
    #    f. Store results in calibration_table list
    #    g. Print summary for each step
    #
    # Hint: Skip steps with < 10 samples
    # Hint: Use .iloc[:10] for first 10, .iloc[-10:] for last 10
    # Hint: Use .median() instead of .mean() to reduce noise effects

    indices = np.unique(step_indices)
    print(f'run indices: {indices}')
    results = {'dz_expected': np.empty((len(indices), )),
               'dz_baro': np.empty((len(indices), )),
               'dz_sonar': np.empty((len(indices), )),
               'velocity_cmd':np.empty((len(indices), )),
               }

    for index in indices:
        print(30*"=")
        print(f'results for index {index}')
        step_data = df[df['step_index']==index]
        if len(step_data) < 10:
            print('skipping because len(step) < 10')
            continue
        velocity_cmd = step_data['velocity_cmd'].iloc[0]
        results['velocity_cmd'][index] = velocity_cmd
        print(f'Velocity commanded: {velocity_cmd} m/s')
        duration = SAMPLE_DT * len(step_data)
        print(f'movement time: {duration} s')
        results['dz_expected'][index] = velocity_cmd*duration
        # print(f'Expected dz: {results["dz_expected"]} m')
        baro_start = step_data['height_baro'].iloc[:10]
        baro_stop = step_data['height_baro'].iloc[-10:]
        results['dz_baro'][index] = baro_stop.median()-baro_start.median()
        # print(f'Baro dz: {results["dz_baro"]} m')
        sonar_start = step_data['height_sonar'].iloc[:10]
        sonar_stop = step_data['height_sonar'].iloc[-10:]
        results['dz_sonar'][index] = sonar_stop.median() - sonar_start.median()
        # print(f'Sonar dz: {results["dz_sonar"]} m')
        print(50*'-')
        print(f'dz expected (m) \t dz_baro (m)\t dz_sonar (m)\n'
              f'{round(results["dz_expected"][index],2)} \t\t\t\t {round(results["dz_baro"][index],2)}\t'
              f'\t\t\t{round(results["dz_sonar"][index],2)}')
        print(50 * '-')


    print("TODO: Implement step-by-step analysis")
    
    # ==================== TODO: CALIBRATION REGRESSION ====================
    print(f"\nCalibration analysis:")
    
    # TODO: Perform linear regression to find calibration factors
    # 1. Extract only steps where velocity != 0 (movement steps)
    # 2. Create arrays of expected vs actual changes for both sensors
    # 3. Use np.polyfit(expected, actual, 1) to fit line
    #    - Returns [scale, bias] where: actual = scale * expected + bias
    # 4. Calculate R-squared for goodness of fit:
    #    - Predict: actual_pred = scale * expected + bias
    #    - ss_res = sum((actual - actual_pred)²)
    #    - ss_tot = sum((actual - mean(actual))²)
    #    - R² = 1 - (ss_res / ss_tot)
    # 5. Do this for both sonar and barometer
    # 6. Calculate RMSE for both sensors
    # 7. Print calibration equations and metrics
    #
    # Hint: Perfect calibration would be scale=1.0, bias=0.0, R²=1.0
    # Hint: Higher R² = better linear relationship
    
    print("TODO: Implement calibration regression")
    results = df.from_dict(results)
    results_nz = results[np.abs(results['velocity_cmd']) > 0]

    [baro_scale,baro_bias] = np.polyfit(results_nz['dz_expected'],results_nz['dz_baro'],1)
    [sonar_scale, sonar_bias] = np.polyfit(results_nz['dz_expected'], results_nz['dz_sonar'], 1)

    print('sensor\t sse \t ss \t R^2 \t RMSE')
    # baro prediction
    baro_prediction = baro_scale*results_nz['dz_expected'] + baro_bias
    sse_baro = np.sum( (baro_prediction-results_nz['dz_baro'])**2)
    ss_baro = np.sum( (baro_prediction-results_nz['dz_baro'].mean())**2)
    R2_baro = 1-sse_baro/ss_baro
    N = len(baro_prediction)
    RMSE_baro = np.sqrt(1/N * sse_baro)
    print(f'baro : \t {round(sse_baro,2)} \t {round(ss_baro,2)}\t {round(R2_baro,3)} \t {round(RMSE_baro,3)}')

    # sonar prediction
    sonar_prediction = sonar_scale * results_nz['dz_expected'] + sonar_bias
    sse_sonar = np.sum((sonar_prediction - results_nz['dz_sonar']) ** 2)
    ss_sonar = np.sum((sonar_prediction - results_nz['dz_sonar'].mean()) ** 2)
    R2_sonar = 1-sse_sonar/ss_sonar
    N = len(baro_prediction)
    RMSE_sonar = np.sqrt(1 / N * sse_sonar)
    print(f'sonar:\t {round(sse_sonar,3)} \t {round(ss_sonar,2)}\t {round(R2_sonar,3)} \t {round(RMSE_sonar,3)}')


    
    # Dummy values for now - replace with your calculations
    sonar_scale, sonar_bias, sonar_r2 = sonar_scale, sonar_bias, R2_sonar
    baro_scale, baro_bias, baro_r2 = baro_scale, baro_bias, R2_baro
    sonar_rmse, baro_rmse = RMSE_sonar, RMSE_baro
    
    # ==================== TODO: SAVE RESULTS ====================
    
    # TODO: Save calibration table to CSV
    # Use pandas: pd.DataFrame(calibration_table).to_csv(...)
    results_nz[['dz_expected','dz_baro','dz_sonar']].to_csv('calibration_table.csv')
    # TODO: Determine recommended sensor
    # Compare R² and RMSE - higher R², lower RMSE is better
    print('based on R^2 and RMSE above barometer appears to be the more accurate sensor but only ever so slightly. \n '
          'Even still I would recommend using the sonar because the drift in the barometer seems to be unreliable \n'
          'based on the calibration in Lab1-Phase1 height-comparison.png')
    
    recommended_sensor = "Sonar"  # TODO: Determine from analysis
    
    # TODO: Save results to JSON file
    # Include: recommended_sensor, sonar_calibration{scale, bias, r2, rmse},
    #          barometer_calibration{scale, bias, r2, rmse}

    json_output = {'recommended_sensor': recommended_sensor,
                   'sonar_calibration':{'scale':sonar_scale,
                                        'bias':sonar_bias,
                                        'r2': sonar_r2,
                                        'rmse':RMSE_sonar},
                   'baro_calibration':{'scale':baro_scale,
                                        'bias':baro_bias,
                                        'r2': baro_r2,
                                        'rmse':RMSE_baro},}

    with open('Lab1-Phase2/results.json','w+') as f:
        json.dump(json_output,f,indent=2)
    print(f"\n{'='*60}")
    print("PHASE 2 CALIBRATION RESULTS")
    print(f"{'='*60}")
    print(f"TODO: Print calibration results")
    print(f"Recommended sensor: {recommended_sensor}")
    
    print(f"\nTODO: Save calibration results to JSON file")

def main():
    print("Phase 2: Step Response Calibration")
    print("=" * 40)
    
    # Run the step calibration test
    #csv_filename = run_step_calibration()
    csv_filename = f'Lab1-Phase2/step_calibration_data.csv'
    
    if csv_filename:
        # Analyze the collected data
        analyze_step_data(csv_filename)
        print(f"\nPhase 2 complete! Files saved in {OUTPUT_DIR}/")
        print("Next: Run heuristic_control.py for Phase 3")
    else:
        print("Calibration test failed - no data to analyze")

if __name__ == "__main__":
    main()