#!/usr/bin/env python3
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import tensorflow as tf
from tensorflow.keras.models import load_model
import argparse
import joblib
from vessel_predictor import predict_trajectory, plot_predicted_vs_actual_trajectory

def load_test_data(data_file):
    """
    Load test data from a CSV file
    
    Args:
        data_file: Path to the CSV file containing test data
        
    Returns:
        df: DataFrame containing the test data
    """
    df = pd.read_csv(data_file)
    print(f"Loaded test data with {len(df)} samples")
    return df

def extract_trajectory_segment(df, start_idx, segment_length=None):
    """
    Extract a segment of trajectory data for testing
    
    Args:
        df: DataFrame containing trajectory data
        start_idx: Starting index for the segment
        segment_length: Length of the segment to extract (if None, use entire trajectory from start_idx)
        
    Returns:
        actual_trajectory: Actual trajectory points
        controls: Control inputs for the segment
        start_state: Initial state for prediction
    """
    # If segment_length is None, use the entire dataset from start_idx
    if segment_length is None:
        segment_length = len(df) - start_idx
    
    if start_idx + segment_length > len(df):
        print(f"Warning: Requested segment length ({segment_length}) exceeds available data. Adjusting.")
        segment_length = len(df) - start_idx
    
    # Extract the trajectory segment
    segment = df.iloc[start_idx:start_idx+segment_length]
    
    # Extract actual trajectory
    actual_trajectory = segment[['position_x', 'position_y']].values
    
    # Extract control inputs
    controls = segment[['rudder', 'propeller']].values
    
    # Extract IMU data for the start state - using only relevant axes
    imu_cols = ['linear_acc_x', 'linear_acc_y', 'angular_vel_z']
    imu_data = segment.iloc[0][imu_cols].values
    
    # Create start state: [x, y, imu_data]
    start_position = actual_trajectory[0]
    start_state = np.concatenate([start_position, imu_data])
    
    return actual_trajectory, controls, start_state, segment

def predict_trajectory(model, start_state, controls, segment_df, scaler):
    """
    Predict a vessel trajectory given initial state and control inputs by predicting position deltas.
    
    Args:
        model: Trained model (predicts delta_x, delta_y)
        start_state: Initial state vector [x, y, imu_data]
        controls: Control inputs for each step [[rudder, propeller], ...]
        segment_df: Original dataframe segment (for reference IMU data)
        scaler: Fitted scaler for the input features
        
    Returns:
        trajectory: Predicted trajectory points (absolute positions)
    """
    # Ensure controls array has the right shape
    num_steps = len(controls)
    
    # Initialize trajectory with the start position (absolute)
    trajectory = np.zeros((num_steps + 1, 2))
    trajectory[0] = start_state[:2]  # Initial absolute x, y
    
    # Current state (absolute position and IMU)
    current_position = start_state[:2].copy()
    current_imu = start_state[2:5].copy()  # IMU data (x,y accel, z angular velocity)
    
    # Predict future positions step by step
    for i in range(num_steps):
        # Use actual IMU data from the *next* time step in the segment for prediction
        # This matches the training setup where X(t) predicts Y(t) which is based on Position(t+1)
        # If we are at step i predicting step i+1, we need IMU data from step i+1
        if i + 1 < len(segment_df):
             current_imu = segment_df.iloc[i+1][['linear_acc_x', 'linear_acc_y', 'angular_vel_z']].values
        else:
             # If we run out of actual IMU data, reuse the last known value
             pass # current_imu remains the same

        # Prepare input for model using current absolute position and current controls
        model_input = np.concatenate([
            controls[i],            # rudder, propeller for step i
            current_position,       # absolute x, y at step i
            current_imu,            # IMU data relevant for predicting step i+1
        ]).reshape(1, -1)
        
        # Scale the input
        model_input_scaled = scaler.transform(model_input)
        
        # Predict the *change* in position (delta_x, delta_y)
        delta_position = model.predict(model_input_scaled, verbose=0)[0]
        
        # Update absolute position
        current_position = current_position + delta_position
        
        # Update trajectory with the new absolute position
        trajectory[i+1] = current_position
        
    return trajectory

def predict_and_evaluate(model, scaler, test_file, start_idx=0, segment_length=None, results_dir='trajectory_results'):
    """
    Predict a trajectory and compare with actual data
    
    Args:
        model: Trained model
        scaler: Fitted scaler for input features
        test_file: Path to test data file
        start_idx: Starting index for the trajectory segment
        segment_length: Length of the trajectory segment
        results_dir: Directory to save results
    """
    # Load test data
    df = load_test_data(test_file)
    
    # Extract trajectory segment
    actual_trajectory, controls, start_state, segment_df = extract_trajectory_segment(
        df, start_idx, segment_length)
    
    # Predict trajectory
    print(f"Predicting trajectory with {len(controls)} control inputs...")
    predicted_trajectory = predict_trajectory(model, start_state, controls, segment_df, scaler)
    
    # Calculate error metrics
    # Ensure we compare arrays of the same length (steps 1 to N-1)
    N = len(actual_trajectory)
    position_errors = np.sqrt(np.sum((predicted_trajectory[1:N] - actual_trajectory[1:N]) ** 2, axis=1))
    
    # Check if position_errors is empty (can happen if N <= 1)
    if position_errors.size > 0:
        mean_error = np.mean(position_errors)
        median_error = np.median(position_errors)
        max_error = np.max(position_errors)
    else:
        mean_error = median_error = max_error = 0.0 # Or handle as appropriate
        print("Warning: No position errors calculated (trajectory length <= 1).")
    
    print(f"\nTrajectory Prediction Metrics:")
    print(f"Mean Position Error: {mean_error:.6f} meters")
    print(f"Median Position Error: {median_error:.6f} meters")
    print(f"Maximum Position Error: {max_error:.6f} meters")
    print(f"Trajectory Length: {len(actual_trajectory)} points")
    
    # Create results directory
    os.makedirs(results_dir, exist_ok=True)
    
    # Save metrics to file
    with open(os.path.join(results_dir, 'trajectory_metrics.txt'), 'w') as f:
        f.write(f"Trajectory Prediction Metrics:\n")
        f.write(f"Mean Position Error: {mean_error:.6f} meters\n")
        f.write(f"Median Position Error: {median_error:.6f} meters\n")
        f.write(f"Maximum Position Error: {max_error:.6f} meters\n")
        f.write(f"Trajectory Length: {len(actual_trajectory)} points\n")
        f.write(f"Start Index: {start_idx}\n")
    
    # Plot the trajectory comparison
    plot_predicted_vs_actual_trajectory(actual_trajectory, predicted_trajectory, results_dir)
    
    # Plot prediction error over time
    plt.figure(figsize=(10, 6))
    plt.plot(np.arange(1, len(position_errors) + 1), position_errors)
    plt.xlabel('Time Step (starting from step 1)')
    plt.ylabel('Position Error (m)')
    plt.title('Prediction Error Over Time')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, 'prediction_error_over_time.png'), dpi=300)
    
    # Plot error histogram
    plt.figure(figsize=(10, 6))
    plt.hist(position_errors, bins=30)
    plt.xlabel('Error (m)')
    plt.ylabel('Frequency')
    plt.title('Distribution of Prediction Errors')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, 'error_histogram.png'), dpi=300)
    
    # Save the predicted and actual trajectories
    trajectory_df = pd.DataFrame({
        'time_step': np.arange(N),
        'actual_x': actual_trajectory[:, 0],
        'actual_y': actual_trajectory[:, 1],
        'predicted_x': predicted_trajectory[:N, 0],
        'predicted_y': predicted_trajectory[:N, 1],
        'error': np.append([0], position_errors)
    })
    trajectory_df.to_csv(os.path.join(results_dir, 'trajectory_comparison.csv'), index=False)
    
    return {
        'mean_error': mean_error,
        'median_error': median_error,
        'max_error': max_error,
        'trajectory_length': len(actual_trajectory)
    }

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Predict vessel trajectory using trained model')
    parser.add_argument('--model_dir', type=str, default='model',
                       help='Directory containing the trained model')
    parser.add_argument('--test_file', type=str, required=True,
                       help='CSV file containing test data')
    parser.add_argument('--start_idx', type=int, default=0,
                       help='Starting index for the trajectory segment')
    parser.add_argument('--segment_length', type=int, default=None,
                       help='Length of the trajectory segment (default: None, use entire dataset)')
    parser.add_argument('--results_dir', type=str, default='trajectory_results',
                       help='Directory to save results')
    args = parser.parse_args()
    
    # Load the model and scaler
    model_path = os.path.join(args.model_dir, 'best_model.h5')
    scaler_path = os.path.join(args.model_dir, 'scaler.pkl')
    
    if not os.path.exists(model_path) or not os.path.exists(scaler_path):
        print(f"Error: Model files not found in {args.model_dir}")
        return
    
    # Load model with custom objects for metrics
    custom_objects = {
        'MeanSquaredError': tf.keras.losses.MeanSquaredError,
        'MeanAbsoluteError': tf.keras.metrics.MeanAbsoluteError
    }
    try:
        model = load_model(model_path, custom_objects=custom_objects)
        print(f"Successfully loaded model from {model_path}")
    except Exception as e:
        print(f"Error loading model: {str(e)}")
        print("Trying alternate loading method...")
        
        # Alternative loading method if the first one fails
        try:
            model = tf.keras.models.load_model(model_path, compile=False)
            model.compile(
                optimizer='adam',
                loss=tf.keras.losses.MeanSquaredError(),
                metrics=[tf.keras.metrics.MeanAbsoluteError()]
            )
            print("Successfully loaded and recompiled model")
        except Exception as e:
            print(f"Failed to load model: {str(e)}")
            return
    
    scaler = joblib.load(scaler_path)
    
    # Predict and evaluate trajectory
    metrics = predict_and_evaluate(
        model, 
        scaler, 
        args.test_file,
        start_idx=args.start_idx,
        segment_length=args.segment_length,
        results_dir=args.results_dir
    )
    
    print("\nDone!")

if __name__ == "__main__":
    main() 