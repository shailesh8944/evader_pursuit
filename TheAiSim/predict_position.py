#!/usr/bin/env python3
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import tensorflow as tf
from tensorflow.keras.models import load_model
from tensorflow.keras.losses import MeanSquaredError
import argparse
import pickle
from sklearn.metrics import mean_squared_error
import datetime

def load_model_and_scalers(model_dir):
    """
    Load the trained model and scalers
    
    Args:
        model_dir: Directory containing the model and scalers
        
    Returns:
        model, x_scaler, y_scaler
    """
    # Load the model - with custom_objects to handle mse error
    model_path = os.path.join(model_dir, 'best_model.h5')
    if not os.path.exists(model_path):
        model_path = os.path.join(model_dir, 'final_model.h5')
    
    # Define custom objects for model loading
    custom_objects = {
        'mse': MeanSquaredError(),
        'mean_squared_error': MeanSquaredError()
    }
    
    model = load_model(model_path, custom_objects=custom_objects)
    
    # Load the scalers
    with open(os.path.join(model_dir, 'x_scaler.pkl'), 'rb') as f:
        x_scaler = pickle.load(f)
    
    with open(os.path.join(model_dir, 'y_scaler.pkl'), 'rb') as f:
        y_scaler = pickle.load(f)
    
    return model, x_scaler, y_scaler

def predict_trajectory(model, x_scaler, y_scaler, initial_state, rudder_angles, time_steps=100):
    """
    Predict trajectory using the trained model
    
    Args:
        model: Trained model
        x_scaler: Scaler for input features
        y_scaler: Scaler for output features
        initial_state: Initial state (pos_x, pos_y, vel_x, vel_y, accel_x, accel_y, ang_vel_z)
        rudder_angles: List of rudder angles to use for prediction
        time_steps: Number of time steps to predict
        
    Returns:
        Predicted trajectory
    """
    # Initialize state
    state = initial_state.copy()
    
    # Initialize trajectory storage
    trajectory = np.zeros((time_steps, 4))  # pos_x, pos_y, vel_x, vel_y
    
    # Predict trajectory step by step
    for i in range(time_steps):
        # Get current rudder angle
        if i < len(rudder_angles):
            rudder_angle = rudder_angles[i]
        else:
            rudder_angle = rudder_angles[-1]  # Use last rudder angle if we run out
        
        # Create input feature vector (without yaw)
        features = np.array([
            state['pos_x'], 
            state['pos_y'], 
            state['vel_x'], 
            state['vel_y'], 
            state['accel_x'], 
            state['accel_y'], 
            state['ang_vel_z'], 
            rudder_angle
        ]).reshape(1, -1)
        
        # Scale features
        scaled_features = x_scaler.transform(features)
        
        # Make prediction
        scaled_prediction = model.predict(scaled_features, verbose=0)
        
        # Inverse transform prediction
        prediction = y_scaler.inverse_transform(scaled_prediction)[0]
        
        # Store prediction
        trajectory[i] = prediction
        
        # Update state for next iteration
        state['pos_x'] = prediction[0]
        state['pos_y'] = prediction[1]
        state['vel_x'] = prediction[2]
        state['vel_y'] = prediction[3]
        
    return trajectory

def evaluate_on_test_data(model, x_scaler, y_scaler, test_data_file, output_dir):
    """
    Evaluate the model on test data
    
    Args:
        model: Trained model
        x_scaler: Scaler for input features
        y_scaler: Scaler for output features
        test_data_file: Path to test data file
        output_dir: Directory to save predictions and plots
        
    Returns:
        MSE, RMSE for each output variable
    """
    # Load test data
    df = pd.read_csv(test_data_file)
    
    # Extract features and targets - without yaw
    features = ['pos_x', 'pos_y', 'vel_x', 'vel_y', 'accel_x', 'accel_y', 'ang_vel_z', 'rudder_angle']
    targets = ['pos_x', 'pos_y', 'vel_x', 'vel_y']
    
    # Check if required columns exist
    if not all(col in df.columns for col in features):
        missing_cols = [col for col in features if col not in df.columns]
        raise ValueError(f"Missing columns {missing_cols} in test data file")
    
    X_test = df[features].values[:-1]  # All but the last row
    y_test = df[targets].values[1:]     # All but the first row
    
    # Original timestamps (excluding the first one since we're predicting from index 1)
    timestamps = df['timestamp'].values[1:]
    
    # Scale features
    X_test_scaled = x_scaler.transform(X_test)
    
    # Make predictions
    y_pred_scaled = model.predict(X_test_scaled)
    y_pred = y_scaler.inverse_transform(y_pred_scaled)
    
    # Calculate errors
    mse = mean_squared_error(y_test, y_pred, multioutput='raw_values')
    rmse = np.sqrt(mse)
    
    print("Test MSE:")
    print(f"Position X: {mse[0]:.6f}")
    print(f"Position Y: {mse[1]:.6f}")
    print(f"Velocity X: {mse[2]:.6f}")
    print(f"Velocity Y: {mse[3]:.6f}")
    print("\nTest RMSE:")
    print(f"Position X: {rmse[0]:.6f}")
    print(f"Position Y: {rmse[1]:.6f}")
    print(f"Velocity X: {rmse[2]:.6f}")
    print(f"Velocity Y: {rmse[3]:.6f}")
    
    # Save predictions to CSV
    predictions_df = pd.DataFrame({
        'timestamp': timestamps,
        'true_pos_x': y_test[:, 0],
        'true_pos_y': y_test[:, 1],
        'true_vel_x': y_test[:, 2],
        'true_vel_y': y_test[:, 3],
        'pred_pos_x': y_pred[:, 0],
        'pred_pos_y': y_pred[:, 1],
        'pred_vel_x': y_pred[:, 2],
        'pred_vel_y': y_pred[:, 3],
        'error_pos_x': np.abs(y_test[:, 0] - y_pred[:, 0]),
        'error_pos_y': np.abs(y_test[:, 1] - y_pred[:, 1]),
        'error_vel_x': np.abs(y_test[:, 2] - y_pred[:, 2]),
        'error_vel_y': np.abs(y_test[:, 3] - y_pred[:, 3])
    })
    
    predictions_df.to_csv(os.path.join(output_dir, 'predictions.csv'), index=False)
    print(f"Predictions saved to {os.path.join(output_dir, 'predictions.csv')}")
    
    # Save metrics to a file
    metrics_df = pd.DataFrame({
        'metric': ['MSE', 'RMSE'],
        'pos_x': [mse[0], rmse[0]],
        'pos_y': [mse[1], rmse[1]],
        'vel_x': [mse[2], rmse[2]],
        'vel_y': [mse[3], rmse[3]]
    })
    
    metrics_df.to_csv(os.path.join(output_dir, 'metrics.csv'), index=False)
    print(f"Metrics saved to {os.path.join(output_dir, 'metrics.csv')}")
    
    # Plot a comparison of predicted vs actual trajectory
    plt.figure(figsize=(12, 10))
    
    # Full trajectory
    plt.subplot(2, 1, 1)
    plt.plot(y_test[:, 0], y_test[:, 1], 'b-', label='Actual Trajectory')
    plt.plot(y_pred[:, 0], y_pred[:, 1], 'r--', label='Predicted Trajectory')
    plt.scatter(y_test[0, 0], y_test[0, 1], c='g', s=100, label='Start')
    plt.scatter(y_test[-1, 0], y_test[-1, 1], c='k', s=100, label='End')
    plt.xlabel('Position X')
    plt.ylabel('Position Y')
    plt.title('Full Trajectory Comparison')
    plt.legend()
    plt.grid(True)
    
    # Zoom to a section
    segment_length = min(300, len(y_test))
    start_idx = np.random.randint(0, len(y_test) - segment_length)
    
    plt.subplot(2, 1, 2)
    plt.plot(y_test[start_idx:start_idx+segment_length, 0], 
             y_test[start_idx:start_idx+segment_length, 1], 
             'b-', label='Actual Trajectory')
    plt.plot(y_pred[start_idx:start_idx+segment_length, 0], 
             y_pred[start_idx:start_idx+segment_length, 1], 
             'r--', label='Predicted Trajectory')
    plt.scatter(y_test[start_idx, 0], y_test[start_idx, 1], c='g', s=100, label='Start')
    plt.scatter(y_test[start_idx+segment_length-1, 0], y_test[start_idx+segment_length-1, 1], c='k', s=100, label='End')
    plt.xlabel('Position X')
    plt.ylabel('Position Y')
    plt.title(f'Trajectory Segment Comparison (steps {start_idx} to {start_idx+segment_length-1})')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'trajectory_comparison.png'))
    
    # Plot velocities
    plt.figure(figsize=(12, 8))
    
    # Velocity X
    plt.subplot(2, 1, 1)
    plt.plot(y_test[start_idx:start_idx+segment_length, 2], 'b-', label='Actual Velocity X')
    plt.plot(y_pred[start_idx:start_idx+segment_length, 2], 'r--', label='Predicted Velocity X')
    plt.xlabel('Time Step')
    plt.ylabel('Velocity X')
    plt.title('Velocity X Comparison')
    plt.legend()
    plt.grid(True)
    
    # Velocity Y
    plt.subplot(2, 1, 2)
    plt.plot(y_test[start_idx:start_idx+segment_length, 3], 'b-', label='Actual Velocity Y')
    plt.plot(y_pred[start_idx:start_idx+segment_length, 3], 'r--', label='Predicted Velocity Y')
    plt.xlabel('Time Step')
    plt.ylabel('Velocity Y')
    plt.title('Velocity Y Comparison')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'velocity_comparison.png'))
    
    # Plot position errors over time
    plt.figure(figsize=(12, 8))
    
    # Position X Error
    plt.subplot(2, 1, 1)
    plt.plot(np.abs(y_test[:, 0] - y_pred[:, 0]), 'r-')
    plt.axhline(y=rmse[0], color='k', linestyle='--', label=f'RMSE: {rmse[0]:.4f}')
    plt.xlabel('Time Step')
    plt.ylabel('Error (m)')
    plt.title('Position X Error')
    plt.legend()
    plt.grid(True)
    
    # Position Y Error
    plt.subplot(2, 1, 2)
    plt.plot(np.abs(y_test[:, 1] - y_pred[:, 1]), 'r-')
    plt.axhline(y=rmse[1], color='k', linestyle='--', label=f'RMSE: {rmse[1]:.4f}')
    plt.xlabel('Time Step')
    plt.ylabel('Error (m)')
    plt.title('Position Y Error')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'position_errors.png'))
    
    # Save information about the test run
    with open(os.path.join(output_dir, 'test_info.txt'), 'w') as f:
        f.write(f"Test data file: {test_data_file}\n")
        f.write(f"Number of samples: {len(y_test)}\n")
        f.write(f"Test MSE:\n")
        f.write(f"Position X: {mse[0]:.6f}\n")
        f.write(f"Position Y: {mse[1]:.6f}\n")
        f.write(f"Velocity X: {mse[2]:.6f}\n")
        f.write(f"Velocity Y: {mse[3]:.6f}\n")
        f.write(f"\nTest RMSE:\n")
        f.write(f"Position X: {rmse[0]:.6f}\n")
        f.write(f"Position Y: {rmse[1]:.6f}\n")
        f.write(f"Velocity X: {rmse[2]:.6f}\n")
        f.write(f"Velocity Y: {rmse[3]:.6f}\n")
    
    return mse, rmse

def predict_with_custom_inputs(model, x_scaler, y_scaler, time_steps=100):
    """
    Predict trajectory with custom input scenarios
    
    Args:
        model: Trained model
        x_scaler: Scaler for input features
        y_scaler: Scaler for output features
        time_steps: Number of time steps to predict
    """
    # Define different rudder angle scenarios
    scenarios = {
        'straight': np.zeros(time_steps),
        'turn_right': np.ones(time_steps) * 10.0,  # 10 degrees right
        'turn_left': np.ones(time_steps) * -10.0,  # 10 degrees left
        'zigzag': np.concatenate([
            np.ones(20) * 15.0,   # Right turn
            np.ones(20) * -15.0,  # Left turn
            np.ones(20) * 15.0,   # Right turn
            np.ones(20) * -15.0,  # Left turn
            np.ones(20) * 0.0     # Straight
        ])
    }
    
    # Initial state (approximately average values from data) - without yaw
    initial_state = {
        'pos_x': 0.0,
        'pos_y': 0.0,
        'vel_x': 0.5,  # Forward velocity
        'vel_y': 0.0,  # No sideways velocity initially
        'accel_x': 0.0,
        'accel_y': 0.0,
        'ang_vel_z': 0.0
    }
    
    plt.figure(figsize=(12, 10))
    
    for scenario_name, rudder_angles in scenarios.items():
        # Predict trajectory
        trajectory = predict_trajectory(
            model, x_scaler, y_scaler, 
            initial_state, rudder_angles, 
            time_steps=min(time_steps, len(rudder_angles))
        )
        
        # Plot trajectory
        plt.plot(trajectory[:, 0], trajectory[:, 1], label=f'Scenario: {scenario_name}')
    
    plt.scatter(initial_state['pos_x'], initial_state['pos_y'], c='g', s=100, label='Start')
    plt.xlabel('Position X')
    plt.ylabel('Position Y')
    plt.title('Predicted Trajectories for Different Rudder Angles')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')  # Equal aspect ratio
    plt.savefig('custom_scenarios.png')

def main():
    parser = argparse.ArgumentParser(description='Make predictions using the trained vessel model')
    parser.add_argument('--model_dir', type=str, default='model',
                        help='Directory containing the trained model and scalers')
    parser.add_argument('--test_data', type=str, default='extracted_data/run1/synchronized.csv',
                        help='Path to test data file')
    parser.add_argument('--output_dir', type=str, default=None,
                        help='Directory to save predictions and plots (default: auto-generated)')
    
    args = parser.parse_args()
    
    # Create output directory if not specified
    if args.output_dir is None:
        # Use test data filename and timestamp to create a unique folder
        test_data_name = os.path.basename(os.path.dirname(args.test_data))
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output_dir = f"predictions_{test_data_name}_{timestamp}"
    
    # Create the output directory
    os.makedirs(args.output_dir, exist_ok=True)
    print(f"Saving predictions to: {args.output_dir}")
    
    # Load model and scalers
    model, x_scaler, y_scaler = load_model_and_scalers(args.model_dir)
    print(f"Model loaded from {args.model_dir}")
    
    # Evaluate on test data
    if os.path.exists(args.test_data):
        print(f"Evaluating on test data: {args.test_data}")
        evaluate_on_test_data(model, x_scaler, y_scaler, args.test_data, args.output_dir)
    else:
        print(f"Warning: Test data file not found: {args.test_data}")

if __name__ == '__main__':
    main() 