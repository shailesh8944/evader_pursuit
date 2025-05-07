#!/usr/bin/env python3
import os
import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from model import DeadReckoningModel

def main():
    parser = argparse.ArgumentParser(description='Use a trained model for dead reckoning predictions')
    parser.add_argument('--test_file', type=str, required=True,
                        help='Path to CSV test data file')
    parser.add_argument('--model_path', type=str, default='dead_reckoning_model',
                        help='Path to the trained model directory')
    parser.add_argument('--window_size', type=int, default=10,
                        help='Number of time steps used as input context (must match training)')
    parser.add_argument('--prediction_steps', type=int, default=100,
                        help='Number of steps to predict ahead')
    parser.add_argument('--start_idx', type=int, default=0,
                        help='Starting index in the test data for prediction')
    parser.add_argument('--output_dir', type=str, default='predictions',
                        help='Directory to save prediction results')
    args = parser.parse_args()
    
    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)
    
    # Load test data
    test_data = pd.read_csv(args.test_file)
    print(f"Loaded test data with {len(test_data)} rows")
    
    # Create model instance with the same window size
    model = DeadReckoningModel(window_size=args.window_size, model_path=args.model_path)
    
    # Load trained model
    print(f"Loading model from {args.model_path}")
    model.load_model()
    
    # Extract initial sequence for prediction
    start_idx = args.start_idx
    end_idx = start_idx + args.window_size
    
    if end_idx >= len(test_data):
        print(f"Error: Not enough data for initial sequence (requested {args.window_size} rows starting at index {start_idx})")
        return
    
    initial_sequence = test_data.iloc[start_idx:end_idx]
    print(f"Using data from index {start_idx} to {end_idx-1} as initial sequence")
    
    # Get ground truth for comparison if available
    ground_truth = test_data.iloc[start_idx:end_idx + args.prediction_steps]
    
    # Predict trajectory
    print(f"Predicting {args.prediction_steps} steps ahead...")
    predictions = model.predict_trajectory(
        initial_sequence,
        num_steps=args.prediction_steps,
        ground_truth=ground_truth
    )
    
    # Save predictions to CSV
    output_file = os.path.join(args.output_dir, f"predictions_{os.path.basename(args.test_file)}")
    predictions.to_csv(output_file, index=False)
    print(f"Predictions saved to {output_file}")
    
    # Compare with ground truth
    if end_idx + args.prediction_steps <= len(test_data):
        # Calculate errors
        true_positions_x = ground_truth.iloc[args.window_size:]['position_x'].values
        true_positions_y = ground_truth.iloc[args.window_size:]['position_y'].values
        true_velocities_x = ground_truth.iloc[args.window_size:]['linear_vel_x'].values
        true_velocities_y = ground_truth.iloc[args.window_size:]['linear_vel_y'].values
        
        pred_positions_x = predictions['position_x'].values
        pred_positions_y = predictions['position_y'].values
        pred_velocities_x = predictions['velocity_x'].values
        pred_velocities_y = predictions['velocity_y'].values
        
        # Limit to actual prediction steps (in case ground truth is shorter)
        limit = min(len(true_positions_x), args.prediction_steps)
        
        true_positions_x = true_positions_x[:limit]
        true_positions_y = true_positions_y[:limit]
        true_velocities_x = true_velocities_x[:limit]
        true_velocities_y = true_velocities_y[:limit]
        
        pred_positions_x = pred_positions_x[:limit]
        pred_positions_y = pred_positions_y[:limit]
        pred_velocities_x = pred_velocities_x[:limit]
        pred_velocities_y = pred_velocities_y[:limit]
        
        # Calculate position error
        pos_errors = np.sqrt((true_positions_x - pred_positions_x)**2 + 
                             (true_positions_y - pred_positions_y)**2)
        
        # Calculate velocity error
        vel_errors = np.sqrt((true_velocities_x - pred_velocities_x)**2 + 
                             (true_velocities_y - pred_velocities_y)**2)
        
        # Calculate statistics
        avg_pos_error = np.mean(pos_errors)
        max_pos_error = np.max(pos_errors)
        avg_vel_error = np.mean(vel_errors)
        max_vel_error = np.max(vel_errors)
        
        print("\nPrediction Results:")
        print(f"Average position error: {avg_pos_error:.6f} m")
        print(f"Maximum position error: {max_pos_error:.6f} m")
        print(f"Average velocity error: {avg_vel_error:.6f} m/s")
        print(f"Maximum velocity error: {max_vel_error:.6f} m/s")
        
        # Plot position error over time
        plt.figure(figsize=(12, 10))
        
        # Position error plot
        plt.subplot(2, 1, 1)
        plt.plot(range(len(pos_errors)), pos_errors)
        plt.axhline(y=avg_pos_error, color='r', linestyle='--', label=f'Average Error: {avg_pos_error:.4f} m')
        plt.title('Position Error Over Time')
        plt.xlabel('Prediction Steps')
        plt.ylabel('Position Error (m)')
        plt.grid(True)
        plt.legend()
        
        # Velocity error plot
        plt.subplot(2, 1, 2)
        plt.plot(range(len(vel_errors)), vel_errors)
        plt.axhline(y=avg_vel_error, color='r', linestyle='--', label=f'Average Error: {avg_vel_error:.4f} m/s')
        plt.title('Velocity Error Over Time')
        plt.xlabel('Prediction Steps')
        plt.ylabel('Velocity Error (m/s)')
        plt.grid(True)
        plt.legend()
        
        plt.tight_layout()
        plt.savefig(os.path.join(args.output_dir, 'prediction_errors.png'))
        
        # Plot 2D trajectory comparison
        plt.figure(figsize=(10, 8))
        plt.plot(true_positions_x, true_positions_y, 'b-', label='Ground Truth')
        plt.plot(pred_positions_x, pred_positions_y, 'r-', label='Dead Reckoning Prediction')
        
        # Plot initial sequence
        initial_x = initial_sequence['position_x'].values
        initial_y = initial_sequence['position_y'].values
        plt.plot(initial_x, initial_y, 'g-', label='Initial Sequence')
        
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.title('Dead Reckoning Trajectory Prediction')
        plt.legend()
        plt.axis('equal')
        plt.grid(True)
        plt.savefig(os.path.join(args.output_dir, 'trajectory_comparison.png'))
        
        print(f"Plots saved to {args.output_dir}")

if __name__ == "__main__":
    main() 