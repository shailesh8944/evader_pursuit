#!/usr/bin/env python3
import os
import numpy as np
import pandas as pd
import tensorflow as tf
import joblib
import matplotlib.pyplot as plt
from tensorflow.keras.models import load_model

# Define custom MSE function
def mean_squared_error(y_true, y_pred):
    return tf.reduce_mean(tf.square(y_pred - y_true), axis=-1)

class VesselPositionPredictor:
    def __init__(self, model_dir=None):
        """
        Initialize the predictor with a trained model
        
        Args:
            model_dir (str): Directory containing the model and scalers
        """
        if model_dir is None:
            model_dir = os.path.join(os.getcwd(), 'model_output')
        
        self.model_dir = model_dir
        self.model = None
        self.scaler_X_sensors = None
        self.scaler_X_position = None
        self.scaler_y = None
        self.sequence_length = 5  # Changed from 10 to 5 to match model's expected input shape
        
    def load_model_and_scalers(self):
        """Load the trained model and scalers"""
        # Load model
        model_path = os.path.join(self.model_dir, 'best_model.h5')
        
        try:
            # Try loading the model with custom_objects to handle the 'mse' loss function
            custom_objects = {
                'mse': mean_squared_error,
                'mean_squared_error': mean_squared_error
            }
            self.model = load_model(model_path, custom_objects=custom_objects)
            print("Model loaded successfully with custom_objects")
        except Exception as e:
            print(f"Error loading model: {str(e)}")
            print("Attempting to load the model with compile=False...")
            try:
                # Try loading without compilation
                self.model = load_model(model_path, compile=False)
                # Recompile the model with string-based loss
                self.model.compile(optimizer='adam', loss='mse')
                print("Model loaded without compilation and recompiled successfully.")
            except Exception as e2:
                print(f"Failed to load model even with compile=False: {str(e2)}")
                raise
        
        # Load scalers
        try:
            # Try to load the new split scalers
            scaler_X_sensors_path = os.path.join(self.model_dir, 'scaler_X_sensors.pkl')
            scaler_X_position_path = os.path.join(self.model_dir, 'scaler_X_position.pkl')
            scaler_y_path = os.path.join(self.model_dir, 'scaler_y.pkl')
            
            self.scaler_X_sensors = joblib.load(scaler_X_sensors_path)
            self.scaler_X_position = joblib.load(scaler_X_position_path)
            self.scaler_y = joblib.load(scaler_y_path)
            self.dual_input = True
            
        except FileNotFoundError:
            # Fall back to the old single scaler if needed
            print("Using legacy single-input model format")
            scaler_X_path = os.path.join(self.model_dir, 'scaler_X.pkl')
            scaler_y_path = os.path.join(self.model_dir, 'scaler_y.pkl')
            
            self.scaler_X_sensors = joblib.load(scaler_X_path)
            self.scaler_X_position = self.scaler_X_sensors  # Use the same scaler
            self.scaler_y = joblib.load(scaler_y_path)
            self.dual_input = False
        
        print(f"Model and scalers loaded from {self.model_dir}")
        
    def predict_trajectory(self, input_data, initial_position, steps=10):
        """
        Predict a trajectory by recursively feeding predictions back as input
        
        Args:
            input_data: DataFrame with IMU and control inputs
            initial_position: Initial [x, y] position
            steps: Number of prediction steps
            
        Returns:
            predicted_trajectory: Array of predicted positions
        """
        if self.model is None:
            self.load_model_and_scalers()
        
        # Extract required sensor features
        sensor_cols = [
            'linear_acc_x', 'linear_acc_y', 'linear_acc_z',
            'angular_vel_x', 'angular_vel_y', 'angular_vel_z',
            'rudder', 'propeller'
        ]
        
        # Check if all required columns are present
        if not all(col in input_data.columns for col in sensor_cols):
            raise ValueError(f"Input data must contain all required columns: {sensor_cols}")
        
        # Extract sensor features
        sensor_features = input_data[sensor_cols].values
        sensor_features_scaled = self.scaler_X_sensors.transform(sensor_features)
        
        # Initialize position history with the initial position
        position = np.array(initial_position).reshape(1, 2)
        position_scaled = self.scaler_X_position.transform(position)
        
        # Create arrays to store the trajectory
        predicted_trajectory = [initial_position]
        
        # Need at least sequence_length sensor readings to make a prediction
        if len(sensor_features_scaled) < self.sequence_length:
            raise ValueError(f"Not enough sensor readings. Need at least {self.sequence_length}, got {len(sensor_features_scaled)}")
        
        # Initial sensor sequence
        sensor_sequence = sensor_features_scaled[:self.sequence_length]
        
        # Initialize position sequence with the initial position repeated
        position_sequence = np.repeat(position_scaled, self.sequence_length, axis=0)
        
        # Make predictions step by step
        for i in range(steps):
            # Reshape sequences for model input
            sensor_input = sensor_sequence.reshape(1, self.sequence_length, -1)
            position_input = position_sequence.reshape(1, self.sequence_length, -1)
            
            # Make prediction
            try:
                if self.dual_input:
                    next_position_scaled = self.model.predict([sensor_input, position_input], verbose=0)
                else:
                    # For legacy models
                    next_position_scaled = self.model.predict(sensor_input, verbose=0)
            except Exception as e:
                print(f"Error during prediction: {str(e)}")
                print("Input shapes - Sensor:", sensor_input.shape, "Position:", position_input.shape)
                raise
            
            # Convert to actual position
            next_position = self.scaler_y.inverse_transform(next_position_scaled)[0]
            predicted_trajectory.append(next_position)
            
            # Update position sequence by removing oldest and adding new prediction
            position_scaled = self.scaler_X_position.transform(next_position.reshape(1, -1))
            position_sequence = np.vstack([position_sequence[1:], position_scaled])
            
            # If we have more sensor data, update the sensor sequence as well
            if i + self.sequence_length < len(sensor_features_scaled):
                sensor_sequence = sensor_features_scaled[i+1:i+self.sequence_length+1]
            
        return np.array(predicted_trajectory)
    
    def predict_from_csv(self, csv_file, output_file=None, steps=None):
        """
        Predict vessel positions from a CSV file containing IMU and control input data
        
        Args:
            csv_file (str): Path to CSV file containing input data
            output_file (str): Path to save predictions (optional)
            steps (int): Number of prediction steps (default: all available data)
            
        Returns:
            pandas.DataFrame: DataFrame with predicted positions
        """
        if self.model is None:
            self.load_model_and_scalers()
        
        # Load data
        df = pd.read_csv(csv_file)
        
        # Extract required sensor features
        sensor_cols = [
            'linear_acc_x', 'linear_acc_y', 'linear_acc_z',
            'angular_vel_x', 'angular_vel_y', 'angular_vel_z',
            'rudder', 'propeller'
        ]
        
        position_cols = ['position_x', 'position_y']
        
        # Check if all required columns are present
        if not all(col in df.columns for col in sensor_cols):
            raise ValueError(f"CSV file must contain all required sensor columns: {sensor_cols}")
        
        # Get timestamps
        timestamps = df['timestamp'].values
        
        # Limit steps if specified
        if steps is None:
            steps = len(df) - self.sequence_length
        else:
            steps = min(steps, len(df) - self.sequence_length)
        
        # Extract initial position
        if all(col in df.columns for col in position_cols):
            initial_position = df[position_cols].iloc[0].values
        else:
            initial_position = np.array([0.0, 0.0])  # Default to origin if no position data
            
        # Predict trajectory
        predicted_trajectory = self.predict_trajectory(df, initial_position, steps)
        
        # Create results DataFrame
        results = pd.DataFrame({
            'timestamp': timestamps[:steps+1],
            'predicted_x': predicted_trajectory[:, 0],
            'predicted_y': predicted_trajectory[:, 1]
        })
        
        # If actual positions are in the DataFrame, include them for comparison
        if all(col in df.columns for col in position_cols):
            results['actual_x'] = df['position_x'].values[:steps+1]
            results['actual_y'] = df['position_y'].values[:steps+1]
            
            # Calculate errors
            results['error_x'] = results['actual_x'] - results['predicted_x']
            results['error_y'] = results['actual_y'] - results['predicted_y']
            results['error_distance'] = np.sqrt(results['error_x']**2 + results['error_y']**2)
            
            # Print error statistics
            print(f"Average prediction error: {results['error_distance'].mean():.4f} meters")
            print(f"Max prediction error: {results['error_distance'].max():.4f} meters")
        
        # Save to CSV if output file is specified
        if output_file:
            results.to_csv(output_file, index=False)
            print(f"Predictions saved to {output_file}")
            
            # If actual positions are available, plot predictions vs actual
            if 'actual_x' in results.columns:
                self._plot_predictions(results, os.path.splitext(output_file)[0] + '.png')
        
        return results
    
    def _plot_predictions(self, results, output_file=None):
        """Plot predicted vs actual positions"""
        plt.figure(figsize=(12, 10))
        
        # Plot full trajectories
        plt.subplot(2, 1, 1)
        plt.plot(results['actual_x'], results['actual_y'], 'b-', label='Actual Path')
        plt.plot(results['predicted_x'], results['predicted_y'], 'r--', label='Predicted Path')
        
        # Add markers for start and end points
        plt.scatter(results['actual_x'].iloc[0], results['actual_y'].iloc[0], 
                   c='green', s=100, marker='o', label='Start')
        plt.scatter(results['actual_x'].iloc[-1], results['actual_y'].iloc[-1], 
                   c='blue', s=100, marker='s', label='Actual End')
        plt.scatter(results['predicted_x'].iloc[-1], results['predicted_y'].iloc[-1], 
                   c='red', s=100, marker='s', label='Predicted End')
        
        # Add arrows to show direction of movement
        step = max(1, len(results) // 20)  # Plot max 20 arrows
        for i in range(0, len(results) - step, step):
            # Actual path arrows
            plt.arrow(
                results['actual_x'].iloc[i], results['actual_y'].iloc[i],
                results['actual_x'].iloc[i+step] - results['actual_x'].iloc[i],
                results['actual_y'].iloc[i+step] - results['actual_y'].iloc[i],
                color='blue', head_width=0.2, head_length=0.3, length_includes_head=True, alpha=0.5
            )
            
            # Predicted path arrows
            plt.arrow(
                results['predicted_x'].iloc[i], results['predicted_y'].iloc[i],
                results['predicted_x'].iloc[i+step] - results['predicted_x'].iloc[i],
                results['predicted_y'].iloc[i+step] - results['predicted_y'].iloc[i],
                color='red', head_width=0.2, head_length=0.3, length_includes_head=True, alpha=0.5
            )
        
        # Add axis labels and legend
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.title('Actual vs Predicted Trajectory')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        
        # Plot error over time
        plt.subplot(2, 1, 2)
        plt.plot(results['timestamp'], results['error_distance'], 'k-')
        plt.axhline(y=results['error_distance'].mean(), color='r', linestyle='--', 
                   label=f'Mean Error: {results["error_distance"].mean():.2f}m')
        plt.fill_between(results['timestamp'], 
                        results['error_distance'].mean() - results['error_distance'].std(),
                        results['error_distance'].mean() + results['error_distance'].std(),
                        alpha=0.2, color='r', label=f'±1 Std: {results["error_distance"].std():.2f}m')
        plt.xlabel('Timestamp')
        plt.ylabel('Error Distance (m)')
        plt.title('Prediction Error Over Time')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        
        # Save figure
        if output_file:
            plt.savefig(output_file, dpi=300)
            print(f"Plot saved to {output_file}")
        else:
            plt.show()

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Predict vessel positions from IMU and control input data')
    parser.add_argument('--model_dir', type=str, default=None, help='Directory containing the trained model')
    parser.add_argument('--input_file', type=str, required=True, help='Input CSV file with IMU and control data')
    parser.add_argument('--output_file', type=str, default=None, help='Output CSV file for predictions')
    parser.add_argument('--steps', type=int, default=None, help='Number of prediction steps')
    
    args = parser.parse_args()
    
    # Create predictor
    predictor = VesselPositionPredictor(model_dir=args.model_dir)
    
    # Make predictions
    predictor.predict_from_csv(args.input_file, args.output_file, steps=args.steps) 