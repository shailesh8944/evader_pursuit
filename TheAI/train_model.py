#!/usr/bin/env python3
import os
import glob
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler, MinMaxScaler
import tensorflow as tf
from tensorflow.keras.models import Sequential, Model
from tensorflow.keras.layers import Dense, LSTM, Bidirectional, Input, Dropout, concatenate
from tensorflow.keras.callbacks import EarlyStopping, ModelCheckpoint, ReduceLROnPlateau
from tensorflow.keras.optimizers import Adam
import joblib
import datetime
import argparse

class PositionPredictor:
    def __init__(self, data_dir, sequence_length=10, test_size=0.2, random_state=42, use_smoothed_data=True, position_delay=1):
        """
        Initialize the model trainer
        
        Args:
            data_dir (str): Directory containing the combined CSV files
            sequence_length (int): Length of input sequences (time steps)
            test_size (float): Proportion of data to use for testing
            random_state (int): Random seed for reproducibility
            use_smoothed_data (bool): Whether to use smoothed data for training
            position_delay (int): How many steps in the future to predict (default 1)
        """
        self.data_dir = data_dir
        self.sequence_length = sequence_length
        self.test_size = test_size
        self.random_state = random_state
        self.use_smoothed_data = use_smoothed_data
        self.position_delay = position_delay
        self.model = None
        self.scaler_X_sensors = StandardScaler()  # For IMU and control inputs
        self.scaler_X_position = StandardScaler()  # For position inputs
        self.scaler_y = StandardScaler()          # For position outputs
        
        # Create output directory for model artifacts
        self.output_dir = os.path.join(os.path.dirname(data_dir), 'model_output')
        os.makedirs(self.output_dir, exist_ok=True)
        
    def load_data(self):
        """Load and preprocess data from combined CSV files"""
        print("Loading data...")
        
        # Find all combined CSV files (regular or smoothed based on parameter)
        file_pattern = "*_smoothed_combined.csv" if self.use_smoothed_data else "*_combined.csv"
        csv_files = glob.glob(os.path.join(self.data_dir, file_pattern))
        
        if not csv_files:
            if self.use_smoothed_data:
                print("No smoothed data files found. Falling back to regular combined data.")
                file_pattern = "*_combined.csv"
                csv_files = glob.glob(os.path.join(self.data_dir, file_pattern))
            
            if not csv_files:
                raise ValueError(f"No combined CSV files found in {self.data_dir}")
        
        # Load all data into a single DataFrame
        all_data = []
        for file in csv_files:
            print(f"Reading {file}")
            df = pd.read_csv(file)
            all_data.append(df)
        
        # Concatenate all data
        data = pd.concat(all_data, ignore_index=True)
        
        # Sort by timestamp to ensure temporal ordering
        data = data.sort_values('timestamp')
        
        # Save first 100 rows for visualization
        data_sample = data.head(100)
        data_sample.to_csv(os.path.join(self.output_dir, 'data_sample.csv'), index=False)
        
        # Extract features and targets
        sensor_cols = [
            'linear_acc_x', 'linear_acc_y', 'linear_acc_z',
            'angular_vel_x', 'angular_vel_y', 'angular_vel_z',
            'rudder', 'propeller'
        ]
        
        position_cols = ['position_x', 'position_y']
        
        # Visualize trajectories
        self._plot_trajectories(data)
        
        # Create feature arrays
        sensor_features = data[sensor_cols].values
        position_features = data[position_cols].values
        targets = data[position_cols].values
        
        # Scale the data
        sensor_features_scaled = self.scaler_X_sensors.fit_transform(sensor_features)
        position_features_scaled = self.scaler_X_position.fit_transform(position_features)
        targets_scaled = self.scaler_y.fit_transform(targets)
        
        # Save the scalers
        joblib.dump(self.scaler_X_sensors, os.path.join(self.output_dir, 'scaler_X_sensors.pkl'))
        joblib.dump(self.scaler_X_position, os.path.join(self.output_dir, 'scaler_X_position.pkl'))
        joblib.dump(self.scaler_y, os.path.join(self.output_dir, 'scaler_y.pkl'))
        
        # Create sequences with position history included
        X_sensor_sequences, X_position_sequences, y_sequences = self._create_sequences_with_position(
            sensor_features_scaled, position_features_scaled, targets_scaled
        )
        
        # Split into train and test sets
        X_sensor_train, X_sensor_test, X_position_train, X_position_test, y_train, y_test = train_test_split(
            X_sensor_sequences, X_position_sequences, y_sequences, 
            test_size=self.test_size, random_state=self.random_state
        )
        
        print(f"Training data shape - Sensors: {X_sensor_train.shape}, Position: {X_position_train.shape}, Targets: {y_train.shape}")
        print(f"Testing data shape - Sensors: {X_sensor_test.shape}, Position: {X_position_test.shape}, Targets: {y_test.shape}")
        
        return (X_sensor_train, X_position_train, X_sensor_test, X_position_test, y_train, y_test)
    
    def _create_sequences_with_position(self, sensor_features, position_features, targets):
        """
        Create sequences for LSTM input, including position history
        
        This creates three arrays:
        1. X_sensor_sequences: Sequences of sensor data [batch, sequence_length, n_sensor_features]
        2. X_position_sequences: Sequences of position data [batch, sequence_length, n_position_features]
        3. y_sequences: Target positions [batch, n_position_features]
        """
        X_sensor_seq, X_position_seq, y_seq = [], [], []
        
        # We need at least sequence_length + position_delay points to create a valid sequence and target
        for i in range(len(sensor_features) - self.sequence_length - self.position_delay + 1):
            # Create sequence of sensor features
            X_sensor_seq.append(sensor_features[i:i + self.sequence_length])
            # Create sequence of position features (same time range as sensor features)
            X_position_seq.append(position_features[i:i + self.sequence_length])
            # Target is the position after the sequence
            y_seq.append(targets[i + self.sequence_length + self.position_delay - 1])
        
        return np.array(X_sensor_seq), np.array(X_position_seq), np.array(y_seq)
    
    def _plot_trajectories(self, data, max_points=5000):
        """Plot the trajectories from the data"""
        
        # Create a figure
        plt.figure(figsize=(12, 8))
        
        # If data has more than max_points, sample to reduce plotting time
        if len(data) > max_points:
            plot_data = data.sample(max_points, random_state=self.random_state)
        else:
            plot_data = data
        
        # Plot the trajectory
        plt.scatter(plot_data['position_x'], plot_data['position_y'], c=range(len(plot_data)), 
                   cmap='viridis', alpha=0.7, s=5)
        
        # Connect points with lines to show the path
        plt.plot(plot_data['position_x'], plot_data['position_y'], 'k-', linewidth=0.5, alpha=0.3)
        
        # Add colorbar to show temporal progression
        cbar = plt.colorbar()
        cbar.set_label('Temporal Progression')
        
        # Add axis labels and title
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        
        data_type = "Smoothed" if self.use_smoothed_data else "Raw"
        plt.title(f'{data_type} Trajectory Data')
        
        # Keep aspect ratio equal
        plt.axis('equal')
        
        # Add grid
        plt.grid(True, alpha=0.3)
        
        # Save the figure
        plt.savefig(os.path.join(self.output_dir, f'{data_type.lower()}_trajectory.png'), dpi=300)
        plt.close()
    
    def build_model(self):
        """Build two-input Bi-LSTM model architecture with position history"""
        print("Building model with position history...")
        
        # Input shapes
        sensor_input_shape = (self.sequence_length, self.scaler_X_sensors.n_features_in_)
        position_input_shape = (self.sequence_length, self.scaler_X_position.n_features_in_)
        
        # Create a more complex model with multiple inputs
        # Sensor input branch (IMU and control inputs)
        sensor_input = Input(shape=sensor_input_shape, name='sensor_input')
        sensor_branch = Bidirectional(LSTM(64, return_sequences=True))(sensor_input)
        sensor_branch = Dropout(0.2)(sensor_branch)
        sensor_branch = Bidirectional(LSTM(32))(sensor_branch)
        sensor_branch = Dropout(0.2)(sensor_branch)
        
        # Position history branch
        position_input = Input(shape=position_input_shape, name='position_input')
        position_branch = Bidirectional(LSTM(32, return_sequences=True))(position_input)
        position_branch = Dropout(0.2)(position_branch)
        position_branch = Bidirectional(LSTM(16))(position_branch)
        position_branch = Dropout(0.2)(position_branch)
        
        # Combine the branches
        combined = concatenate([sensor_branch, position_branch])
        combined = Dense(32, activation='relu')(combined)
        combined = Dropout(0.2)(combined)
        combined = Dense(16, activation='relu')(combined)
        
        # Output layer (position_x, position_y)
        output = Dense(2, name='position_output')(combined)
        
        # Create model
        model = Model(inputs=[sensor_input, position_input], outputs=output)
        
        # Compile the model
        model.compile(
            optimizer=Adam(learning_rate=0.001),
            loss='mse'
        )
        
        model.summary()
        
        # Save model summary to a file
        with open(os.path.join(self.output_dir, 'model_summary.txt'), 'w') as f:
            # Redirect summary to file
            model.summary(print_fn=lambda x: f.write(x + '\n'))
        
        self.model = model
        return model
    
    def train_model(self, X_sensor_train, X_position_train, X_sensor_test, X_position_test, y_train, y_test, epochs=100, batch_size=32):
        """Train the model with sensor and position history inputs"""
        print("Training model...")
        
        # Create callbacks
        callbacks = [
            EarlyStopping(monitor='val_loss', patience=15, restore_best_weights=True),
            ReduceLROnPlateau(monitor='val_loss', factor=0.5, patience=5, min_lr=1e-6),
            ModelCheckpoint(
                os.path.join(self.output_dir, 'best_model.h5'),
                monitor='val_loss',
                save_best_only=True
            )
        ]
        
        # Train the model with multiple inputs
        history = self.model.fit(
            [X_sensor_train, X_position_train], y_train,
            epochs=epochs,
            batch_size=batch_size,
            validation_data=([X_sensor_test, X_position_test], y_test),
            callbacks=callbacks,
            verbose=1
        )
        
        # Save the final model
        self.model.save(os.path.join(self.output_dir, 'final_model.h5'))
        
        # Plot training history
        self._plot_history(history)
        
        return history
    
    def evaluate_model(self, X_sensor_test, X_position_test, y_test):
        """Evaluate the model on test data"""
        print("Evaluating model...")
        
        # Evaluate the model
        mse = self.model.evaluate([X_sensor_test, X_position_test], y_test)
        print(f"Test MSE: {mse}")
        
        # Make predictions
        y_pred_scaled = self.model.predict([X_sensor_test, X_position_test])
        
        # Inverse transform predictions and actual values
        y_pred = self.scaler_y.inverse_transform(y_pred_scaled)
        y_actual = self.scaler_y.inverse_transform(y_test)
        
        # Calculate errors
        rmse = np.sqrt(np.mean((y_pred - y_actual) ** 2, axis=0))
        mae = np.mean(np.abs(y_pred - y_actual), axis=0)
        
        print(f"RMSE X: {rmse[0]:.4f}, Y: {rmse[1]:.4f}")
        print(f"MAE X: {mae[0]:.4f}, Y: {mae[1]:.4f}")
        
        # Plot predictions
        self._plot_predictions(y_actual, y_pred)
        
        # Also plot predicted vs actual full trajectories for a better visualization
        self._plot_trajectory_comparison(X_position_test, y_actual, y_pred)
        
        # Save evaluation metrics to a file
        with open(os.path.join(self.output_dir, 'evaluation_metrics.txt'), 'w') as f:
            f.write(f"Test MSE: {mse}\n")
            f.write(f"RMSE X: {rmse[0]:.4f}, Y: {rmse[1]:.4f}\n")
            f.write(f"MAE X: {mae[0]:.4f}, Y: {mae[1]:.4f}\n")
        
        return rmse, mae
    
    def _plot_history(self, history):
        """Plot training history"""
        plt.figure(figsize=(10, 6))
        plt.plot(history.history['loss'], label='Training Loss')
        plt.plot(history.history['val_loss'], label='Validation Loss')
        plt.title('Model Training History')
        plt.xlabel('Epoch')
        plt.ylabel('Loss (MSE)')
        plt.legend()
        plt.grid(True)
        plt.savefig(os.path.join(self.output_dir, 'training_history.png'))
        plt.close()
    
    def _plot_predictions(self, y_actual, y_pred, num_samples=100):
        """Plot predicted vs actual positions"""
        # Choose a random subset of data points to visualize
        indices = np.random.choice(len(y_actual), min(num_samples, len(y_actual)), replace=False)
        
        # Plot X position
        plt.figure(figsize=(12, 6))
        plt.subplot(1, 2, 1)
        plt.scatter(y_actual[indices, 0], y_pred[indices, 0], alpha=0.5)
        plt.plot([y_actual[:, 0].min(), y_actual[:, 0].max()], 
                 [y_actual[:, 0].min(), y_actual[:, 0].max()], 
                 'r--')
        plt.xlabel('Actual X Position')
        plt.ylabel('Predicted X Position')
        plt.title('X Position Prediction')
        plt.grid(True)
        
        # Plot Y position
        plt.subplot(1, 2, 2)
        plt.scatter(y_actual[indices, 1], y_pred[indices, 1], alpha=0.5)
        plt.plot([y_actual[:, 1].min(), y_actual[:, 1].max()], 
                 [y_actual[:, 1].min(), y_actual[:, 1].max()], 
                 'r--')
        plt.xlabel('Actual Y Position')
        plt.ylabel('Predicted Y Position')
        plt.title('Y Position Prediction')
        plt.grid(True)
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, 'prediction_vs_actual.png'))
        plt.close()
        
        # Plot trajectory
        plt.figure(figsize=(10, 8))
        plt.plot(y_actual[indices, 0], y_actual[indices, 1], 'b-', label='Actual Path')
        plt.plot(y_pred[indices, 0], y_pred[indices, 1], 'r--', label='Predicted Path')
        plt.scatter(y_actual[indices, 0], y_actual[indices, 1], c='blue', s=10, alpha=0.5)
        plt.scatter(y_pred[indices, 0], y_pred[indices, 1], c='red', s=10, alpha=0.5)
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.title('Actual vs Predicted Trajectory (Sample Points)')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.savefig(os.path.join(self.output_dir, 'trajectory_comparison_samples.png'))
        plt.close()
    
    def _plot_trajectory_comparison(self, X_position_test, y_actual, y_pred, max_sequences=5):
        """Plot complete trajectory predictions compared to actual trajectories"""
        # Choose a subset of test sequences
        sequence_indices = np.random.choice(len(X_position_test), min(max_sequences, len(X_position_test)), replace=False)
        
        plt.figure(figsize=(15, 10))
        
        for i, idx in enumerate(sequence_indices):
            # Get the last position from the input sequence
            last_position = self.scaler_X_position.inverse_transform(X_position_test[idx, -1].reshape(1, -1)).flatten()
            
            # Get the predicted next position
            pred_position = y_pred[idx]
            
            # Get the actual next position
            actual_position = y_actual[idx]
            
            # Plot the trajectory segment
            plt.subplot(2, 3, i + 1)
            plt.plot([last_position[0], actual_position[0]], [last_position[1], actual_position[1]], 'b-', linewidth=2, label='Actual')
            plt.plot([last_position[0], pred_position[0]], [last_position[1], pred_position[1]], 'r--', linewidth=2, label='Predicted')
            plt.scatter(last_position[0], last_position[1], c='green', s=50, marker='o', label='Start')
            plt.scatter(actual_position[0], actual_position[1], c='blue', s=50, marker='x')
            plt.scatter(pred_position[0], pred_position[1], c='red', s=50, marker='x')
            
            # Add arrows to show direction
            plt.arrow(last_position[0], last_position[1], 
                     (actual_position[0] - last_position[0]) * 0.9, 
                     (actual_position[1] - last_position[1]) * 0.9,
                     color='blue', head_width=0.15, head_length=0.15, length_includes_head=True, alpha=0.7)
            plt.arrow(last_position[0], last_position[1], 
                     (pred_position[0] - last_position[0]) * 0.9, 
                     (pred_position[1] - last_position[1]) * 0.9,
                     color='red', head_width=0.15, head_length=0.15, length_includes_head=True, alpha=0.7)
            
            plt.title(f'Trajectory Segment {i+1}')
            plt.xlabel('X Position (m)')
            plt.ylabel('Y Position (m)')
            plt.grid(True)
            
            # Add trajectory error info
            error = np.sqrt((pred_position[0] - actual_position[0])**2 + (pred_position[1] - actual_position[1])**2)
            plt.annotate(f'Error: {error:.2f}m', xy=(0.05, 0.95), xycoords='axes fraction')
            
            if i == 0:
                plt.legend()
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, 'trajectory_segments.png'))
        plt.close()
    
    def run_pipeline(self):
        """Run the entire training pipeline"""
        # Load and preprocess data
        X_sensor_train, X_position_train, X_sensor_test, X_position_test, y_train, y_test = self.load_data()
        
        # Build model
        self.build_model()
        
        # Train model
        self.train_model(X_sensor_train, X_position_train, X_sensor_test, X_position_test, y_train, y_test)
        
        # Evaluate model
        self.evaluate_model(X_sensor_test, X_position_test, y_test)

if __name__ == '__main__':
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Train vessel position predictor model')
    parser.add_argument('--data_dir', type=str, default=None, 
                        help='Directory containing combined CSV files')
    parser.add_argument('--sequence_length', type=int, default=10, 
                        help='Length of input sequences (time steps)')
    parser.add_argument('--test_size', type=float, default=0.2, 
                        help='Proportion of data to use for testing')
    parser.add_argument('--epochs', type=int, default=100, 
                        help='Maximum number of training epochs')
    parser.add_argument('--batch_size', type=int, default=32, 
                        help='Batch size for training')
    parser.add_argument('--use_raw_data', action='store_true',
                        help='Use raw data instead of smoothed data')
    parser.add_argument('--position_delay', type=int, default=1,
                        help='How many steps in the future to predict')
    args = parser.parse_args()
    
    # Set default data directory if not provided
    if args.data_dir is None:
        args.data_dir = os.path.join(os.getcwd(), 'extracted_data')
    
    # Initialize the model trainer
    predictor = PositionPredictor(
        data_dir=args.data_dir,
        sequence_length=args.sequence_length,
        test_size=args.test_size,
        use_smoothed_data=not args.use_raw_data,
        position_delay=args.position_delay
    )
    
    # Run the pipeline
    predictor.run_pipeline() 