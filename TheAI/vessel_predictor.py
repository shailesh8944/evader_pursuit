#!/usr/bin/env python3
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import tensorflow as tf
from tensorflow.keras.models import Sequential, load_model
from tensorflow.keras.layers import Dense, Dropout, BatchNormalization
from tensorflow.keras.callbacks import EarlyStopping, ModelCheckpoint, ReduceLROnPlateau
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import argparse
import glob
from datetime import datetime


def load_and_prepare_data(data_dir, file_pattern='*_combined.csv'):
    """
    Load and prepare data from combined CSV files
    
    Args:
        data_dir: Directory containing the extracted data
        file_pattern: Pattern to match combined data files
        
    Returns:
        X: Input features 
        y: Target values (change in position: delta_x, delta_y)
    """
    # Find all combined data files
    data_files = glob.glob(os.path.join(data_dir, file_pattern))
    
    if not data_files:
        raise ValueError(f"No data files found matching pattern '{file_pattern}' in directory '{data_dir}'")
    
    print(f"Found {len(data_files)} data files")
    
    # Load and combine all data files
    dfs = []
    for file in data_files:
        df = pd.read_csv(file)
        # Analyze IMU data quality - focus on the 3 relevant IMU measurements
        unique_imu_values = len(df[['linear_acc_x', 'linear_acc_y', 'angular_vel_z']].drop_duplicates())
        print(f"File {os.path.basename(file)}: {len(df)} samples, {unique_imu_values} unique IMU readings")
        
        dfs.append(df)
    
    # Concatenate all dataframes
    if len(dfs) > 1:
        data = pd.concat(dfs, ignore_index=True)
    else:
        data = dfs[0]
    
    print(f"Combined dataset shape: {data.shape}")
    
    # Calculate the change in position (delta)
    data['delta_x'] = data['position_x'].shift(-1) - data['position_x']
    data['delta_y'] = data['position_y'].shift(-1) - data['position_y']
    
    # Drop the last row which has NaN values for deltas
    data = data.dropna(subset=['delta_x', 'delta_y'])
    
    # Print all column names to verify
    print("Available columns in dataset:")
    print(data.columns.tolist())
    
    # Explicitly exclude timestamp by selecting only the required features
    # Focus on relevant IMU data: x,y accelerations and z angular velocity
    X = data[[
        'rudder', 'propeller',                   # Control inputs
        'position_x', 'position_y',              # Current position
        'linear_acc_x', 'linear_acc_y',          # X,Y linear accelerations only
        'angular_vel_z'                          # Z angular velocity only
    ]].values
    
    # Select output targets: change in position
    y = data[['delta_x', 'delta_y']].values
    
    # Print feature shapes to verify
    print(f"Input features shape: {X.shape}")
    print(f"Output targets shape: {y.shape} (delta_x, delta_y)")
    
    return X, y


def create_model(input_shape):
    """
    Create a neural network model for position prediction with special focus on IMU data
    
    Args:
        input_shape: Shape of the input features
        
    Returns:
        model: Compiled Keras model
    """
    # Define input feature groups for separate processing
    inputs = tf.keras.Input(shape=(input_shape,))
    
    # Extract feature groups - updated for reduced IMU features
    control_inputs = tf.keras.layers.Lambda(lambda x: x[:, 0:2])(inputs)  # rudder, propeller
    position = tf.keras.layers.Lambda(lambda x: x[:, 2:4])(inputs)        # position_x, position_y
    imu_data = tf.keras.layers.Lambda(lambda x: x[:, 4:7])(inputs)        # linear_acc_x, linear_acc_y, angular_vel_z
    
    # Process control inputs
    control_branch = tf.keras.layers.Dense(32, activation='relu')(control_inputs)
    control_branch = tf.keras.layers.BatchNormalization()(control_branch)
    
    # Process position data
    position_branch = tf.keras.layers.Dense(32, activation='relu')(position)
    position_branch = tf.keras.layers.BatchNormalization()(position_branch)
    
    # Process IMU data with special attention
    imu_branch = tf.keras.layers.Dense(64, activation='relu')(imu_data)
    imu_branch = tf.keras.layers.BatchNormalization()(imu_branch)
    imu_branch = tf.keras.layers.Dense(64, activation='relu')(imu_branch)
    imu_branch = tf.keras.layers.BatchNormalization()(imu_branch)
    
    # Combine all branches
    combined = tf.keras.layers.Concatenate()([control_branch, position_branch, imu_branch])
    
    # Final processing
    x = tf.keras.layers.Dense(128, activation='relu')(combined)
    x = tf.keras.layers.BatchNormalization()(x)
    x = tf.keras.layers.Dropout(0.2)(x)
    
    x = tf.keras.layers.Dense(256, activation='relu')(x)
    x = tf.keras.layers.BatchNormalization()(x)
    x = tf.keras.layers.Dropout(0.3)(x)
    
    x = tf.keras.layers.Dense(128, activation='relu')(x)
    x = tf.keras.layers.BatchNormalization()(x)
    x = tf.keras.layers.Dropout(0.2)(x)
    
    x = tf.keras.layers.Dense(64, activation='relu')(x)
    x = tf.keras.layers.BatchNormalization()(x)
    x = tf.keras.layers.Dropout(0.1)(x)
    
    # Output layer for position prediction
    outputs = tf.keras.layers.Dense(2, activation='linear')(x)
    
    # Create model
    model = tf.keras.Model(inputs=inputs, outputs=outputs)
    
    # Compile with Huber loss for robustness to outliers
    optimizer = tf.keras.optimizers.Adam(learning_rate=0.001)
    
    model.compile(
        optimizer=optimizer,
        loss=tf.keras.losses.Huber(),
        metrics=[tf.keras.metrics.MeanAbsoluteError()]
    )
    
    # Print model summary
    model.summary()
    
    return model


def train_model(X, y, model_dir='model', batch_size=32, epochs=200, validation_split=0.2):
    """
    Train the neural network model
    
    Args:
        X: Input features
        y: Target values
        model_dir: Directory to save the model
        batch_size: Batch size for training
        epochs: Maximum number of epochs
        validation_split: Fraction of data to use for validation
        
    Returns:
        model: Trained model
        history: Training history
        X_train, X_val, y_train, y_val: Train/validation split data
    """
    # Create model directory if it doesn't exist
    os.makedirs(model_dir, exist_ok=True)
    
    # Split data into train and validation sets
    X_train, X_val, y_train, y_val = train_test_split(X, y, test_size=validation_split, random_state=42)
    
    # Standardize the input features
    scaler = StandardScaler()
    X_train = scaler.fit_transform(X_train)
    X_val = scaler.transform(X_val)
    
    # Save the scaler for later use
    import joblib
    joblib.dump(scaler, os.path.join(model_dir, 'scaler.pkl'))
    
    # Create the model
    model = create_model(X_train.shape[1])
    
    # Set up callbacks with more patience and improved monitoring
    callbacks = [
        EarlyStopping(
            patience=30,
            restore_best_weights=True,
            monitor='val_loss'
        ),
        ReduceLROnPlateau(
            factor=0.5,
            patience=15,
            min_lr=1e-6,
            monitor='val_loss',
            verbose=1
        ),
        ModelCheckpoint(
            os.path.join(model_dir, 'best_model.h5'),
            save_best_only=True,
            monitor='val_loss'
        )
    ]
    
    # Train the model
    history = model.fit(
        X_train, y_train,
        batch_size=batch_size,
        epochs=epochs,
        validation_data=(X_val, y_val),
        callbacks=callbacks,
        verbose=1
    )
    
    # Save the final model
    model.save(os.path.join(model_dir, 'final_model.h5'))
    
    return model, history, X_train, X_val, y_train, y_val, scaler


def evaluate_model(model, X_val, y_val, scaler, results_dir='results'):
    """
    Evaluate the model and generate plots
    
    Args:
        model: Trained model
        X_val: Validation features
        y_val: Validation targets
        scaler: Fitted scaler for the input features
        results_dir: Directory to save results
    """
    # Create results directory if it doesn't exist
    os.makedirs(results_dir, exist_ok=True)
    
    # Make predictions
    y_pred = model.predict(X_val)
    
    # Calculate error metrics
    mse = np.mean((y_pred - y_val) ** 2)
    mae = np.mean(np.abs(y_pred - y_val))
    rmse = np.sqrt(mse)
    
    # Calculate Euclidean distance error
    euclidean_error = np.sqrt(np.sum((y_pred - y_val) ** 2, axis=1))
    mean_dist_error = np.mean(euclidean_error)
    median_dist_error = np.median(euclidean_error)
    
    print(f"\nModel Evaluation Metrics:")
    print(f"MSE: {mse:.6f}")
    print(f"MAE: {mae:.6f}")
    print(f"RMSE: {rmse:.6f}")
    print(f"Mean Euclidean Error: {mean_dist_error:.6f} meters")
    print(f"Median Euclidean Error: {median_dist_error:.6f} meters")
    
    # Plot actual vs predicted positions
    plt.figure(figsize=(10, 10))
    plt.scatter(y_val[:, 0], y_val[:, 1], label='Actual', alpha=0.5)
    plt.scatter(y_pred[:, 0], y_pred[:, 1], label='Predicted', alpha=0.5)
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Actual vs Predicted Positions')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, 'position_comparison.png'), dpi=300)
    
    # Plot error histogram
    plt.figure(figsize=(10, 6))
    plt.hist(euclidean_error, bins=50)
    plt.xlabel('Euclidean Error (m)')
    plt.ylabel('Frequency')
    plt.title('Distribution of Position Prediction Errors')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, 'error_histogram.png'), dpi=300)
    
    # Plot error CDF
    plt.figure(figsize=(10, 6))
    sorted_errors = np.sort(euclidean_error)
    p = np.arange(1, len(sorted_errors) + 1) / len(sorted_errors)
    plt.plot(sorted_errors, p)
    plt.xlabel('Euclidean Error (m)')
    plt.ylabel('Cumulative Probability')
    plt.title('CDF of Position Prediction Errors')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, 'error_cdf.png'), dpi=300)
    
    # Save metrics to file
    with open(os.path.join(results_dir, 'metrics.txt'), 'w') as f:
        f.write(f"Model Evaluation Metrics:\n")
        f.write(f"MSE: {mse:.6f}\n")
        f.write(f"MAE: {mae:.6f}\n")
        f.write(f"RMSE: {rmse:.6f}\n")
        f.write(f"Mean Euclidean Error: {mean_dist_error:.6f} meters\n")
        f.write(f"Median Euclidean Error: {median_dist_error:.6f} meters\n")
        f.write(f"90th Percentile Error: {np.percentile(euclidean_error, 90):.6f} meters\n")
        f.write(f"95th Percentile Error: {np.percentile(euclidean_error, 95):.6f} meters\n")
        f.write(f"99th Percentile Error: {np.percentile(euclidean_error, 99):.6f} meters\n")
    
    return {
        'mse': mse,
        'mae': mae,
        'rmse': rmse,
        'mean_dist_error': mean_dist_error,
        'median_dist_error': median_dist_error
    }


def predict_trajectory(model, start_state, controls, num_steps, scaler):
    """
    Predict a vessel trajectory given initial state and control inputs
    
    Args:
        model: Trained model
        start_state: Initial state vector [x, y, lin_acc_x, lin_acc_y, lin_acc_z, ang_vel_x, ang_vel_y, ang_vel_z]
        controls: Control inputs for each step [[rudder, propeller], ...]
        num_steps: Number of steps to predict
        scaler: Fitted scaler for the input features
        
    Returns:
        trajectory: Predicted trajectory points
    """
    # Ensure controls array has the right shape
    if len(controls) < num_steps:
        # Repeat the last control if not enough controls are provided
        controls = np.vstack([controls, np.tile(controls[-1], (num_steps - len(controls), 1))])
    
    # Initialize trajectory with the start position
    trajectory = np.zeros((num_steps + 1, 2))
    trajectory[0] = start_state[:2]  # x, y
    
    # Current state
    current_state = np.array(start_state)
    
    # Predict future positions step by step
    for i in range(num_steps):
        # Prepare input for model
        model_input = np.concatenate([
            controls[i],  # rudder, propeller
            current_state[:2],  # x, y position
            current_state[2:]  # IMU data
        ]).reshape(1, -1)
        
        # Scale the input
        model_input_scaled = scaler.transform(model_input)
        
        # Predict the next position
        next_position = model.predict(model_input_scaled, verbose=0)[0]
        
        # Update trajectory
        trajectory[i+1] = next_position
        
        # Update current state for next iteration
        current_state[:2] = next_position
        
        # Note: In a real application, you would update the IMU data too
        # Here we're keeping it simple and reusing the same IMU data
    
    return trajectory


def plot_predicted_vs_actual_trajectory(actual_trajectory, predicted_trajectory, results_dir='results'):
    """
    Plot actual vs predicted trajectory
    
    Args:
        actual_trajectory: Actual trajectory points
        predicted_trajectory: Predicted trajectory points
        results_dir: Directory to save results
    """
    plt.figure(figsize=(12, 10))
    
    # Plot actual trajectory
    plt.plot(actual_trajectory[:, 0], actual_trajectory[:, 1], 'b-', linewidth=2, label='Actual')
    plt.scatter(actual_trajectory[0, 0], actual_trajectory[0, 1], c='blue', s=100, marker='o', label='Start')
    plt.scatter(actual_trajectory[-1, 0], actual_trajectory[-1, 1], c='blue', s=100, marker='x', label='End')
    
    # Plot predicted trajectory
    plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], 'r--', linewidth=2, label='Predicted')
    plt.scatter(predicted_trajectory[0, 0], predicted_trajectory[0, 1], c='red', s=100, marker='o')
    plt.scatter(predicted_trajectory[-1, 0], predicted_trajectory[-1, 1], c='red', s=100, marker='x')
    
    # Add arrows to show direction
    arrow_indices = np.linspace(0, len(actual_trajectory)-1, 20, dtype=int)
    for i in arrow_indices:
        if i+1 < len(actual_trajectory):
            plt.arrow(actual_trajectory[i, 0], actual_trajectory[i, 1],
                     (actual_trajectory[i+1, 0] - actual_trajectory[i, 0]) * 0.8,
                     (actual_trajectory[i+1, 1] - actual_trajectory[i, 1]) * 0.8,
                     head_width=0.05, head_length=0.1, fc='blue', ec='blue', alpha=0.3)
    
    arrow_indices = np.linspace(0, len(predicted_trajectory)-1, 20, dtype=int)
    for i in arrow_indices:
        if i+1 < len(predicted_trajectory):
            plt.arrow(predicted_trajectory[i, 0], predicted_trajectory[i, 1],
                     (predicted_trajectory[i+1, 0] - predicted_trajectory[i, 0]) * 0.8,
                     (predicted_trajectory[i+1, 1] - predicted_trajectory[i, 1]) * 0.8,
                     head_width=0.05, head_length=0.1, fc='red', ec='red', alpha=0.3)
    
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Actual vs Predicted Vessel Trajectory')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.tight_layout()
    
    # Save the plot
    os.makedirs(results_dir, exist_ok=True)
    plt.savefig(os.path.join(results_dir, 'trajectory_comparison.png'), dpi=300)
    plt.close()


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Train and evaluate vessel position prediction model')
    parser.add_argument('--data_dir', type=str, default='extracted_data',
                       help='Directory containing the extracted data CSV files')
    parser.add_argument('--model_dir', type=str, default='model',
                       help='Directory to save the trained model')
    parser.add_argument('--results_dir', type=str, default='results',
                       help='Directory to save results and plots')
    parser.add_argument('--file_pattern', type=str, default='*_combined.csv',
                       help='Pattern to match data files')
    parser.add_argument('--batch_size', type=int, default=32,
                       help='Batch size for training')
    parser.add_argument('--epochs', type=int, default=200,
                       help='Maximum number of epochs')
    parser.add_argument('--validate', action='store_true',
                       help='Validate an existing model instead of training')
    args = parser.parse_args()
    
    # Ensure TensorFlow uses GPU if available
    physical_devices = tf.config.list_physical_devices('GPU')
    if physical_devices:
        print(f"Found {len(physical_devices)} GPU(s):")
        for device in physical_devices:
            print(f"  {device}")
        
        # Allow memory growth for GPUs
        for device in physical_devices:
            tf.config.experimental.set_memory_growth(device, True)
    else:
        print("No GPU found, using CPU")
    
    # Load and prepare data
    X, y = load_and_prepare_data(args.data_dir, args.file_pattern)
    print(f"Loaded data: X shape = {X.shape}, y shape = {y.shape}")
    
    if not args.validate:
        # Train the model
        print("\nTraining model...")
        model, history, X_train, X_val, y_train, y_val, scaler = train_model(
            X, y, 
            model_dir=args.model_dir, 
            batch_size=args.batch_size, 
            epochs=args.epochs
        )
        
        # Plot training history
        plt.figure(figsize=(12, 5))
        
        # Plot training & validation loss
        plt.subplot(1, 2, 1)
        plt.plot(history.history['loss'])
        plt.plot(history.history['val_loss'])
        plt.title('Model Loss')
        plt.ylabel('Loss')
        plt.xlabel('Epoch')
        plt.legend(['Train', 'Validation'], loc='upper right')
        
        # Plot training & validation MAE
        plt.subplot(1, 2, 2)
        plt.plot(history.history['mean_absolute_error'])
        plt.plot(history.history['val_mean_absolute_error'])
        plt.title('Model MAE')
        plt.ylabel('MAE')
        plt.xlabel('Epoch')
        plt.legend(['Train', 'Validation'], loc='upper right')
        
        plt.tight_layout()
        os.makedirs(args.results_dir, exist_ok=True)
        plt.savefig(os.path.join(args.results_dir, 'training_history.png'), dpi=300)
        
        # Evaluate the model
        print("\nEvaluating model...")
        metrics = evaluate_model(model, X_val, y_val, scaler, results_dir=args.results_dir)
        
    else:
        # Load existing model and scaler
        print("\nLoading existing model for validation...")
        model_path = os.path.join(args.model_dir, 'final_model.h5')
        scaler_path = os.path.join(args.model_dir, 'scaler.pkl')
        
        if not os.path.exists(model_path) or not os.path.exists(scaler_path):
            print(f"Error: Model files not found in {args.model_dir}")
            return
        
        model = load_model(model_path)
        import joblib
        scaler = joblib.load(scaler_path)
        
        # Split data for validation
        _, X_val, _, y_val = train_test_split(X, y, test_size=0.2, random_state=42)
        X_val = scaler.transform(X_val)
        
        # Evaluate the model
        print("\nEvaluating model...")
        metrics = evaluate_model(model, X_val, y_val, scaler, results_dir=args.results_dir)

    print("\nDone!")


if __name__ == "__main__":
    main() 