#!/usr/bin/env python3
import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import tensorflow as tf
from tensorflow.keras.models import Sequential, load_model
from tensorflow.keras.layers import Dense, Dropout, BatchNormalization
from tensorflow.keras.callbacks import EarlyStopping, ModelCheckpoint
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import argparse

def load_all_data(data_dir, time_steps=1):
    """
    Load and prepare data from all synchronized CSV files in the data directory
    
    Args:
        data_dir: Directory containing run* subdirectories with synchronized.csv files
        time_steps: Number of time steps to consider for each sample
        
    Returns:
        X_train, X_val, y_train, y_val, scalers: Training and validation data with scalers
    """
    print(f"Loading data from all runs in {data_dir}")
    
    # Find all synchronized.csv files in the data directory
    csv_files = glob.glob(os.path.join(data_dir, "run*/synchronized.csv"))
    
    if not csv_files:
        raise FileNotFoundError(f"No synchronized.csv files found in {data_dir}/run* directories")
    
    print(f"Found {len(csv_files)} data files:")
    for file in csv_files:
        print(f"  - {file}")
    
    # Initialize lists to store all data
    all_features = []
    all_targets = []
    
    # Define features for 2D prediction (removing yaw)
    # For input features: pos_x, pos_y, vel_x, vel_y, accel_x, accel_y, ang_vel_z, rudder_angle
    features = ['pos_x', 'pos_y', 'vel_x', 'vel_y', 'accel_x', 'accel_y', 'ang_vel_z', 'rudder_angle']
    
    # Process each file
    for csv_file in csv_files:
        try:
            # Load the synchronized data
            df = pd.read_csv(csv_file)
            
            # Check if required columns exist
            if not all(col in df.columns for col in features + ['timestamp']):
                missing_cols = [col for col in features + ['timestamp'] if col not in df.columns]
                print(f"Warning: Missing columns {missing_cols} in {csv_file}. Skipping file.")
                continue
            
            # Calculate time difference between consecutive samples
            timestamps = df['timestamp'].values
            dt = np.diff(timestamps)
            avg_dt = np.mean(dt)
            print(f"  Average time between samples for {os.path.basename(os.path.dirname(csv_file))}: {avg_dt:.4f} seconds")
            
            # Create features (X) and targets (y) for this file
            for i in range(len(df) - time_steps):
                # Input features
                feature_vector = df.iloc[i][features].values
                
                # Target (next position and velocity)
                target = df.iloc[i + time_steps][['pos_x', 'pos_y', 'vel_x', 'vel_y']].values
                
                all_features.append(feature_vector)
                all_targets.append(target)
                
            print(f"  Added {len(df) - time_steps} samples from {csv_file}")
            
        except Exception as e:
            print(f"Error processing {csv_file}: {str(e)}")
    
    if not all_features:
        raise ValueError("No valid data found in any of the CSV files")
    
    # Convert lists to numpy arrays
    X = np.array(all_features)
    y = np.array(all_targets)
    
    print(f"Total samples collected: {len(X)}")
    
    # Split data into training and validation sets
    X_train, X_val, y_train, y_val = train_test_split(X, y, test_size=0.2, random_state=42)
    
    # Standardize the input features
    x_scaler = StandardScaler()
    X_train = x_scaler.fit_transform(X_train)
    X_val = x_scaler.transform(X_val)
    
    # Standardize the target values
    y_scaler = StandardScaler()
    y_train = y_scaler.fit_transform(y_train)
    y_val = y_scaler.transform(y_val)
    
    # Print shapes
    print(f"X_train shape: {X_train.shape}")
    print(f"y_train shape: {y_train.shape}")
    print(f"X_val shape: {X_val.shape}")
    print(f"y_val shape: {y_val.shape}")
    
    return X_train, X_val, y_train, y_val, (x_scaler, y_scaler)

def build_model(input_shape, output_shape=4):
    """
    Build a feedforward neural network model
    
    Args:
        input_shape: Shape of input features
        output_shape: Shape of output features (default: 4 for pos_x, pos_y, vel_x, vel_y)
        
    Returns:
        Compiled Keras model
    """
    model = Sequential([
        # Input layer
        Dense(64, activation='relu', input_shape=(input_shape,)),
        BatchNormalization(),
        
        # Hidden layers
        Dense(128, activation='relu'),
        BatchNormalization(),
        Dropout(0.2),
        
        Dense(128, activation='relu'),
        BatchNormalization(),
        Dropout(0.2),
        
        Dense(64, activation='relu'),
        BatchNormalization(),
        
        # Output layer - linear activation for regression
        Dense(output_shape, activation='linear')
    ])
    
    # Compile the model with explicitly named loss function
    model.compile(
        optimizer='adam',
        loss=tf.keras.losses.MeanSquaredError(name='mean_squared_error')
    )
    
    model.summary()
    return model

def train_model(X_train, y_train, X_val, y_val, model_dir):
    """
    Train the neural network model
    
    Args:
        X_train, y_train: Training data
        X_val, y_val: Validation data
        model_dir: Directory to save the model
        
    Returns:
        Trained model and training history
    """
    os.makedirs(model_dir, exist_ok=True)
    
    # Build the model
    model = build_model(X_train.shape[1])
    
    # Set up callbacks
    early_stopping = EarlyStopping(
        monitor='val_loss',
        patience=10,
        restore_best_weights=True
    )
    
    model_checkpoint = ModelCheckpoint(
        os.path.join(model_dir, 'best_model.h5'),
        monitor='val_loss',
        save_best_only=True
    )
    
    # Train the model
    history = model.fit(
        X_train, y_train,
        validation_data=(X_val, y_val),
        epochs=50,
        batch_size=32,
        callbacks=[early_stopping, model_checkpoint],
        verbose=1
    )
    
    # Save the final model
    model.save(os.path.join(model_dir, 'final_model.h5'))
    
    return model, history

def evaluate_model(model, X_val, y_val, y_scaler, history):
    """
    Evaluate the model and show prediction examples
    
    Args:
        model: Trained model
        X_val, y_val: Validation data
        y_scaler: Scaler for the target values
        history: Training history
    """
    # Make predictions on validation data
    y_pred = model.predict(X_val)
    
    # Inverse transform predictions and actual values
    y_val_orig = y_scaler.inverse_transform(y_val)
    y_pred_orig = y_scaler.inverse_transform(y_pred)
    
    # Calculate errors
    mse = np.mean((y_val_orig - y_pred_orig) ** 2, axis=0)
    rmse = np.sqrt(mse)
    
    print("Validation MSE:")
    print(f"Position X: {mse[0]:.6f}")
    print(f"Position Y: {mse[1]:.6f}")
    print(f"Velocity X: {mse[2]:.6f}")
    print(f"Velocity Y: {mse[3]:.6f}")
    print("\nValidation RMSE:")
    print(f"Position X: {rmse[0]:.6f}")
    print(f"Position Y: {rmse[1]:.6f}")
    print(f"Velocity X: {rmse[2]:.6f}")
    print(f"Velocity Y: {rmse[3]:.6f}")
    
    # Plot a sample of predictions vs ground truth
    sample_size = min(100, len(y_val_orig))
    indices = np.random.choice(len(y_val_orig), sample_size, replace=False)
    
    plt.figure(figsize=(15, 10))
    
    # Position X
    plt.subplot(2, 2, 1)
    plt.scatter(y_val_orig[indices, 0], y_pred_orig[indices, 0])
    plt.plot([-10, 10], [-10, 10], 'r--')
    plt.xlabel('True Position X')
    plt.ylabel('Predicted Position X')
    plt.title('Position X Prediction')
    
    # Position Y
    plt.subplot(2, 2, 2)
    plt.scatter(y_val_orig[indices, 1], y_pred_orig[indices, 1])
    plt.plot([-10, 10], [-10, 10], 'r--')
    plt.xlabel('True Position Y')
    plt.ylabel('Predicted Position Y')
    plt.title('Position Y Prediction')
    
    # Velocity X
    plt.subplot(2, 2, 3)
    plt.scatter(y_val_orig[indices, 2], y_pred_orig[indices, 2])
    plt.plot([-1, 1], [-1, 1], 'r--')
    plt.xlabel('True Velocity X')
    plt.ylabel('Predicted Velocity X')
    plt.title('Velocity X Prediction')
    
    # Velocity Y
    plt.subplot(2, 2, 4)
    plt.scatter(y_val_orig[indices, 3], y_pred_orig[indices, 3])
    plt.plot([-1, 1], [-1, 1], 'r--')
    plt.xlabel('True Velocity Y')
    plt.ylabel('Predicted Velocity Y')
    plt.title('Velocity Y Prediction')
    
    plt.tight_layout()
    plt.savefig('prediction_comparison.png')
    
    # Plot training history
    plt.figure(figsize=(12, 4))
    plt.subplot(1, 1, 1)
    plt.plot(history.history['loss'], label='Training Loss')
    plt.plot(history.history['val_loss'], label='Validation Loss')
    plt.title('Training and Validation Loss')
    plt.xlabel('Epochs')
    plt.ylabel('Loss')
    plt.legend()
    plt.savefig('training_history.png')
    
    # Plot trajectory comparison for a sequence of points
    seq_length = 100
    start_idx = np.random.randint(0, len(X_val) - seq_length)
    
    # Generate sequential predictions
    x_seq = X_val[start_idx:start_idx + seq_length]
    y_true_seq = y_scaler.inverse_transform(y_val[start_idx:start_idx + seq_length])
    y_pred_seq = y_scaler.inverse_transform(model.predict(x_seq))
    
    plt.figure(figsize=(10, 8))
    plt.plot(y_true_seq[:, 0], y_true_seq[:, 1], 'b-', label='True Trajectory')
    plt.plot(y_pred_seq[:, 0], y_pred_seq[:, 1], 'r--', label='Predicted Trajectory')
    plt.scatter(y_true_seq[0, 0], y_true_seq[0, 1], c='g', s=100, label='Start')
    plt.scatter(y_true_seq[-1, 0], y_true_seq[-1, 1], c='k', s=100, label='End')
    plt.xlabel('Position X')
    plt.ylabel('Position Y')
    plt.title('Trajectory Comparison')
    plt.legend()
    plt.grid(True)
    plt.savefig('trajectory_comparison.png')

def plot_learning_curves(history):
    """
    Plot learning curves from training history
    
    Args:
        history: Training history from model.fit()
    """
    plt.figure(figsize=(10, 4))
    plt.plot(history.history['loss'], label='Training Loss')
    plt.plot(history.history['val_loss'], label='Validation Loss')
    plt.title('Training and Validation Loss')
    plt.xlabel('Epochs')
    plt.ylabel('Loss')
    plt.legend()
    plt.savefig('learning_curves.png')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Train a neural network model for predicting vessel position and velocity')
    parser.add_argument('--data_dir', type=str, default='extracted_data',
                        help='Directory containing run folders with synchronized.csv files')
    parser.add_argument('--model_dir', type=str, default='model',
                        help='Directory to save the trained model')
    parser.add_argument('--time_steps', type=int, default=1,
                        help='Number of time steps to look ahead for prediction')
    
    args = parser.parse_args()
    
    # Load and prepare data from all runs
    X_train, X_val, y_train, y_val, scalers = load_all_data(args.data_dir, args.time_steps)
    x_scaler, y_scaler = scalers
    
    # Train the model
    model, history = train_model(X_train, y_train, X_val, y_val, args.model_dir)
    
    # Evaluate the model
    evaluate_model(model, X_val, y_val, y_scaler, history)
    
    # Plot learning curves
    plot_learning_curves(history)
    
    # Save the scalers for later use
    import pickle
    with open(os.path.join(args.model_dir, 'x_scaler.pkl'), 'wb') as f:
        pickle.dump(x_scaler, f)
    
    with open(os.path.join(args.model_dir, 'y_scaler.pkl'), 'wb') as f:
        pickle.dump(y_scaler, f)
    
    print(f"Training complete. Model saved to {args.model_dir}.") 