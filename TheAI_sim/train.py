#!/usr/bin/env python3
import os
import argparse
import numpy as np
from model import DeadReckoningModel
import matplotlib.pyplot as plt

def main():
    parser = argparse.ArgumentParser(description='Train a dead reckoning model using IMU data')
    parser.add_argument('--data_dir', type=str, default='extracted_data',
                        help='Directory containing CSV data files')
    parser.add_argument('--window_size', type=int, default=10,
                        help='Number of time steps to use as input context')
    parser.add_argument('--hidden_layers', type=str, default='128,64,32',
                        help='Comma-separated list of neurons in hidden layers')
    parser.add_argument('--learning_rate', type=float, default=0.001,
                        help='Learning rate for the optimizer')
    parser.add_argument('--batch_size', type=int, default=32,
                        help='Batch size for training')
    parser.add_argument('--epochs', type=int, default=100,
                        help='Number of training epochs')
    parser.add_argument('--model_path', type=str, default='dead_reckoning_model',
                        help='Directory to save the model')
    args = parser.parse_args()
    
    # Parse hidden layers from string
    hidden_layers = [int(x) for x in args.hidden_layers.split(',')]
    
    # Find all CSV files in the data directory
    data_files = []
    for file in os.listdir(args.data_dir):
        if file.endswith('.csv'):
            data_files.append(os.path.join(args.data_dir, file))
    
    if not data_files:
        print(f"No CSV files found in {args.data_dir}")
        return
    
    print(f"Found {len(data_files)} data files: {data_files}")
    
    # Create and train the model
    model = DeadReckoningModel(
        window_size=args.window_size,
        hidden_layers=hidden_layers,
        learning_rate=args.learning_rate,
        model_path=args.model_path
    )
    
    # Prepare data
    print("Preparing data...")
    X_train, X_val, y_train, y_val = model.prepare_data(data_files)
    print(f"Training data shape: {X_train.shape}, {y_train.shape}")
    print(f"Validation data shape: {X_val.shape}, {y_val.shape}")
    
    # Train the model
    print("Training model...")
    history = model.train(
        X_train, y_train,
        X_val, y_val,
        epochs=args.epochs,
        batch_size=args.batch_size
    )
    
    # Plot training history
    model.plot_training_history(history)
    
    # Evaluate on validation data
    print("Evaluating model on validation data...")
    metrics = model.evaluate_error(X_val, y_val)
    
    print("\nValidation Metrics:")
    for key, value in metrics.items():
        print(f"{key}: {value:.6f}")
    
    print(f"\nModel saved to {args.model_path}")

if __name__ == "__main__":
    main() 