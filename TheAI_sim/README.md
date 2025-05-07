# Dead Reckoning with Neural Networks

This project implements a neural network approach to dead reckoning using IMU data from a marine vessel simulation. The model learns to predict vessel position and velocity by filtering IMU noise during integration.

## Overview

Traditional dead reckoning based on IMU data involves integrating acceleration and angular velocity measurements to estimate position. However, this approach suffers from accumulated errors due to IMU noise and drift. This project uses a Feedforward Neural Network (FNN) to learn the relationship between IMU measurements, previous positions, and velocities to predict future positions and velocities more accurately.

## Requirements

Required Python packages are listed in `requirements.txt`. Install them using:

```bash
pip install -r requirements.txt
```

## Dataset

The project uses CSV files containing IMU data from a marine vessel simulation. Each file contains:
- Timestamps
- Linear accelerations (x, y, z)
- Angular velocities (x, y, z)
- Linear velocities (x, y, z)
- Position (x, y, z)
- Orientation (quaternion)
- Control inputs (propeller, rudder)

## Model Architecture

The model is a Feedforward Neural Network (FNN) that:
1. Takes a sequence of previous states as input, including:
   - Previous x,y positions
   - Linear accelerations (x,y)
   - Angular velocity (z/yaw)
   - Previous velocities (x,y)
2. Predicts:
   - Next x,y position
   - Next x,y velocity

The model effectively learns to filter IMU noise and better estimate position and velocity.

## Scripts

### 1. Training

Use `train.py` to train the model:

```bash
python train.py --data_dir extracted_data --window_size 10 --hidden_layers 128,64,32 --epochs 100
```

Options:
- `--data_dir`: Directory containing CSV data files (default: "extracted_data")
- `--window_size`: Number of time steps to use as input context (default: 10)
- `--hidden_layers`: Comma-separated list of neurons in hidden layers (default: "128,64,32")
- `--learning_rate`: Learning rate for the optimizer (default: 0.001)
- `--batch_size`: Batch size for training (default: 32)
- `--epochs`: Number of training epochs (default: 100)
- `--model_path`: Directory to save the model (default: "dead_reckoning_model")

### 2. Prediction

Use `predict.py` to make predictions:

```bash
python predict.py --test_file extracted_data/test1_combined.csv --window_size 10 --prediction_steps 100
```

Options:
- `--test_file`: Path to CSV test data file (required)
- `--model_path`: Path to the trained model directory (default: "dead_reckoning_model")
- `--window_size`: Number of time steps used as input context (must match training) (default: 10)
- `--prediction_steps`: Number of steps to predict ahead (default: 100)
- `--start_idx`: Starting index in the test data for prediction (default: 0)
- `--output_dir`: Directory to save prediction results (default: "predictions")

## Usage Example

1. Train the model:
   ```bash
   python train.py
   ```

2. Make predictions:
   ```bash
   python predict.py --test_file extracted_data/test1_combined.csv
   ```

## Key Benefits

- **Improved Accuracy**: The neural network approach learns to filter IMU noise patterns, leading to more accurate position estimates compared to traditional integration methods.
- **Drift Correction**: The model can compensate for systematic biases and drift in IMU measurements.
- **Adaptability**: After training, the model adapts to the specific characteristics of the sensor package. 