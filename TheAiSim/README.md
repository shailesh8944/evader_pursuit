# Marine Vessel Position and Velocity Prediction Model

This project implements a feedforward neural network (FNN) to predict a marine vessel's 2D position and velocity based on current state and control inputs. The model learns to account for IMU sensor noise and bias to provide accurate dead reckoning.

## Overview

The model takes the following inputs:
- Current 2D position (x, y)
- Current 2D velocity (vx, vy)
- Current 2D acceleration from IMU (ax, ay)
- Angular velocity around z-axis (ang_vel_z)
- Rudder angle (control input)

And predicts:
- Next 2D position (x, y)
- Next 2D velocity (vx, vy)

## Files

- `train_model.py`: Script for training the neural network model
- `predict_position.py`: Script for making predictions using the trained model
- `extract_direct.py`: Script for extracting and preprocessing data from ROS2 bags (provided)

## Requirements

```
tensorflow
numpy
pandas
matplotlib
scikit-learn
```

Install dependencies with:

```bash
pip install tensorflow numpy pandas matplotlib scikit-learn
```

## Usage

### Data Extraction

The data extraction from ROS2 bags is handled by the provided `extract_direct.py` script, which generates synchronized CSV files with all necessary sensor data.

### Training the Model

To train the model using all available data:

```bash
python train_model.py --data_dir extracted_data --model_dir model
```

Parameters:
- `--data_dir`: Directory containing run* folders with synchronized CSV files (default: 'extracted_data')
- `--model_dir`: Directory to save the trained model (default: 'model')
- `--time_steps`: Number of time steps to look ahead for prediction (default: 1)

The training script will:
1. Load and combine data from all runs in the data directory
2. Preprocess the combined data
3. Split it into training and validation sets
4. Train the neural network model
5. Save the trained model and scalers to the specified directory
6. Generate evaluation plots

### Making Predictions

To evaluate the trained model on test data:

```bash
python predict_position.py --model_dir model --test_data extracted_data/run1/synchronized.csv
```

Parameters:
- `--model_dir`: Directory containing the trained model and scalers (default: 'model')
- `--test_data`: Path to test data file (default: 'extracted_data/run1/synchronized.csv')

The prediction script will:
1. Load the trained model and scalers
2. Evaluate the model on test data
3. Calculate performance metrics (MSE, RMSE)
4. Create visualizations of the predictions compared to actual trajectories

The model works by iteratively predicting the next state (position, velocity) based on the current state. Each prediction becomes input for the next time step, allowing the model to generate a complete trajectory from a starting point.

## Model Architecture

The feedforward neural network has the following architecture:
- Input layer: 8 features (pos_x, pos_y, vel_x, vel_y, accel_x, accel_y, ang_vel_z, rudder_angle)
- Hidden layers:
  - Dense layer with 64 units, ReLU activation + Batch Normalization
  - Dense layer with 128 units, ReLU activation + Batch Normalization + Dropout (0.2)
  - Dense layer with 128 units, ReLU activation + Batch Normalization + Dropout (0.2)
  - Dense layer with 64 units, ReLU activation + Batch Normalization
- Output layer: 4 units (pos_x, pos_y, vel_x, vel_y) with linear activation

## License

This project is provided as is, without any warranty. 