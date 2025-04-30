# Marine Vessel Position Prediction Model

This project implements a deep learning model to predict the next position of a marine vessel based on current control inputs, position, and IMU data. It's designed for a Markov process-based position prediction.

## Setup

### Requirements

- Python 3.6+
- TensorFlow 2.x
- NumPy
- Pandas
- Matplotlib
- scikit-learn
- joblib

Install the required packages:

```bash
pip install tensorflow numpy pandas matplotlib scikit-learn joblib
```

## Data Processing

1. First, process the ROS bag files to extract data:

```bash
python extract_data.py --base_dir ../data_from_test --output_dir extracted_data
```

The data extraction process properly synchronizes between IMU data (100Hz) and other topics (10Hz) to ensure that the closest IMU sample is associated with each actuator/odometry sample.

## Training the Model

Train the position prediction model:

```bash
python vessel_predictor.py --data_dir extracted_data --model_dir model --results_dir results
```

Options:
- `--data_dir`: Directory containing extracted data files
- `--model_dir`: Directory to save the trained model
- `--results_dir`: Directory to save results and plots
- `--file_pattern`: Pattern to match data files (default: "*_combined.csv")
- `--batch_size`: Batch size for training (default: 32)
- `--epochs`: Maximum number of epochs (default: 100)

## Predicting Trajectories

Predict and visualize vessel trajectories:

```bash
python trajectory_predictor.py --model_dir model --test_file extracted_data/my_test_file.csv --segment_length 100
```

Options:
- `--model_dir`: Directory containing the trained model
- `--test_file`: CSV file containing test data (required)
- `--start_idx`: Starting index for the trajectory segment (default: 0)
- `--segment_length`: Length of the trajectory segment (default: 100)
- `--results_dir`: Directory to save results (default: "trajectory_results")

## Model Details

The model is a feed-forward neural network with the following inputs:
- Control inputs (rudder, propeller)
- Current position (x, y)
- IMU data (linear accelerations, angular velocities)

Output:
- Predicted next position (x, y)

A standard scaler is used to normalize the input features.

## Results

The model generates several plots to evaluate its performance:
- Training history (loss and MAE)
- Actual vs predicted position scatter plot
- Error histogram
- Error CDF (Cumulative Distribution Function)
- Trajectory comparison
- Prediction error over time

These plots help assess the quality of the position predictions and understand error patterns.

## Files

- `extract_data.py`: Extracts and processes data from ROS bag files, synchronizing between 100Hz IMU and 10Hz topics
- `vessel_predictor.py`: Trains and evaluates the position prediction model
- `trajectory_predictor.py`: Predicts vessel trajectories using the trained model 