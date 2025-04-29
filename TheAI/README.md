# Marine Vessel Position Predictor

This project implements a deep learning model to predict the position (x, y) of a marine vessel based on IMU data (linear and angular acceleration) and control inputs (rudder angle and propeller power).

## Project Structure

- `extract_data.py`: Script to extract data from ROS2 bags and save to CSV files
- `train_model.py`: Script to train the deep learning model on the extracted data
- `predict.py`: Script to make predictions using the trained model
- `requirements.txt`: List of required Python packages

## Data Processing

### Data Extraction

The `extract_data.py` script extracts the following data from ROS2 bags:

1. IMU data from `/sookshma_00/imu/data`: Linear and angular acceleration
2. Odometry data from `/sookshma_00/odometry`: Position (x, y, z) and orientation
3. Actuator commands from `/sookshma_00/actuator_cmd`: Rudder angle and propeller power

The data is saved to CSV files in the `extracted_data` directory, including separate files for each data type and a combined file with synchronized timestamps.

### Data Synchronization

Since the data is published at different rates (IMU at 100Hz, odometry and actuator commands at 10Hz), the script synchronizes the data based on the actuator command timestamps. For each actuator command, the closest IMU and odometry data points are matched.

### Data Smoothing and Filtering

The position data may contain noise and random jumps. To address this, the following preprocessing steps are applied:

1. **Gaussian Smoothing**: The position data (x, y) is smoothed using a Gaussian filter with a sigma of 2.0 to reduce noise while preserving the overall trajectory shape.

2. **Minimum Distance Filtering**: Points that are too close to each other (less than 0.3 meters apart) are filtered out to remove redundant data and reduce the impact of noise.

3. **Separate Datasets**: Both the original and smoothed data are saved to separate CSV files, allowing for comparison and training on either dataset.

The smoothed data provides several advantages:
- Reduced noise in the training data
- More consistent trajectory patterns
- Less overfitting to noisy data points
- Improved generalization of the model

You can choose whether to use the raw or smoothed data for training with the `--use_raw_data` flag in the training script.

## Model Architecture

The model uses a dual-input Bidirectional LSTM (Bi-LSTM) architecture to enable proper dead reckoning capabilities.

### Input

The model now takes two separate input streams:
1. **Sensor Data Stream**: Sequence of IMU data (linear and angular acceleration) and control inputs (rudder, propeller)
   - Features per time step: 8 (linear_acc_x, linear_acc_y, linear_acc_z, angular_vel_x, angular_vel_y, angular_vel_z, rudder, propeller)

2. **Position History Stream**: Sequence of previous position data (x, y)
   - Features per time step: 2 (position_x, position_y)

Both input streams share the same sequence length (default: 10 time steps).

### Architecture

```
Model: "model"
__________________________________________________________________________________________________
 Layer (type)                    Output Shape         Param #     Connected to                     
==================================================================================================
 sensor_input (InputLayer)       [(None, 10, 8)]      0           []                               
                                                                                                  
 position_input (InputLayer)     [(None, 10, 2)]      0           []                               
                                                                                                  
 bidirectional (Bidirectional)   (None, 10, 128)      37,376      ['sensor_input[0][0]']           
                                                                                                  
 bidirectional_2 (Bidirectional) (None, 10, 64)       20,864      ['position_input[0][0]']         
                                                                                                  
 dropout (Dropout)               (None, 10, 128)      0           ['bidirectional[0][0]']          
                                                                                                  
 dropout_2 (Dropout)             (None, 10, 64)       0           ['bidirectional_2[0][0]']        
                                                                                                  
 bidirectional_1 (Bidirectional) (None, 64)           41,216      ['dropout[0][0]']                
                                                                                                  
 bidirectional_3 (Bidirectional) (None, 32)           10,368      ['dropout_2[0][0]']              
                                                                                                  
 dropout_1 (Dropout)             (None, 64)           0           ['bidirectional_1[0][0]']        
                                                                                                  
 dropout_3 (Dropout)             (None, 32)           0           ['bidirectional_3[0][0]']        
                                                                                                  
 concatenate (Concatenate)       (None, 96)           0           ['dropout_1[0][0]',              
                                                                  'dropout_3[0][0]']               
                                                                                                  
 dense (Dense)                   (None, 32)           3,104       ['concatenate[0][0]']            
                                                                                                  
 dropout_4 (Dropout)             (None, 32)           0           ['dense[0][0]']                  
                                                                                                  
 dense_1 (Dense)                 (None, 16)           528         ['dropout_4[0][0]']              
                                                                                                  
 position_output (Dense)         (None, 2)            34          ['dense_1[0][0]']                
                                                                                                  
==================================================================================================
Total params: 113,490
Trainable params: 113,490
Non-trainable params: 0
__________________________________________________________________________________________________
```

### Output

- Position prediction: [x, y]

### Dead Reckoning Approach

The dual-input architecture is specifically designed for dead reckoning, which means predicting the vessel's position based on previously determined positions combined with new sensor readings:

1. **Position History Branch**: Processes the sequence of previous positions to establish the vessel's trajectory pattern and movement history.

2. **Sensor Branch**: Processes IMU and control input data to understand how the vessel is currently moving and being controlled.

3. **Combined Processing**: The outputs of both branches are combined to predict the next position, leveraging both the vessel's history and current sensor readings.

This approach allows the model to perform true dead reckoning navigation - using sensor data to update a position estimate starting from a known position. The model can be used recursively, feeding back its position predictions to generate a complete trajectory.

### Why This Architecture Works Better for Dead Reckoning

1. **Historical Context**: By including previous positions as explicit inputs, the model learns to relate IMU/control data to position changes rather than absolute positions.

2. **Recursive Prediction**: The model can generate trajectories by feeding its own predictions back into the position history branch.

3. **Initial Position Awareness**: The model takes into account the starting position, making it possible to predict from any arbitrary starting point.

4. **Reduced Error Accumulation**: The model learns to correct its predictions based on both sensor data and position history, potentially reducing error accumulation over time.

## Training Process

The training process includes:

1. **Data Preparation**:
   - Standardization of both input features and target values (zero mean, unit variance)
   - Sequence creation for LSTM input
   - Train-test split (80% training, 20% testing)

2. **Training**:
   - Loss function: Mean Squared Error (MSE)
   - Optimizer: Adam with initial learning rate of 0.001
   - Early stopping to prevent overfitting
   - Learning rate reduction on plateau
   - Model checkpointing to save the best model

3. **Evaluation**:
   - Root Mean Squared Error (RMSE) for position prediction
   - Mean Absolute Error (MAE) for position prediction
   - Visualization of predicted vs. actual trajectories

## Usage

### Data Extraction

```bash
python extract_data.py
```

This will extract data from all ROS2 bags in the `data_from_test` directory and save it to `TheAI/extracted_data`.

### Model Training

```bash
# Train using the smoothed data (default)
python train_model.py

# Train using the raw, unsmoothed data
python train_model.py --use_raw_data

# Specify sequence length
python train_model.py --sequence_length 15
```

This will train the Bi-LSTM model on the extracted data and save the model artifacts to `TheAI/model_output`.

### Making Predictions

```bash
# Basic prediction
python predict.py --input_file path/to/input.csv --output_file path/to/output.csv

# Specify number of prediction steps
python predict.py --input_file path/to/input.csv --output_file path/to/output.csv --steps 100
```

This will load the trained model and make predictions on the input data, saving the results to the output file.

## Requirements

- Python 3.8+
- TensorFlow 2.8+
- NumPy
- Pandas
- Matplotlib
- scikit-learn
- ROS2 libraries (rosbag2-py, rclpy)

## Future Improvements

1. **Hyperparameter Tuning**: Optimize sequence length, model architecture, and learning parameters
2. **Feature Engineering**: Add derived features like acceleration magnitude, jerk, or rate of change of control inputs
3. **Ensemble Methods**: Combine multiple models for improved prediction accuracy
4. **Attention Mechanism**: Add attention layers to focus on the most relevant parts of the input sequence
5. **Online Learning**: Implement online learning for adapting to changing vessel dynamics
6. **Uncertainty Estimation**: Add uncertainty estimates to the predictions 