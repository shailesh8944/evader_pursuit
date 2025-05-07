import numpy as np
import pandas as pd
import tensorflow as tf
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import os

class DeadReckoningModel:
    """
    A model for dead reckoning navigation using IMU data.
    This model predicts x,y position and x,y velocity given previous positions and IMU data.
    """
    
    def __init__(self, 
                 window_size=5, 
                 hidden_layers=[64, 32],
                 learning_rate=0.001,
                 model_path='dead_reckoning_model'):
        """
        Initialize the model.
        
        Args:
            window_size: Number of time steps to use as input context
            hidden_layers: List of neurons in each hidden layer
            learning_rate: Learning rate for the optimizer
            model_path: Path to save the model
        """
        self.window_size = window_size
        self.hidden_layers = hidden_layers
        self.learning_rate = learning_rate
        self.model_path = model_path
        self.model = None
        self.x_scaler = StandardScaler()
        self.y_scaler = StandardScaler()
        
    def _create_sequences(self, data, window_size):
        """
        Create sequences from the data for time-series prediction.
        
        Args:
            data: DataFrame containing the data
            window_size: Number of time steps to use as input
            
        Returns:
            X: Input sequences
            y: Target values
        """
        X = []
        y = []
        
        for i in range(len(data) - window_size):
            # Input features: previous positions, IMU data (accelerations, angular velocities)
            seq = data.iloc[i:i+window_size]
            
            # Extract relevant features
            features = []
            for j in range(window_size):
                # Previous positions
                pos_x = seq.iloc[j]['position_x']
                pos_y = seq.iloc[j]['position_y']
                
                # IMU data (accelerations and angular velocities relevant to x,y)
                acc_x = seq.iloc[j]['linear_acc_x']
                acc_y = seq.iloc[j]['linear_acc_y']
                ang_z = seq.iloc[j]['angular_vel_z']  # Only use angular velocity around z axis (yaw) for 2D navigation
                
                # Velocity data
                vel_x = seq.iloc[j]['linear_vel_x']
                vel_y = seq.iloc[j]['linear_vel_y']
                
                # Combine all features
                time_step_features = [pos_x, pos_y, acc_x, acc_y, ang_z, vel_x, vel_y]
                features.append(time_step_features)
            
            X.append(np.array(features).flatten())
            
            # Target: next position and velocity
            next_pos_x = data.iloc[i+window_size]['position_x']
            next_pos_y = data.iloc[i+window_size]['position_y']
            next_vel_x = data.iloc[i+window_size]['linear_vel_x']
            next_vel_y = data.iloc[i+window_size]['linear_vel_y']
            
            y.append([next_pos_x, next_pos_y, next_vel_x, next_vel_y])
        
        return np.array(X), np.array(y)
    
    def build_model(self, input_shape):
        """
        Build the Feedforward Neural Network model.
        
        Args:
            input_shape: Shape of the input data
            
        Returns:
            Compiled TensorFlow model
        """
        model = tf.keras.Sequential()
        
        # Input layer
        model.add(tf.keras.layers.Dense(self.hidden_layers[0], 
                                         activation='relu', 
                                         input_shape=(input_shape,)))
        
        # Hidden layers
        for units in self.hidden_layers[1:]:
            model.add(tf.keras.layers.Dense(units, activation='relu'))
            model.add(tf.keras.layers.Dropout(0.2))  # Add dropout to prevent overfitting
            
        # Output layer: x,y position and x,y velocity
        model.add(tf.keras.layers.Dense(4))
        
        # Compile model
        model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=self.learning_rate),
                      loss='mse',
                      metrics=['mae'])
        
        return model
    
    def prepare_data(self, data_files):
        """
        Prepare data from multiple CSV files.
        
        Args:
            data_files: List of CSV file paths
            
        Returns:
            X_train, X_val, y_train, y_val: Prepared training and validation data
        """
        all_data = []
        
        # Load and combine all data files
        for file in data_files:
            df = pd.read_csv(file)
            all_data.append(df)
            
        data = pd.concat(all_data, ignore_index=True)
        
        # Create sequences
        X, y = self._create_sequences(data, self.window_size)
        
        # Split data into training and validation sets
        X_train, X_val, y_train, y_val = train_test_split(X, y, test_size=0.2, random_state=42)
        
        # Normalize the data
        X_train = self.x_scaler.fit_transform(X_train)
        X_val = self.x_scaler.transform(X_val)
        
        y_train = self.y_scaler.fit_transform(y_train)
        y_val = self.y_scaler.transform(y_val)
        
        return X_train, X_val, y_train, y_val
    
    def train(self, X_train, y_train, X_val, y_val, epochs=50, batch_size=32):
        """
        Train the model.
        
        Args:
            X_train, y_train: Training data
            X_val, y_val: Validation data
            epochs: Number of training epochs
            batch_size: Batch size for training
            
        Returns:
            Training history
        """
        if self.model is None:
            self.model = self.build_model(X_train.shape[1])
            
        # Create callbacks
        early_stopping = tf.keras.callbacks.EarlyStopping(
            monitor='val_loss',
            patience=10,
            restore_best_weights=True
        )
        
        # Train the model
        history = self.model.fit(
            X_train, y_train,
            validation_data=(X_val, y_val),
            epochs=epochs,
            batch_size=batch_size,
            callbacks=[early_stopping],
            verbose=1
        )
        
        # Save the model
        self.save_model()
        
        return history
    
    def save_model(self):
        """
        Save the model and scalers.
        """
        if self.model is not None:
            # Create directory if it doesn't exist
            os.makedirs(self.model_path, exist_ok=True)
            
            # Save TensorFlow model
            self.model.save(os.path.join(self.model_path, 'model'))
            
            # Save scalers
            np.save(os.path.join(self.model_path, 'x_scaler_mean.npy'), self.x_scaler.mean_)
            np.save(os.path.join(self.model_path, 'x_scaler_scale.npy'), self.x_scaler.scale_)
            np.save(os.path.join(self.model_path, 'y_scaler_mean.npy'), self.y_scaler.mean_)
            np.save(os.path.join(self.model_path, 'y_scaler_scale.npy'), self.y_scaler.scale_)
            
    def load_model(self):
        """
        Load the model and scalers.
        """
        # Load TensorFlow model
        self.model = tf.keras.models.load_model(os.path.join(self.model_path, 'model'))
        
        # Load scalers
        x_mean = np.load(os.path.join(self.model_path, 'x_scaler_mean.npy'))
        x_scale = np.load(os.path.join(self.model_path, 'x_scaler_scale.npy'))
        y_mean = np.load(os.path.join(self.model_path, 'y_scaler_mean.npy'))
        y_scale = np.load(os.path.join(self.model_path, 'y_scaler_scale.npy'))
        
        # Recreate scalers
        self.x_scaler = StandardScaler()
        self.x_scaler.mean_ = x_mean
        self.x_scaler.scale_ = x_scale
        
        self.y_scaler = StandardScaler()
        self.y_scaler.mean_ = y_mean
        self.y_scaler.scale_ = y_scale
    
    def predict(self, data_sequence):
        """
        Predict position and velocity from a sequence of data.
        
        Args:
            data_sequence: DataFrame containing window_size rows of data
            
        Returns:
            Predicted position_x, position_y, velocity_x, velocity_y
        """
        if self.model is None:
            raise ValueError("Model not loaded or trained yet.")
        
        if len(data_sequence) != self.window_size:
            raise ValueError(f"Data sequence must have exactly {self.window_size} time steps.")
        
        # Extract features similar to training
        features = []
        for i in range(self.window_size):
            pos_x = data_sequence.iloc[i]['position_x']
            pos_y = data_sequence.iloc[i]['position_y']
            acc_x = data_sequence.iloc[i]['linear_acc_x']
            acc_y = data_sequence.iloc[i]['linear_acc_y']
            ang_z = data_sequence.iloc[i]['angular_vel_z']
            vel_x = data_sequence.iloc[i]['linear_vel_x']
            vel_y = data_sequence.iloc[i]['linear_vel_y']
            
            time_step_features = [pos_x, pos_y, acc_x, acc_y, ang_z, vel_x, vel_y]
            features.append(time_step_features)
        
        X = np.array(features).flatten().reshape(1, -1)
        
        # Normalize
        X = self.x_scaler.transform(X)
        
        # Predict
        y_pred = self.model.predict(X)
        
        # Denormalize predictions
        y_pred = self.y_scaler.inverse_transform(y_pred)
        
        return y_pred[0]  # Returns [position_x, position_y, velocity_x, velocity_y]
    
    def predict_trajectory(self, initial_sequence, num_steps=100, ground_truth=None):
        """
        Predict a trajectory by recursively predicting each next step.
        
        Args:
            initial_sequence: Initial DataFrame with window_size rows of data
            num_steps: Number of steps to predict
            ground_truth: Optional DataFrame with ground truth for comparison
            
        Returns:
            Predicted trajectory as a DataFrame
        """
        if self.model is None:
            raise ValueError("Model not loaded or trained yet.")
        
        # Initialize with the initial sequence
        sequence = initial_sequence.copy()
        
        # Create arrays to store predictions
        pred_positions_x = []
        pred_positions_y = []
        pred_velocities_x = []
        pred_velocities_y = []
        
        # Store ground truth if available
        true_positions_x = []
        true_positions_y = []
        
        if ground_truth is not None:
            for i in range(len(initial_sequence), len(initial_sequence) + num_steps):
                if i < len(ground_truth):
                    true_positions_x.append(ground_truth.iloc[i]['position_x'])
                    true_positions_y.append(ground_truth.iloc[i]['position_y'])
        
        # Predict trajectory
        for i in range(num_steps):
            # Make prediction for next step
            pred = self.predict(sequence)
            
            pred_pos_x, pred_pos_y, pred_vel_x, pred_vel_y = pred
            
            # Store predictions
            pred_positions_x.append(pred_pos_x)
            pred_positions_y.append(pred_pos_y)
            pred_velocities_x.append(pred_vel_x)
            pred_velocities_y.append(pred_vel_y)
            
            # Create a new row for the predicted data
            new_row = sequence.iloc[-1].copy()
            new_row['position_x'] = pred_pos_x
            new_row['position_y'] = pred_pos_y
            new_row['linear_vel_x'] = pred_vel_x
            new_row['linear_vel_y'] = pred_vel_y
            
            # Update the sequence by removing the first row and adding the new row
            sequence = sequence.iloc[1:].append(new_row, ignore_index=True)
        
        # Plot results if ground truth available
        if ground_truth is not None and len(true_positions_x) > 0:
            plt.figure(figsize=(10, 8))
            
            # Plot ground truth
            plt.plot(true_positions_x, true_positions_y, 'b-', label='Ground Truth')
            
            # Plot predictions
            plt.plot(pred_positions_x, pred_positions_y, 'r-', label='Predictions')
            
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
            plt.savefig(os.path.join(self.model_path, 'trajectory_prediction.png'))
            plt.close()
        
        # Return as DataFrame
        results = pd.DataFrame({
            'position_x': pred_positions_x,
            'position_y': pred_positions_y,
            'velocity_x': pred_velocities_x,
            'velocity_y': pred_velocities_y
        })
        
        return results

    def evaluate_error(self, X_test, y_test):
        """
        Evaluate the model on test data.
        
        Args:
            X_test: Test input data
            y_test: True test outputs
            
        Returns:
            Dictionary of evaluation metrics
        """
        if self.model is None:
            raise ValueError("Model not loaded or trained yet.")
        
        # Predict
        y_pred = self.model.predict(X_test)
        
        # Denormalize
        y_test = self.y_scaler.inverse_transform(y_test)
        y_pred = self.y_scaler.inverse_transform(y_pred)
        
        # Calculate errors
        pos_x_mae = np.mean(np.abs(y_test[:, 0] - y_pred[:, 0]))
        pos_y_mae = np.mean(np.abs(y_test[:, 1] - y_pred[:, 1]))
        vel_x_mae = np.mean(np.abs(y_test[:, 2] - y_pred[:, 2]))
        vel_y_mae = np.mean(np.abs(y_test[:, 3] - y_pred[:, 3]))
        
        # Calculate RMSE (Root Mean Squared Error)
        pos_x_rmse = np.sqrt(np.mean((y_test[:, 0] - y_pred[:, 0])**2))
        pos_y_rmse = np.sqrt(np.mean((y_test[:, 1] - y_pred[:, 1])**2))
        vel_x_rmse = np.sqrt(np.mean((y_test[:, 2] - y_pred[:, 2])**2))
        vel_y_rmse = np.sqrt(np.mean((y_test[:, 3] - y_pred[:, 3])**2))
        
        # Total position error
        pos_mae = np.mean(np.sqrt((y_test[:, 0] - y_pred[:, 0])**2 + (y_test[:, 1] - y_pred[:, 1])**2))
        
        metrics = {
            'position_x_mae': pos_x_mae,
            'position_y_mae': pos_y_mae,
            'velocity_x_mae': vel_x_mae,
            'velocity_y_mae': vel_y_mae,
            'position_x_rmse': pos_x_rmse,
            'position_y_rmse': pos_y_rmse,
            'velocity_x_rmse': vel_x_rmse,
            'velocity_y_rmse': vel_y_rmse,
            'position_total_mae': pos_mae
        }
        
        return metrics
        
    def plot_training_history(self, history):
        """
        Plot training and validation loss.
        
        Args:
            history: Training history from model.fit()
        """
        plt.figure(figsize=(12, 4))
        
        # Plot loss
        plt.subplot(1, 2, 1)
        plt.plot(history.history['loss'], label='Training Loss')
        plt.plot(history.history['val_loss'], label='Validation Loss')
        plt.ylabel('Loss (MSE)')
        plt.xlabel('Epoch')
        plt.legend()
        plt.title('Training and Validation Loss')
        
        # Plot MAE
        plt.subplot(1, 2, 2)
        plt.plot(history.history['mae'], label='Training MAE')
        plt.plot(history.history['val_mae'], label='Validation MAE')
        plt.ylabel('MAE')
        plt.xlabel('Epoch')
        plt.legend()
        plt.title('Training and Validation MAE')
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.model_path, 'training_history.png'))
        plt.close() 