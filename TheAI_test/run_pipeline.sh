#!/bin/bash

# Marine Vessel Position Prediction Pipeline
# This script runs the entire pipeline from data extraction to trajectory prediction

# Set up directories
DATA_DIR="../data_from_test"
EXTRACT_DIR="./extracted_data"
MODEL_DIR="./model"
RESULTS_DIR="./results"
TRAJECTORY_DIR="./trajectory_results"

# Physics parameters
MAX_SPEED=0.5  # Maximum speed in m/s
MAX_TURN_RATE=30.0  # Maximum turn rate in degrees/s

# ML parameters
BATCH_SIZE=64   # Larger batch size for better training
EPOCHS=300      # More epochs for better convergence

# Clean up old data and models to start fresh
echo "========================================================"
echo "Cleaning up old data and models"
echo "========================================================"
rm -rf $MODEL_DIR/* $RESULTS_DIR/* $TRAJECTORY_DIR/*

# Create necessary directories
mkdir -p $EXTRACT_DIR $MODEL_DIR $RESULTS_DIR $TRAJECTORY_DIR

# Step 1: Extract data from ROS bags
echo "========================================================"
echo "Step 1: Extracting data from ROS bags with physics-based filtering"
echo "========================================================"
python extract_data.py --base_dir $DATA_DIR --output_dir $EXTRACT_DIR --max_speed $MAX_SPEED --max_turn_rate $MAX_TURN_RATE

# Check if extraction was successful
if [ $? -ne 0 ]; then
    echo "Error: Data extraction failed. Exiting."
    exit 1
fi

# Find a suitable test file for trajectory prediction
TEST_FILE=$(ls $EXTRACT_DIR/*_combined.csv | head -n 1)
if [ -z "$TEST_FILE" ]; then
    echo "Error: No combined CSV files found for testing. Exiting."
    exit 1
fi
echo "Selected test file: $TEST_FILE"

# Step 2: Train the position prediction model
echo "========================================================"
echo "Step 2: Training enhanced neural network model"
echo "========================================================"
python vessel_predictor.py --data_dir $EXTRACT_DIR --model_dir $MODEL_DIR --results_dir $RESULTS_DIR --epochs $EPOCHS --batch_size $BATCH_SIZE

# Check if training was successful
if [ $? -ne 0 ]; then
    echo "Error: Model training failed. Exiting."
    exit 1
fi

# Step 3: Predict and evaluate trajectory
echo "========================================================"
echo "Step 3: Running trajectory prediction with improved model"
echo "========================================================"
# Predict full trajectory
python trajectory_predictor.py --model_dir $MODEL_DIR --test_file $TEST_FILE --results_dir $TRAJECTORY_DIR

# Check if prediction was successful
if [ $? -ne 0 ]; then
    echo "Error: Trajectory prediction failed."
    exit 1
fi

echo "========================================================"
echo "Pipeline completed successfully!"
echo "========================================================"
echo "Results saved to:"
echo "- Model: $MODEL_DIR"
echo "- Evaluation results: $RESULTS_DIR"
echo "- Trajectory results: $TRAJECTORY_DIR"
echo "========================================================" 