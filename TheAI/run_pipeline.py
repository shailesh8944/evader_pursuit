#!/usr/bin/env python3
import os
import argparse
import subprocess
import time

def run_command(command, description):
    """Run a command and print its output"""
    print(f"\n{'=' * 80}")
    print(f"Running: {description}")
    print(f"{'=' * 80}")
    
    start_time = time.time()
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
        shell=True
    )
    
    # Print output in real-time
    for line in process.stdout:
        print(line, end='')
    
    process.wait()
    end_time = time.time()
    
    if process.returncode != 0:
        print(f"\n❌ {description} failed with return code {process.returncode}")
        return False
    else:
        print(f"\n✅ {description} completed successfully in {end_time - start_time:.2f} seconds")
        return True

def main():
    parser = argparse.ArgumentParser(description='Run the complete vessel position predictor pipeline')
    parser.add_argument('--skip_extraction', action='store_true', help='Skip data extraction step')
    parser.add_argument('--skip_training', action='store_true', help='Skip model training step')
    parser.add_argument('--bag_dir', type=str, default=None, help='Directory containing ROS2 bags (default: data_from_test)')
    parser.add_argument('--sequence_length', type=int, default=10, help='Sequence length for LSTM input (default: 10)')
    parser.add_argument('--use_raw_data', action='store_true', help='Use raw data instead of smoothed data for training')
    parser.add_argument('--min_distance', type=float, default=0.3, help='Minimum distance between position points in meters (default: 0.3)')
    parser.add_argument('--position_delay', type=int, default=1, help='How many steps in the future to predict (default: 1)')
    parser.add_argument('--prediction_steps', type=int, default=None, help='Number of steps to predict in the evaluation phase')
    args = parser.parse_args()
    
    # Set paths
    current_dir = os.path.dirname(os.path.abspath(__file__))
    root_dir = os.path.dirname(current_dir)
    
    # Default bag directory
    if args.bag_dir is None:
        args.bag_dir = os.path.join(root_dir, 'data_from_test')
    
    # Make paths absolute
    args.bag_dir = os.path.abspath(args.bag_dir)
    
    # Create output directories
    extracted_data_dir = os.path.join(current_dir, 'extracted_data')
    model_output_dir = os.path.join(current_dir, 'model_output')
    
    os.makedirs(extracted_data_dir, exist_ok=True)
    os.makedirs(model_output_dir, exist_ok=True)
    
    # Step 1: Extract data
    if not args.skip_extraction:
        extract_cmd = f"python3 {os.path.join(current_dir, 'extract_data.py')} --min_distance {args.min_distance}"
        if not run_command(extract_cmd, "Data Extraction"):
            return
    else:
        print("Skipping data extraction...")
    
    # Step 2: Train model
    if not args.skip_training:
        train_cmd = f"python3 {os.path.join(current_dir, 'train_model.py')} --sequence_length {args.sequence_length} --position_delay {args.position_delay}"
        
        if args.use_raw_data:
            train_cmd += " --use_raw_data"
            
        if not run_command(train_cmd, "Model Training"):
            return
    else:
        print("Skipping model training...")
    
    # Step 3: Test model on a sample
    # Find a sample combined file
    file_pattern = "*_combined.csv" if args.use_raw_data else "*_smoothed_combined.csv"
    combined_files = [f for f in os.listdir(extracted_data_dir) if f.endswith(file_pattern)]
    
    if not combined_files and not args.use_raw_data:
        print("No smoothed combined files found. Falling back to regular combined files.")
        combined_files = [f for f in os.listdir(extracted_data_dir) if f.endswith("_combined.csv")]
    
    if combined_files:
        sample_file = os.path.join(extracted_data_dir, combined_files[0])
        output_file = os.path.join(model_output_dir, 'sample_predictions.csv')
        
        predict_cmd = f"python3 {os.path.join(current_dir, 'predict.py')} --input_file {sample_file} --output_file {output_file}"
        
        if args.prediction_steps is not None:
            predict_cmd += f" --steps {args.prediction_steps}"
            
        run_command(predict_cmd, "Sample Prediction")
    
    print("\n" + "=" * 80)
    print("Pipeline completed!")
    print("=" * 80)
    print(f"Extracted data saved to: {extracted_data_dir}")
    print(f"Model artifacts saved to: {model_output_dir}")
    
    data_type = "raw" if args.use_raw_data else "smoothed"
    print(f"\nModel trained on {data_type} data with sequence length {args.sequence_length} and position delay {args.position_delay}")
    
    if os.path.exists(os.path.join(model_output_dir, 'trajectory_segments.png')):
        print(f"Sample trajectory plot saved to: {os.path.join(model_output_dir, 'trajectory_segments.png')}")
    
    print("\nTo make predictions on new data:")
    print(f"python3 {os.path.join(current_dir, 'predict.py')} --input_file path/to/input.csv --output_file path/to/output.csv")

if __name__ == '__main__':
    main() 