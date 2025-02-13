from flask import request, jsonify, send_from_directory, url_for
from pathlib import Path
from web.js.calculate_hydrodynamics import main as calculate_hydro

@app.route('/calculate_hydrodynamics', methods=['POST'])
def calculate_hydrodynamics():
    try:
        # Get vessel data file from request
        if 'vessel_data' not in request.files:
            return jsonify({'error': 'No vessel data file provided'}), 400
        
        vessel_data_file = request.files['vessel_data']
        
        # Save vessel data temporarily
        temp_dir = Path('temp')
        temp_dir.mkdir(exist_ok=True)
        
        vessel_data_path = temp_dir / 'vessel_data.json'
        vessel_data_file.save(vessel_data_path)
        
        # Calculate hydrodynamics
        result = calculate_hydro(str(vessel_data_path))
        
        # Save result to hydrodynamics.yml
        output_path = temp_dir / 'hydrodynamics.yml'
        
        # Return file URL for download
        return jsonify({
            'success': True,
            'file_url': url_for('download_file', filename='hydrodynamics.yml')
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/download/<filename>')
def download_file(filename):
    return send_from_directory('temp', filename, as_attachment=True) 