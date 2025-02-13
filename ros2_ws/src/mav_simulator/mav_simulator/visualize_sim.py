from typing import NoReturn
import numpy as np
from flask import Flask, render_template_string, jsonify
from threading import Thread, Event, Lock
import time
import sys
import os
import traceback

from read_input import read_input
from class_vessel_lat import Vessel

# Global variables to store simulation data
simulation_data = {
    'time': [],
    'position': {'x': [], 'y': [], 'z': []},
    'velocity': {'u': [], 'v': [], 'w': []},
    'orientation': {'phi': [], 'theta': [], 'psi': []},
    'forces': {
        'hydrodynamic': {'x': [], 'y': [], 'z': [], 'k': [], 'm': [], 'n': []},
        'gravitational': {'x': [], 'y': [], 'z': [], 'k': [], 'm': [], 'n': []},
        'control': {'x': [], 'y': [], 'z': [], 'k': [], 'm': [], 'n': []},
        'thruster': {'x': [], 'y': [], 'z': [], 'k': [], 'm': [], 'n': []}
    },
    'controls': {'angles': [], 'rpms': []},
    'simulation_running': True,
    'restart_count': 0  # Add a counter to force frontend refresh
}

# Global control variables
simulation_event = Event()
data_lock = Lock()
current_sim_thread = None

# HTML template with embedded JavaScript
HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <title>Vessel Simulation</title>
    <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
    <style>
        .plot-row { 
            display: flex;
            justify-content: space-between;
            margin: 10px 0;
        }
        .plot { 
            width: 49%; 
            height: 400px; 
        }
        .plot-section {
            margin: 20px 0;
            padding: 10px;
            background: #f5f5f5;
            border-radius: 5px;
        }
        h2 {
            margin: 10px;
            color: #333;
        }
        .control-panel {
            text-align: center;
            margin: 20px 0;
        }
        .restart-btn {
            padding: 10px 20px;
            font-size: 16px;
            background-color: #4CAF50;
            color: white;
            border: none;
            border-radius: 5px;
            cursor: pointer;
            transition: background-color 0.3s;
        }
        .restart-btn:hover {
            background-color: #45a049;
        }
    </style>
</head>
<body>
    <h1 style="text-align: center;">Vessel Simulation Live Data</h1>
    
    <div class="control-panel">
        <button class="restart-btn" onclick="restartSimulation()">Restart Simulation</button>
    </div>
    
    <!-- Position and Velocity Section -->
    <div class="plot-section">
        <h2>Position and Velocity</h2>
        <div class="plot-row">
            <div id="position-plot" class="plot"></div>
            <div id="velocity-plot" class="plot"></div>
        </div>
    </div>

    <!-- Orientation Section -->
    <div class="plot-section">
        <h2>Orientation</h2>
        <div class="plot-row">
            <div id="orientation-plot" class="plot"></div>
        </div>
    </div>

    <!-- Forces Section -->
    <div class="plot-section">
        <h2>Forces</h2>
        <div class="plot-row">
            <div id="hydrodynamic-forces-plot" class="plot"></div>
            <div id="gravitational-forces-plot" class="plot"></div>
        </div>
        <div class="plot-row">
            <div id="control-forces-plot" class="plot"></div>
            <div id="thruster-forces-plot" class="plot"></div>
        </div>
    </div>

    <!-- Controls Section -->
    <div class="plot-section">
        <h2>Controls</h2>
        <div class="plot-row">
            <div id="control-surfaces-plot" class="plot"></div>
            <div id="thruster-rpms-plot" class="plot"></div>
        </div>
    </div>

    <script>
        let updateInterval = null;
        
        function restartSimulation() {
            fetch('/restart')
                .then(response => response.json())
                .then(data => {
                    console.log('Simulation restarted');
                    if (!updateInterval) {
                        updatePlots();
                    }
                });
        }

        function updatePlots() {
            fetch('/data').then(response => response.json()).then(data => {
                // Check if simulation has been restarted
                if (lastRestartCount !== -1 && lastRestartCount !== data.restart_count) {
                    // Clear all plots
                    document.querySelectorAll('.plot').forEach(plot => {
                        Plotly.purge(plot);
                    });
                }
                lastRestartCount = data.restart_count;

                const layout = {
                    xaxis: { title: 'Time (s)' },
                    yaxis: { title: '' }
                };

                // Position plot
                const positionData = [{
                    x: data.time,
                    y: data.position.x,
                    name: 'X',
                    type: 'scatter'
                }, {
                    x: data.time,
                    y: data.position.y,
                    name: 'Y',
                    type: 'scatter'
                }, {
                    x: data.time,
                    y: data.position.z,
                    name: 'Z',
                    type: 'scatter'
                }];
                layout.yaxis.title = 'Position (m)';
                Plotly.newPlot('position-plot', positionData, {...layout, title: 'Position vs Time'});

                // Velocity plot
                const velocityData = [{
                    x: data.time,
                    y: data.velocity.u,
                    name: 'u',
                    type: 'scatter'
                }, {
                    x: data.time,
                    y: data.velocity.v,
                    name: 'v',
                    type: 'scatter'
                }, {
                    x: data.time,
                    y: data.velocity.w,
                    name: 'w',
                    type: 'scatter'
                }];
                layout.yaxis.title = 'Velocity (m/s)';
                Plotly.newPlot('velocity-plot', velocityData, {...layout, title: 'Velocity vs Time'});

                // Orientation plot
                const orientationData = [{
                    x: data.time,
                    y: data.orientation.phi,
                    name: 'Roll (φ)',
                    type: 'scatter'
                }, {
                    x: data.time,
                    y: data.orientation.theta,
                    name: 'Pitch (θ)',
                    type: 'scatter'
                }, {
                    x: data.time,
                    y: data.orientation.psi,
                    name: 'Yaw (ψ)',
                    type: 'scatter'
                }];
                layout.yaxis.title = 'Angle (deg)';
                Plotly.newPlot('orientation-plot', orientationData, {...layout, title: 'Orientation vs Time'});

                // Forces plots
                const forceTypes = ['hydrodynamic', 'gravitational', 'control', 'thruster'];
                const components = ['x', 'y', 'z', 'k', 'm', 'n'];
                
                forceTypes.forEach(forceType => {
                    const forceData = components.map(component => ({
                        x: data.time,
                        y: data.forces[forceType][component],
                        name: component.toUpperCase(),
                        type: 'scatter'
                    }));
                    
                    layout.yaxis.title = 'Force (N) / Moment (Nm)';
                    Plotly.newPlot(`${forceType}-forces-plot`, forceData, {
                        ...layout,
                        title: `${forceType.charAt(0).toUpperCase() + forceType.slice(1)} Forces vs Time`
                    });
                });

                // Control surface angles plot
                if (data.controls.angles.length > 0) {
                    const controlSurfaceData = [];
                    for (let i = 0; i < data.controls.angles[0].length; i++) {
                        controlSurfaceData.push({
                            x: data.time,
                            y: data.controls.angles.map(angles => angles[i]),
                            name: `Surface ${i+1}`,
                            type: 'scatter'
                        });
                    }
                    layout.yaxis.title = 'Angle (deg)';
                    Plotly.newPlot('control-surfaces-plot', controlSurfaceData, {
                        ...layout,
                        title: 'Control Surface Angles vs Time'
                    });
                }

                // Thruster RPMs plot
                if (data.controls.rpms.length > 0) {
                    const thrusterData = [];
                    for (let i = 0; i < data.controls.rpms[0].length; i++) {
                        thrusterData.push({
                            x: data.time,
                            y: data.controls.rpms.map(rpms => rpms[i]),
                            name: `Thruster ${i+1}`,
                            type: 'scatter'
                        });
                    }
                    layout.yaxis.title = 'RPM';
                    Plotly.newPlot('thruster-rpms-plot', thrusterData, {
                        ...layout,
                        title: 'Thruster RPMs vs Time'
                    });
                }

                if (data.simulation_running) {
                    if (!updateInterval) {
                        updateInterval = setTimeout(updatePlots, 500);  // Update every 500ms
                    }
                } else {
                    clearTimeout(updateInterval);
                    updateInterval = null;
                }
            });
        }
        updatePlots();
    </script>
</body>
</html>
'''

def clear_simulation_data():
    """Clear all simulation data while thread-safe"""
    with data_lock:
        for key in simulation_data:
            if key == 'restart_count':
                simulation_data[key] += 1  # Increment restart counter
            elif isinstance(simulation_data[key], dict):
                for subkey in simulation_data[key]:
                    if isinstance(simulation_data[key][subkey], dict):
                        for subsubkey in simulation_data[key][subkey]:
                            simulation_data[key][subkey][subsubkey] = []
                    else:
                        simulation_data[key][subkey] = []
            else:
                simulation_data[key] = [] if key != 'simulation_running' else True

def run_simulation():
    """Run vessel simulation and update global simulation_data"""
    try:
        # Clear previous simulation data
        clear_simulation_data()
        
        print("Starting new simulation...")

        # Get the current directory
        current_dir = os.path.dirname(os.path.abspath(__file__))
        # Go up to the workspace root and construct the input path
        workspace_root = os.path.abspath(os.path.join(current_dir, '..', '..', '..', '..'))
        input_path = os.path.join(workspace_root, 'inputs', 'mavymini', 'simulation_input.yml')
        
        # Read input files and create vessel
        sim_params, vessel_configs, hydrodynamic_data = read_input(input_path)
        vessel = Vessel(vessel_configs[0], hydrodynamic_data[0])
        
        # Run simulation
        vessel.reset()
        last_update_time = time.time()
        update_interval = 0.02  # 50Hz update rate
        
        print(f"Simulation will run for {vessel.Tmax} seconds")
        
        while vessel.t < vessel.Tmax and simulation_event.is_set():
            current_time = time.time()
            if current_time - last_update_time >= update_interval:
                # Step simulation
                vessel.step()
                
                # Extract data
                pos = vessel.current_state[6:9]
                vel = vessel.current_state[0:3]
                angles = vessel.current_state[9:12]
                
                # Calculate forces
                F_hyd = vessel.hydrodynamic_forces(vessel.current_state[0:6])
                F_g = vessel.gravitational_forces(angles[0], angles[1])
                
                # Get control surface and thruster forces
                control_start = 12
                thruster_start = control_start + len(vessel.control_surfaces['control_surfaces']) if hasattr(vessel, 'control_surfaces') else control_start
                
                control_angles = vessel.current_state[control_start:thruster_start] if hasattr(vessel, 'control_surfaces') else []
                thruster_rpms = vessel.current_state[thruster_start:] if hasattr(vessel, 'thrusters') else []
                
                F_control = vessel.control_forces(control_angles) if hasattr(vessel, 'control_surfaces') else np.zeros(6)
                F_thrust = vessel.thruster_forces(thruster_rpms) if hasattr(vessel, 'thrusters') else np.zeros(6)
                
                # Update simulation data thread-safe
                with data_lock:
                    # Update time, position, velocity, orientation
                    simulation_data['time'].append(float(vessel.t))
                    simulation_data['position']['x'].append(float(pos[0]))
                    simulation_data['position']['y'].append(float(pos[1]))
                    simulation_data['position']['z'].append(float(pos[2]))
                    simulation_data['velocity']['u'].append(float(vel[0]))
                    simulation_data['velocity']['v'].append(float(vel[1]))
                    simulation_data['velocity']['w'].append(float(vel[2]))
                    simulation_data['orientation']['phi'].append(float(np.rad2deg(angles[0])))
                    simulation_data['orientation']['theta'].append(float(np.rad2deg(angles[1])))
                    simulation_data['orientation']['psi'].append(float(np.rad2deg(angles[2])))
                    
                    # Update forces
                    force_mapping = {
                        'hydrodynamic': F_hyd,
                        'gravitational': F_g,
                        'control': F_control,
                        'thruster': F_thrust
                    }
                    
                    components = ['x', 'y', 'z', 'k', 'm', 'n']
                    for force_type, force in force_mapping.items():
                        for j, component in enumerate(components):
                            simulation_data['forces'][force_type][component].append(float(force[j]))
                    
                    # Update controls
                    simulation_data['controls']['angles'].append([float(np.rad2deg(angle)) for angle in control_angles])
                    simulation_data['controls']['rpms'].append([float(rpm) for rpm in thruster_rpms])
                
                last_update_time = current_time
                
                if vessel.t % 1 < update_interval:  # Print every second
                    print(f"Simulation time: {vessel.t:.2f} / {vessel.Tmax:.2f}")
            else:
                time.sleep(0.001)  # Small sleep to prevent CPU hogging
                
        print("Simulation completed normally")
    except Exception as e:
        print(f"Simulation error: {str(e)}")
        print(traceback.format_exc())
    finally:
        with data_lock:
            simulation_data['simulation_running'] = False
        print("Simulation thread ending")

def create_flask_app():
    """Create and configure the Flask application"""
    app = Flask(__name__)
    
    @app.route('/')
    def home():
        return render_template_string(HTML_TEMPLATE)
    
    @app.route('/data')
    def get_data():
        with data_lock:
            return jsonify(simulation_data)
    
    @app.route('/restart')
    def restart_simulation():
        global simulation_event, current_sim_thread
        
        print("Restart requested")
        
        # Stop current simulation if running
        simulation_event.clear()
        if current_sim_thread and current_sim_thread.is_alive():
            print("Waiting for current simulation to stop...")
            current_sim_thread.join(timeout=2.0)  # Wait for current simulation to stop
            
        # Start new simulation
        print("Starting new simulation thread...")
        simulation_event.set()
        current_sim_thread = Thread(target=run_simulation)
        current_sim_thread.daemon = True
        current_sim_thread.start()
        
        return jsonify({"status": "restarted"})
    
    return app

def main():
    global simulation_event, current_sim_thread
    
    # Initialize control variables
    simulation_event.set()
    
    # Start initial simulation in a daemon thread
    current_sim_thread = Thread(target=run_simulation)
    current_sim_thread.daemon = True
    current_sim_thread.start()
    
    # Create and run the Flask app
    app = create_flask_app()
    app.run(debug=False, host='0.0.0.0', port=5000)

if __name__ == '__main__':
    main() 