from typing import Dict, Tuple, List
import numpy as np
import yaml
import os
from pathlib import Path

def load_yaml(file_path: str) -> Dict:
    """Load a YAML file and return its contents.
    
    Args:
        file_path: Path to the YAML file
        
    Returns:
        Dict containing the YAML contents
    """
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def resolve_path(base_path: str, file_path: str, name: str) -> str:
    """Resolve a file path, replacing {name} with the actual vessel name.
    
    Args:
        base_path: Base directory path
        file_path: File path from YAML
        name: Vessel name to substitute
        
    Returns:
        Resolved absolute path
    """
    resolved = file_path.replace('{name}', name)
    if not os.path.isabs(resolved):
        resolved = os.path.join(base_path, resolved)
    return resolved

def read_input(input_file: str = None) -> Tuple[Dict, List[Dict]]:
    """Read vessel parameters and configuration from input files.
    
    Args:
        input_file: Path to the main simulation input file. If None, uses default.
        
    Returns:
        Tuple containing:
            sim_params (Dict): Global simulation parameters
            vessel_configs (List[Dict]): List of vessel configurations
    """    
    base_path = str(Path(input_file).parent.parent)
    sim_config = load_yaml(input_file)
    
    # Extract global simulation parameters
    sim_params = {
        'sim_time': sim_config.get('sim_time', 100),
        'time_step': sim_config.get('time_step', 0.01),
        'density': sim_config.get('density', 1000),
        'gravity': sim_config.get('gravity', 9.80665),
        'world_size': sim_config.get('world_size', [100, 100, float('inf')]),
        'gps_datum': sim_config.get('gps_datum', [0, 0, 0]),
        'nagents': sim_config.get('nagents', 1)
    }
    
    # Process each vessel's configuration
    vessel_configs = []
    hydrodynamic_data = []
    for agent in sim_config.get('agents', []):
        vessel_name = agent['name']
        vessel_config = {
            'name': vessel_name,
            'type': agent['type']
        }
        
        # Load all referenced configuration files
        for config_type in ['geometry', 'inertia', 'hydrodynamics', 'control_surfaces', 
                          'initial_conditions', 'sensors', 'guidance', 'control', 'thrusters']:
            if config_type in agent:
                file_path = resolve_path(base_path, agent[config_type], vessel_name)
                vessel_config[config_type] = load_yaml(file_path)
        
        # # Process initial conditions
        # if 'initial_conditions' in vessel_config:
        #     ic = vessel_config['initial_conditions']
        #     vessel_config.update({
        #         'initial_velocity': np.array(ic.get('start_velocity', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])),
        #         'initial_position': np.array(ic.get('start_location', [0.0, 0.0, 0.0])),
        #         'initial_attitude': np.array(ic.get('start_attitude', [0.0, 0.0, 0.0])),
        #         'use_quaternion': ic.get('use_quaternion', False)
        #     })

        # # Process control parameters
        # if 'control' in vessel_config:
        #     ctrl = vessel_config['control']
        #     vessel_config.update({
        #         'control_surface_control_type': ctrl.get('control_surface_control_type', 'fixed_rudder'),
        #         'thruster_control_type': ctrl.get('thruster_control_type', 'fixed_rpm'),
        #         'delta_max': np.deg2rad(ctrl.get('delta_max', 35.0)),
        #         'deltad_max': np.deg2rad(ctrl.get('deltad_max', 1.0)),  # Convert to radians
        #     })

        # # Process geometry parameters
        # if 'geometry' in vessel_config:
        #     geom = vessel_config['geometry']
        #     vessel_config.update({
        #         'L': geom.get('L', 1.0),  # Length
        #         'B': geom.get('B', 1.0),  # Beam
        #         'D': geom.get('D', 1.0)   # Draft
        #     })

        # # Process inertia parameters
        # if 'inertia' in vessel_config:
        #     inertia = vessel_config['inertia']
        #     vessel_config.update({
        #         'mass': inertia.get('mass', 1.0),
        #         'co': np.array(inertia.get('CO', [0.0, 0.0, 0.0])),
        #         'cg': np.array(inertia.get('CG', [0.0, 0.0, 0.0])),
        #         'cb': np.array(inertia.get('CB', [0.0, 0.0, 0.0])),
        #         'gyration': np.array(inertia.get('gyration', [1.0, 1.0, 1.0])),
        #         'buoyancy_mass': inertia.get('buoyancy_mass', 1.0)
        #     })

        # Process thruster parameters
        # if 'thrusters' in vessel_config:
        #     thrusters = vessel_config['thrusters']
            # vessel_config['thrusters'] = []
            # for thruster in thrusters:
            #     vessel_config['thrusters'].append({
            #         'thruster_name': thruster.get('thruster_name'),
            #         'thruster_id': thruster.get('thruster_id'),
            #         'thruster_location': thruster.get('thruster_location', [0.0, 0.0, 0.0]),
            #         'thruster_orientation': thruster.get('thruster_orientation', [0.0, 0.0, 0.0]),
            #         'T_prop': thruster.get('T_prop', 1.0),
            #         'D_prop': thruster.get('D_prop', 0.1),
            #         'tp': thruster.get('tp', 0.0),
            #         'J_vs_KT': thruster.get('J_vs_KT')
            #     })

        # Process control surface parameters
        # if 'control_surfaces' in vessel_config:
        #     cs = vessel_config['control_surfaces']
        #     vessel_config['control_surfaces'] = cs.get('control_surfaces', [])
        #     # Add NACA airfoil data file path
        #     vessel_config['naca_file'] =  cs.get('naca_file', 'NACA0015.csv')

        # Add simulation parameters to vessel config
        vessel_config.update({
            'sim_time': sim_params['sim_time'],
            'time_step': sim_params['time_step'],
            'density': sim_params['density'],
            'gravity': sim_params['gravity']
        })
        
        vessel_configs.append(vessel_config)
        if 'hydrodynamics' in vessel_config:
            hydrodynamic_data.append(vessel_config['hydrodynamics'])
        else:
            hydrodynamic_data.append({})
            
    return sim_params, vessel_configs, hydrodynamic_data

if __name__ == "__main__":
    # Example usage
    input_file = input("Please enter the path to the input file: ")
    sim_params, vessel_configs, hydrodynamic_data = read_input(input_file)
    print("Simulation parameters:")
    print(sim_params)
    print("\nVessel configurations:")
    for config in vessel_configs:
        print(f"\nVessel: {config['name']}")
        print(f"Initial position: {config.get('initial_position')}")
        print(f"Initial velocity: {config.get('initial_velocity')}")
        print(f"Control surfaces: {len(config.get('control_surfaces', []))}")
        print(f"Thrusters: {len(config.get('thrusters', []))}")