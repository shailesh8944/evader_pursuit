from typing import Dict, Tuple, List
import numpy as np
import yaml
import os
from pathlib import Path
from generate_cross_flow import CrossFlowGenerator

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
            agents (List[Dict]): List of agents, each containing vessel config and hydrodynamics
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
        'nagents': sim_config.get('nagents', 1),
        'geofence': sim_config.get('geofence', [])
    }
    
    # Process each vessel's configuration
    agents = []
    
    for agent in sim_config.get('agents', []):
        vessel_name = agent['name']
        vessel_config = {
            'name': vessel_name,
            'type': agent['type']
        }
        
        gdf_file = load_yaml(resolve_path(base_path, agent['geometry'], vessel_name))['geometry_file']
        hydra_file = load_yaml(resolve_path(base_path, agent['hydrodynamics'], vessel_name))['hydra_file']
        hydrodynamics_file = resolve_path(base_path, agent['hydrodynamics'], vessel_name)
        initial_conditions_file = resolve_path(base_path, agent['initial_conditions'], vessel_name)
        cross_flow_generator = CrossFlowGenerator(gdf_file, hydra_file, hydrodynamics_file, initial_conditions_file, agent['type'])
        cross_flow_generator.update_yaml_file()
        
        # Load all referenced configuration files
        for config_type in ['geometry', 'inertia', 'hydrodynamics', 'control_surfaces', 
                          'initial_conditions', 'sensors', 'guidance', 'control', 'thrusters']:
            if config_type in agent:
                file_path = resolve_path(base_path, agent[config_type], vessel_name)
                vessel_config[config_type] = load_yaml(file_path)

        # Add simulation parameters to vessel config
        vessel_config.update({
            'sim_time': sim_params['sim_time'],
            'time_step': sim_params['time_step'],
            'density': sim_params['density'],
            'gravity': sim_params['gravity'],
            'gps_datum': sim_params['gps_datum']
        })
        
        # Create agent dictionary with vessel config and hydrodynamics
        agent_data = {
            'vessel_config': vessel_config,
            'hydrodynamics': vessel_config.get('hydrodynamics', {})
        }
        agents.append(agent_data)
            
    return sim_params, agents

if __name__ == "__main__":
    # Example usage
    input_file = input("Please enter the path to the input file: ")
    sim_params, agents = read_input(input_file)
    print("Simulation parameters:")
    print(sim_params)
    print("\nAgents:")
    for agent in agents:
        vessel_config = agent['vessel_config']
        print(f"\nVessel: {vessel_config['name']}")
        print(f"Initial position: {vessel_config.get('initial_position')}")
        print(f"Initial velocity: {vessel_config.get('initial_velocity')}")
        print(f"Control surfaces: {len(vessel_config.get('control_surfaces', []))}")
        print(f"Thrusters: {len(vessel_config.get('thrusters', []))}")
        print(f"Has hydrodynamics data: {bool(agent['hydrodynamics'])}")