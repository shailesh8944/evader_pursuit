from typing import NoReturn
import numpy as np
import sys
sys.path.append('/workspaces/mavlab/')

from read_input import read_input
from class_vessel_lat import Vessel

def simulate() -> NoReturn:
    """Run vessel simulation using parameters from input files."""
    # Read input files and create vessel
    sim_params, vessel_configs, hydrodynamic_data = read_input("/workspaces/mavlab/inputs/mavymini/simulation_input.yml")
    vessel = Vessel(vessel_configs[0], hydrodynamic_data[0])
    
    print("\nStarting simulation...")
    print("Time [s] | Position [x, y, z] | Velocity [u, v, w]")
    print("-" * 60)
    
    # Run simulation
    vessel.reset()
    while vessel.t < vessel.Tmax:
        # Step simulation
        vessel.step()
        
        # Extract position and velocity
        pos = vessel.current_state[6:9]  # [x, y, z]
        vel = vessel.current_state[0:3]  # [u, v, w]
        
        # Print to terminal
        print(f"{vessel.t:6.2f} | [{pos[0]:6.2f}, {pos[1]:6.2f}, {pos[2]:6.2f}] | [{vel[0]:6.2f}, {vel[1]:6.2f}, {vel[2]:6.2f}]")
    
    print("-" * 60)
    print("Simulation complete!")

if __name__ == "__main__":
    simulate()
