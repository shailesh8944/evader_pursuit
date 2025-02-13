import numpy as np
import yaml
from pathlib import Path
import json

class CalculateHydrodynamics:
    def __init__(self, vessel_data):
        self.vessel_data = vessel_data
        self.length = vessel_data.get('length', 1.0)
        self.breadth = vessel_data.get('breadth', 0.2)
        self.draft = vessel_data.get('draft', 0.1)
        self.scale = vessel_data.get('scale', 1.0)
        self.rho = vessel_data.get('density', 1000)
        self.g = 9.81
        
        # Design speed and Froude number
        self.Fn = vessel_data.get('design_froude_number', 0.2)
        self.U_des = self.Fn * np.sqrt(self.g * self.length)
        if 'design_speed' in vessel_data:
            self.U_des = vessel_data['design_speed']
            self.Fn = self.U_des / np.sqrt(self.g * self.length)

    def calculate_coefficients(self):
        """Calculate all hydrodynamic coefficients"""
        coefficients = {}
        
        # Added mass coefficients
        coefficients.update(self.calculate_added_mass())
        
        # Damping coefficients
        coefficients.update(self.calculate_damping())
        
        # Cross-flow drag coefficients
        coefficients.update(self.calculate_cross_flow_drag())
        
        # Propulsion and control coefficients
        coefficients.update(self.calculate_propulsion_coefficients())
        
        return coefficients

    def calculate_added_mass(self):
        """Calculate added mass coefficients"""
        # Simplified strip theory approach
        L = self.length
        B = self.breadth
        T = self.draft
        
        # Non-dimensional added mass coefficients
        Xu = -0.05  # Surge added mass
        Yv = -np.pi * (T/L)**2  # Sway added mass
        Zw = -np.pi * (T/L)**2  # Heave added mass
        Kr = -0.002  # Roll added mass
        Mq = -np.pi * (T/L)**4 * (1 + 0.16 * B/T)  # Pitch added mass
        Nr = -np.pi * (T/L)**4 * (1 + 0.16 * B/T)  # Yaw added mass
        
        return {
            'Xu': Xu,
            'Yv': Yv,
            'Zw': Zw,
            'Kr': Kr,
            'Mq': Mq,
            'Nr': Nr
        }

    def calculate_damping(self):
        """Calculate damping coefficients"""
        L = self.length
        B = self.breadth
        T = self.draft
        
        # Reynolds number
        Rn = self.U_des * L * 1e6
        Cf = 0.075 / ((np.log10(Rn) - 2) ** 2)
        
        # Linear damping coefficients
        Xu = -Cf * (1 + 0.1 * B/L)  # Surge damping
        Yv = -0.5 * np.pi * (T/L)**2  # Sway damping
        Zw = -0.5 * np.pi * (T/L)**2  # Heave damping
        Kr = -0.001  # Roll damping
        Mq = -0.5 * np.pi * (T/L)**4  # Pitch damping
        Nr = -0.5 * np.pi * (T/L)**4  # Yaw damping
        
        return {
            'Xu_l': Xu,
            'Yv_l': Yv,
            'Zw_l': Zw,
            'Kr_l': Kr,
            'Mq_l': Mq,
            'Nr_l': Nr
        }

    def calculate_cross_flow_drag(self):
        """Calculate cross-flow drag coefficients"""
        L = self.length
        B = self.breadth
        T = self.draft
        
        # Cross-flow drag coefficients based on Hoerner's equations
        Cd = 1.1 + 0.2 * (B/T)
        
        # Non-dimensional coefficients
        Yvv = -0.5 * Cd * (T/L)**2
        Yvr = -0.5 * Cd * (T/L)**2
        Zww = -0.5 * Cd * (T/L)**2
        Zwq = -0.5 * Cd * (T/L)**2
        Mww = -0.5 * Cd * (T/L)**3
        Nrr = -0.5 * Cd * (T/L)**3
        
        return {
            'Yvv': Yvv,
            'Yvr': Yvr,
            'Zww': Zww,
            'Zwq': Zwq,
            'Mww': Mww,
            'Nrr': Nrr
        }

    def calculate_propulsion_coefficients(self):
        """Calculate propulsion and control surface coefficients"""
        L = self.length
        
        # Propeller coefficients
        Kt = 0.5  # Thrust coefficient
        Kq = 0.05  # Torque coefficient
        wp = 0.2  # Wake fraction
        t = 0.1  # Thrust deduction
        
        # Control surface coefficients
        Yr = -0.1  # Rudder sway force
        Nr = -0.05  # Rudder yaw moment
        
        return {
            'Kt': Kt,
            'Kq': Kq,
            'wp': wp,
            't': t,
            'Yr': Yr,
            'Nr': Nr
        }

    def save_to_yaml(self, output_path='hydrodynamics.yml'):
        """Save calculated coefficients to YAML file"""
        coefficients = self.calculate_coefficients()
        
        # Add metadata
        data = {
            'vessel': {
                'name': self.vessel_data.get('name', 'unknown'),
                'length': self.length,
                'breadth': self.breadth,
                'draft': self.draft,
                'scale': self.scale,
                'design_speed': self.U_des,
                'froude_number': self.Fn
            },
            'coefficients': coefficients
        }
        
        # Save to YAML file
        with open(output_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        
        return data

def main(vessel_data_path):
    """Main function to calculate hydrodynamics from vessel data file"""
    # Load vessel data
    with open(vessel_data_path, 'r') as f:
        vessel_data = yaml.safe_load(f)
    
    # Calculate hydrodynamics
    hydro = CalculateHydrodynamics(vessel_data)
    
    # Save results
    output_path = Path(vessel_data_path).parent / 'hydrodynamics.yml'
    return hydro.save_to_yaml(output_path)

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        main(sys.argv[1])
    else:
        print("Please provide path to vessel data YAML file") 