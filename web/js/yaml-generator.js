class YAMLGenerator {
    constructor() {
        this.defaultSimParams = {
            sim_time: 100,  // seconds
            time_step: 0.01,  // seconds
            density: 1025,  // kg/m³ (seawater)
            gravity: 9.80665,  // m/s²
            world_size: [1000, 1000, 100],  // meters [x, y, z]
            gps_datum: [12.99300425860631, 80.23913114094384, -87.0],  // [lat, lon, alt]
            nagents: 1
        };
    }

    validateGeometryFile(file) {
        return new Promise((resolve, reject) => {
            const reader = new FileReader();
            reader.onload = (e) => {
                try {
                    const data = jsyaml.load(e.target.result);
                    const required = ['length', 'breadth', 'depth', 'CO', 'CG', 'CB'];
                    const missing = required.filter(key => !(key in data));
                    
                    if (missing.length > 0) {
                        reject(`Missing required fields in geometry file: ${missing.join(', ')}`);
                        return;
                    }

                    // Validate numeric values
                    if (typeof data.length !== 'number' || data.length <= 0) reject('Invalid length');
                    if (typeof data.breadth !== 'number' || data.breadth <= 0) reject('Invalid breadth');
                    if (typeof data.depth !== 'number' || data.depth <= 0) reject('Invalid depth');
                    
                    // Validate arrays
                    ['CO', 'CG', 'CB'].forEach(key => {
                        if (!Array.isArray(data[key]) || data[key].length !== 3) {
                            reject(`Invalid ${key} coordinates`);
                        }
                    });

                    resolve(data);
                } catch (error) {
                    reject(`Invalid YAML format: ${error.message}`);
                }
            };
            reader.onerror = () => reject('Error reading file');
            reader.readAsText(file);
        });
    }

    validateInertiaFile(file) {
        return new Promise((resolve, reject) => {
            const reader = new FileReader();
            reader.onload = (e) => {
                try {
                    const data = jsyaml.load(e.target.result);
                    const required = ['mass', 'inertia_matrix', 'added_mass_matrix'];
                    const missing = required.filter(key => !(key in data));
                    
                    if (missing.length > 0) {
                        reject(`Missing required fields in inertia file: ${missing.join(', ')}`);
                        return;
                    }

                    // Validate mass
                    if (typeof data.mass !== 'number' || data.mass <= 0) {
                        reject('Invalid mass value');
                    }

                    // Validate matrices
                    ['inertia_matrix', 'added_mass_matrix'].forEach(matrix => {
                        if (!Array.isArray(data[matrix]) || 
                            data[matrix].length !== 3 || 
                            !data[matrix].every(row => Array.isArray(row) && row.length === 3)) {
                            reject(`Invalid ${matrix} format`);
                        }
                    });

                    resolve(data);
                } catch (error) {
                    reject(`Invalid YAML format: ${error.message}`);
                }
            };
            reader.onerror = () => reject('Error reading file');
            reader.readAsText(file);
        });
    }

    validateHydrodynamicsFile(file) {
        return new Promise((resolve, reject) => {
            const reader = new FileReader();
            reader.onload = (e) => {
                try {
                    const data = jsyaml.load(e.target.result);
                    const required = ['X_u', 'X_w', 'X_q', 'Y_v', 'Y_p', 'Y_r', 'N_v', 'N_p', 'N_r'];
                    const missing = required.filter(key => !(key in data));
                    
                    if (missing.length > 0) {
                        reject(`Missing required fields in hydrodynamics file: ${missing.join(', ')}`);
                        return;
                    }

                    // Validate all coefficients are numbers
                    required.forEach(coeff => {
                        if (typeof data[coeff] !== 'number') {
                            reject(`Invalid coefficient value for ${coeff}`);
                        }
                    });

                    resolve(data);
                } catch (error) {
                    reject(`Invalid YAML format: ${error.message}`);
                }
            };
            reader.onerror = () => reject('Error reading file');
            reader.readAsText(file);
        });
    }

    generateSimulationInput(vesselConfig) {
        const simInput = {
            ...this.defaultSimParams,
            agents: [{
                name: vesselConfig.name,
                type: vesselConfig.type,
                geometry: `inputs/${vesselConfig.name}/geometry.yml`,
                inertia: `inputs/${vesselConfig.name}/inertia.yml`,
                hydrodynamics: `inputs/${vesselConfig.name}/hydrodynamics.yml`,
                control_surfaces: `inputs/${vesselConfig.name}/control_surfaces.yml`,
                initial_conditions: `inputs/${vesselConfig.name}/initial_conditions.yml`,
                sensors: `inputs/${vesselConfig.name}/sensors.yml`,
                guidance: `inputs/${vesselConfig.name}/guidance.yml`,
                control: `inputs/${vesselConfig.name}/control.yml`
            }]
        };

        if (vesselConfig.thrusters && vesselConfig.thrusters.thrusters.length > 0) {
            simInput.agents[0].thrusters = `inputs/${vesselConfig.name}/thrusters.yml`;
        }

        return simInput;
    }

    generateControlSurfacesYAML(controlSurfaces) {
        return {
            naca_number: controlSurfaces.naca_number || '0015',
            control_surfaces: controlSurfaces.control_surfaces.map(surface => ({
                control_surface_id: surface.control_surface_id,
                control_surface_type: surface.control_surface_type,
                control_surface_location: surface.control_surface_location.map(val => Number(val.toFixed(4))),
                control_surface_orientation: surface.control_surface_orientation.map(val => Number(val.toFixed(4))),
                control_surface_area: Number(surface.control_surface_area.toFixed(4)),
                control_surface_NACA: surface.control_surface_NACA || '0015'
            }))
        };
    }

    generateThrustersYAML(thrusters) {
        return {
            thrusters: thrusters.thrusters.map(thruster => ({
                thruster_id: thruster.thruster_id,
                thruster_type: thruster.thruster_type,
                thruster_location: thruster.thruster_location.map(val => Number(val.toFixed(4))),
                thruster_orientation: thruster.thruster_orientation.map(val => Number(val.toFixed(4))),
                max_thrust: Number(thruster.max_thrust.toFixed(2))
            }))
        };
    }

    generateSensorsYAML(sensors) {
        return {
            sensors: sensors.sensors.map(sensor => ({
                sensor_id: sensor.sensor_id,
                sensor_type: sensor.sensor_type,
                sensor_location: sensor.sensor_location.map(val => Number(val.toFixed(4))),
                sensor_orientation: sensor.sensor_orientation.map(val => Number(val.toFixed(4))),
                publish_rate: Number(sensor.publish_rate.toFixed(1))
            }))
        };
    }

    generateInitialConditionsYAML(initialConditions) {
        return {
            velocity: initialConditions.velocity.map(val => Number(val.toFixed(4))),
            position: initialConditions.position.map(val => Number(val.toFixed(4))),
            quaternion: initialConditions.quaternion.map(val => Number(val.toFixed(4)))
        };
    }
}

// Export the class
window.YAMLGenerator = YAMLGenerator; 