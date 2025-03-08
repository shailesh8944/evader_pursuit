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

    // Helper function to format array inline without quotes
    formatInlineArray(arr, precision = 4) {
        if (!Array.isArray(arr)) {
            console.warn('Invalid array input:', arr);
            return '[0.0, 0.0, 0.0]';
        }
        return '[' + arr.map(val => Number(parseFloat(val).toFixed(precision))).join(', ') + ']';
    }

    // Helper function to format 2D array inline without quotes
    format2DInlineArray(arr, precision = 6) {
        if (!arr || !Array.isArray(arr)) return '[[0, 0, 0], [0, 0, 0], [0, 0, 0]]';
        
        let result = '[';
        for (let i = 0; i < arr.length; i++) {
            result += '[' + arr[i].map(val => Number(parseFloat(val)).toFixed(precision)).join(', ') + ']';
            if (i < arr.length - 1) result += ', ';
        }
        result += ']';
        return result;
    }

    // Helper to create Docker-compatible file paths
    formatFilePath(vesselName, fileName) {
        return `/workspaces/mavlab/inputs/${vesselName}/${fileName}`;
    }

    // Formats a numerical value with a comment
    formatWithComment(value, comment) {
        return `${value} # ${comment}`;
    }

    generateHydrodynamicsYAML(hydrodynamics, vesselName) {
        const hydraFile = this.formatFilePath(vesselName, `HydRA/${vesselName.toUpperCase()}_hydra.json`);
        
        return {
            hydra_file: hydraFile,
            dim_flag: hydrodynamics.dim_flag || false,
            cross_flow_drag: hydrodynamics.cross_flow_drag || false,
            
            // Surge hydrodynamic derivatives
            X_u_u: hydrodynamics.X_u_u || -0.01,
            
            // Sway hydrodynamic derivatives
            Y_v_v: hydrodynamics.Y_v_v || -0.010659050686665396,
            Y_r_r: hydrodynamics.Y_r_r || 0.0017799664706713543,
            
            // Yaw hydrodynamic derivatives
            N_v_v: hydrodynamics.N_v_v || -0.0006242757399292063,
            N_r_r: hydrodynamics.N_r_r || -0.001889456681929024,
            
            // Heave hydrodynamic derivatives
            Z_w_w: hydrodynamics.Z_w_w || -0.01
        };
    }

    generateGeometryYAML(geometry, vesselName) {
        // Apply scale to dimensions if present
        const scale = geometry.scale || [1, 1, 1];
        const scaledLength = Number((geometry.length * scale[0]).toFixed(4));
        const scaledBreadth = Number((geometry.breadth * scale[1]).toFixed(4));
        const scaledDepth = Number((geometry.depth * scale[2]).toFixed(4));

        // Calculate default gyration values if not provided
        let gyration = geometry.gyration;
        if (!gyration || gyration.every(val => val === 0)) {
            // Default gyration calculation based on vessel dimensions
            gyration = [
                scaledBreadth * 0.35, // Roll gyration radius
                scaledLength * 0.25,  // Pitch gyration radius
                scaledLength * 0.25   // Yaw gyration radius
            ];
        }

        return {
            length: scaledLength,
            breadth: scaledBreadth,
            depth: scaledDepth,
            gyration: this.formatInlineArray(gyration, 4),
            CO: this.formatInlineArray(geometry.CO || [0.0, 0.0, 0.0], 4),
            CG: this.formatInlineArray(geometry.CG || [0.0, 0.0, 0.0], 4),
            CB: this.formatInlineArray(geometry.CB || [0.0, 0.0, 0.0], 4),
            geometry_file: geometry.geometry_file || this.formatFilePath(vesselName, 'HydRA/input/vessel.gdf')
        };
    }

    generateInertiaYAML(inertia) {
        const mass = Number(parseFloat(inertia.mass || 1).toFixed(4));
        const buoyancy_mass = Number(parseFloat(inertia.buoyancy_mass || mass).toFixed(4));
        
        return {
            mass: this.formatWithComment(mass, 'in kg'),
            buoyancy_mass: this.formatWithComment(buoyancy_mass, 'in kg'),
            inertia_matrix: inertia.inertia_matrix ? this.format2DInlineArray(inertia.inertia_matrix, 6) + ' # in kg*m^2' : 'None # in kg*m^2',
            added_mass_matrix: inertia.added_mass_matrix ? this.format2DInlineArray(inertia.added_mass_matrix, 6) + ' # in kg*m^2/s' : 'None # in kg*m^2/s'
        };
    }

    generateControlSurfacesYAML(controlSurfaces, vesselName) {
        const nacaNumber = controlSurfaces.naca_number || '0015';
        const nacaFile = this.formatFilePath(vesselName, `NACA${nacaNumber}.csv`);
        
        const yaml = {
            naca_number: `&naca ${nacaNumber}`,
            naca_file: nacaFile,
            delta_max: '&dmax 35.0',
            deltad_max: '&ddmax 1.0',
            area: '&area 0.1',
            control_surfaces: []
        };
        
        // Add control surfaces if available
        if (controlSurfaces.control_surfaces && controlSurfaces.control_surfaces.length > 0) {
            yaml.control_surfaces = controlSurfaces.control_surfaces.map(surface => ({
                control_surface_type: surface.control_surface_type || 'Rudder',
                control_surface_id: surface.control_surface_id,
                control_surface_location: this.formatInlineArray(surface.control_surface_location, 4) + '    # With respect to BODY frame',
                control_surface_orientation: this.formatInlineArray(surface.control_surface_orientation, 4) + '  # With respect to BODY frame',
                control_surface_area: '*area',
                control_surface_NACA: '*naca',
                control_surface_T: surface.control_surface_T || 0.1,
                control_surface_delta_max: '*dmax',
                control_surface_deltad_max: '*ddmax'
            }));
        }
        
        return yaml;
    }

    generateThrustersYAML(thrusters, vesselName) {
        const yaml = {
            thrusters: []
        };
        
        if (thrusters?.thrusters && thrusters.thrusters.length > 0) {
            yaml.thrusters = thrusters.thrusters.map(thruster => ({
                thruster_name: thruster.thruster_name || `thruster_${thruster.thruster_id}`,
                thruster_id: thruster.thruster_id,
                thruster_location: this.formatInlineArray(thruster.thruster_location, 4) + '    # With respect to BODY frame',
                thruster_orientation: this.formatInlineArray(thruster.thruster_orientation, 4) + '  # With respect to BODY frame',
                T_prop: thruster.T_prop || 1.0,
                D_prop: Number(parseFloat(thruster.D_prop || 0.1).toFixed(4)),
                tp: Number(parseFloat(thruster.tp || 0.1).toFixed(4)),
                J_vs_KT: this.formatFilePath(vesselName, 'J_vs_KT.csv')
            }));
        }
        
        return yaml;
    }

    generateInitialConditionsYAML(initialConditions) {
        return {
            start_location: this.formatInlineArray(initialConditions.start_location || [0, 0, 0], 4),
            start_orientation: this.formatInlineArray(initialConditions.start_attitude || [0, 0, 0], 4),
            start_velocity: this.formatInlineArray(initialConditions.start_velocity || [0, 0, 0, 0, 0, 0], 4),
            use_quaternion: initialConditions.use_quaternion || false
        };
    }

    generateGuidanceYAML(guidance) {
        const yaml = {
            guidance_type: guidance.guidance_type || 'waypoint'
        };
        
        if (guidance.waypoints && guidance.waypoints.length > 0) {
            yaml.waypoints = guidance.waypoints.map(wp => ({
                x: Number(parseFloat(wp.x || 0).toFixed(4)),
                y: Number(parseFloat(wp.y || 0).toFixed(4)),
                z: Number(parseFloat(wp.z || 0).toFixed(4)),
                psi: Number(parseFloat(wp.psi || 0).toFixed(4))
            }));
        }
        
        return yaml;
    }

    generateControlYAML(control) {
        return {
            control_surface_control_type: control.control_surface_control_type || 'fixed_rudder',
            thruster_control_type: control.thruster_control_type || 'fixed_rpm'
        };
    }

    generateSensorsYAML(sensors) {
        const yaml = {
            sensors: []
        };
        
        if (sensors.sensors && sensors.sensors.length > 0) {
            yaml.sensors = sensors.sensors.map(sensor => ({
                sensor_type: sensor.sensor_type || 'IMU',
                sensor_id: sensor.sensor_id,
                sensor_location: sensor.sensor_location ? this.formatInlineArray(sensor.sensor_location, 4) + '          # With respect to BODY frame' : 'None',
                sensor_orientation: sensor.sensor_orientation ? this.formatInlineArray(sensor.sensor_orientation, 4) + '    # With respect to BODY frame' : 'None',
                publish_rate: Number(parseFloat(sensor.publish_rate || 10).toFixed(1))
            }));
        }
        
        return yaml;
    }

    generateSimulationInputYAML(vesselConfig) {
        const vesselName = vesselConfig.name || 'vessel';
        
        const simInput = {
            ...this.defaultSimParams,
            agents: [{
                name: vesselName,
                type: vesselConfig.type || 'auv',
                geometry: this.formatFilePath(vesselName, 'geometry.yml'),
                inertia: this.formatFilePath(vesselName, 'inertia.yml'),
                hydrodynamics: this.formatFilePath(vesselName, 'hydrodynamics.yml'),
                control_surfaces: this.formatFilePath(vesselName, 'control_surfaces.yml'),
                initial_conditions: this.formatFilePath(vesselName, 'initial_conditions.yml'),
                sensors: this.formatFilePath(vesselName, 'sensors.yml'),
                guidance: this.formatFilePath(vesselName, 'guidance.yml'),
                control: this.formatFilePath(vesselName, 'control.yml')
            }]
        };

        if (vesselConfig.thrusters && vesselConfig.thrusters.thrusters && vesselConfig.thrusters.thrusters.length > 0) {
            simInput.agents[0].thrusters = this.formatFilePath(vesselName, 'thrusters.yml');
        }

        return simInput;
    }
    
    // Convert JavaScript object to YAML string with proper formatting
    toYAML(obj, indent = 0) {
        let yaml = '';
        const spaces = ' '.repeat(indent);
        
        for (const [key, value] of Object.entries(obj)) {
            // Skip empty arrays and objects
            if ((Array.isArray(value) && value.length === 0) || 
                (typeof value === 'object' && value !== null && !Array.isArray(value) && Object.keys(value).length === 0)) {
                continue;
            }
            
            if (value === null || value === undefined) {
                yaml += `${spaces}${key}: null\n`;
            } else if (typeof value === 'object' && !Array.isArray(value) && !(typeof value === 'string' && value.includes('#'))) {
                // Handle nested objects
                if (key === 'control_surfaces' || key === 'thrusters' || key === 'sensors' || key === 'waypoints' || key === 'agents' || key === 'geofence') {
                    // These are arrays of objects
                    yaml += `${spaces}${key}:\n`;
                    if (Array.isArray(value)) {
                        // Handle array of objects
                        for (const item of value) {
                            yaml += `${spaces}  - \n`;
                            for (const [itemKey, itemValue] of Object.entries(item)) {
                                if (typeof itemValue === 'string' && (itemValue.startsWith('*') || itemValue.includes('#'))) {
                                    yaml += `${spaces}    ${itemKey}: ${itemValue}\n`;
                                } else if (typeof itemValue === 'object' && !Array.isArray(itemValue)) {
                                    yaml += `${spaces}    ${itemKey}:\n${this.toYAML(itemValue, indent + 6)}`;
                                } else {
                                    yaml += `${spaces}    ${itemKey}: ${itemValue}\n`;
                                }
                            }
                        }
                    } else {
                        // It's an object with a reserved name but not an array
                        yaml += this.toYAML(value, indent + 2);
                    }
                } else {
                    // Regular nested object
                    yaml += `${spaces}${key}:\n${this.toYAML(value, indent + 2)}`;
                }
            } else if (Array.isArray(value)) {
                if (value.length === 0) {
                    yaml += `${spaces}${key}: []\n`;
                } else if (typeof value[0] === 'object') {
                    // Array of objects
                    yaml += `${spaces}${key}:\n`;
                    for (const item of value) {
                        yaml += `${spaces}  -\n${this.toYAML(item, indent + 4)}`;
                    }
                } else {
                    // Check if the value is already formatted as a string with formatting
                    if (typeof value === 'string' && (value.startsWith('[') || value.includes('#'))) {
                        yaml += `${spaces}${key}: ${value}\n`;
                    } else {
                        // Format array
                        yaml += `${spaces}${key}: ${this.formatInlineArray(value)}\n`;
                    }
                }
            } else if (typeof value === 'string' && (value.startsWith('*') || value.includes('#'))) {
                // Handle anchor references and comments
                yaml += `${spaces}${key}: ${value}\n`;
            } else if (typeof value === 'string' && value.startsWith('/')) {
                // File paths
                yaml += `${spaces}${key}: ${value}\n`;
            } else {
                // Default formatting
                yaml += `${spaces}${key}: ${value}\n`;
            }
        }
        
        return yaml;
    }

    // Generate all YAML files and return them as an object
    generateAllYAMLFiles(vesselModel) {
        const vessel = vesselModel.config;
        
        const files = {
            'simulation_input.yml': this.toYAML(this.generateSimulationInputYAML(vessel)),
            'geometry.yml': this.toYAML(this.generateGeometryYAML(vessel.geometry, vessel.name)),
            'inertia.yml': this.toYAML(this.generateInertiaYAML(vessel.inertia)),
            'hydrodynamics.yml': this.toYAML(this.generateHydrodynamicsYAML(vessel.hydrodynamics, vessel.name)),
            'control_surfaces.yml': this.toYAML(this.generateControlSurfacesYAML(vessel.control_surfaces, vessel.name)),
            'initial_conditions.yml': this.toYAML(this.generateInitialConditionsYAML(vessel.initial_conditions)),
            'sensors.yml': this.toYAML(this.generateSensorsYAML(vessel.sensors)),
            'guidance.yml': this.toYAML(this.generateGuidanceYAML(vessel.guidance)),
            'control.yml': this.toYAML(this.generateControlYAML(vessel.control))
        };
        
        if (vessel.thrusters && vessel.thrusters.thrusters && vessel.thrusters.thrusters.length > 0) {
            files['thrusters.yml'] = this.toYAML(this.generateThrustersYAML(vessel.thrusters, vessel.name));
        }
        
        return files;
    }

    // Creates a .zip file with all the YAML files
    async createZipFile(vesselModel) {
        const files = this.generateAllYAMLFiles(vesselModel);
        const vesselName = vesselModel.config.name;
        
        const zip = new JSZip();
        const folder = zip.folder(vesselName);
        
        // Add each YAML file to the zip
        for (const [filename, content] of Object.entries(files)) {
            folder.file(filename, content);
        }
        
        // Create HydRA folder
        folder.folder('HydRA');
        
        // Create a default NACA file if we have control surfaces
        if (vesselModel.config.control_surfaces && vesselModel.config.control_surfaces.control_surfaces.length > 0) {
            const nacaNumber = vesselModel.config.control_surfaces.naca_number || '0015';
            if (vesselModel.getDefaultNacaFile) {
                folder.file(`NACA${nacaNumber}.csv`, vesselModel.getDefaultNacaFile());
            }
        }
        
        // Generate zip content
        return await zip.generateAsync({ type: 'blob' });
    }
}

// Export the class
window.YAMLGenerator = YAMLGenerator; 