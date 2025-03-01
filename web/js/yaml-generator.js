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
    formatInlineArray(arr, precision = 6) {
        return arr.map(val => Number(val).toFixed(precision));
    }

    // Helper function to format 2D array inline without quotes
    format2DInlineArray(arr, precision = 6) {
        // Return raw array string without quotes
        return [[...arr.map(row => [...row.map(val => Number(val).toFixed(precision))])]];
    }

    generateHydrodynamicsYAML(hydrodynamics) {
        return {
            hydra_file: hydrodynamics.hydra_file || '/workspaces/mavlab/inputs/mavymini/HydRA/MAVYMINI_hydra.json',
            dim_flag: hydrodynamics.dim_flag || false,
            
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

    generateGeometryYAML(geometry) {
        return {
            length: Number(geometry.length.toFixed(4)),
            breadth: Number(geometry.breadth.toFixed(4)),
            depth: Number(geometry.depth.toFixed(4)),
            CO: Array[geometry.CO[0], geometry.CO[1], geometry.CO[2]],
            CG: this.formatInlineArray([geometry.CG[0], geometry.CG[1], geometry.CG[2]], 4),
            CB: this.formatInlineArray([geometry.CB[0], geometry.CB[1], geometry.CB[2]], 4),
            gyration: this.formatInlineArray([geometry.gyration[0], geometry.gyration[1], geometry.gyration[2]], 4)
        };
    }

    generateInertiaYAML(inertia) {
        return {
            'mass': Number(inertia.mass.toFixed(4)) + ' # in kg',
            'buoyancy_mass': Number(inertia.buoyancy_mass.toFixed(4)) + ' # in kg',
            'inertia_matrix': this.format2DInlineArray(inertia.inertia_matrix) + ' # in kg*m^2',
            'added_mass_matrix': this.format2DInlineArray(inertia.added_mass_matrix) + ' # in kg*m^2/s'
        };
    }

    generateControlSurfacesYAML(controlSurfaces) {
        return {
            naca_file: controlSurfaces.naca_file || '/workspaces/mavlab/inputs/mavymini/naca0015.csv',
            control_surfaces: controlSurfaces.control_surfaces.map(surface => ({
                control_surface_id: surface.control_surface_id,
                control_surface_type: surface.control_surface_type,
                control_surface_location: this.formatInlineArray(surface.control_surface_location, 4),
                control_surface_orientation: this.formatInlineArray(surface.control_surface_orientation, 4),
                control_surface_area: Number(surface.control_surface_area.toFixed(4)),
                control_surface_T: Number(surface.control_surface_T.toFixed(4)),
                control_surface_delta_max: Number(surface.control_surface_delta_max.toFixed(4)),
                control_surface_deltad_max: Number(surface.control_surface_deltad_max.toFixed(4))
            }))
        };
    }

    generateThrustersYAML(thrusters) {
        return {
            thrusters: thrusters.thrusters.map(thruster => ({
                thruster_id: thruster.thruster_id,
                thruster_type: thruster.thruster_type,
                thruster_location: this.formatInlineArray(thruster.thruster_location, 4),
                thruster_orientation: this.formatInlineArray(thruster.thruster_orientation, 4),
                D_prop: Number(thruster.D_prop.toFixed(4)),
                tp: Number(thruster.tp.toFixed(4))
            }))
        };
    }

    generateInitialConditionsYAML(initialConditions) {
        return {
            start_velocity: this.formatInlineArray(initialConditions.start_velocity, 4),
            start_location: this.formatInlineArray(initialConditions.start_location, 4),
            start_attitude: this.formatInlineArray(initialConditions.start_attitude, 4),
            use_quaternion: initialConditions.use_quaternion || false
        };
    }

    generateGuidanceYAML(guidance) {
        return {
            guidance_type: guidance.guidance_type || 'waypoint',
            waypoints: (guidance.waypoints || []).map(wp => ({
                x: Number(wp.x.toFixed(4)),
                y: Number(wp.y.toFixed(4)),
                z: Number(wp.z.toFixed(4)),
                psi: Number(wp.psi.toFixed(4))
            }))
        };
    }

    generateControlYAML(control) {
        return {
            control_surface_control_type: control.control_surface_control_type || 'fixed_rudder',
            thruster_control_type: control.thruster_control_type || 'fixed_rpm'
        };
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

    generateSensorsYAML(sensors) {
        return {
            sensors: sensors.sensors.map(sensor => ({
                sensor_id: sensor.sensor_id,
                sensor_type: sensor.sensor_type,
                sensor_location: this.formatInlineArray(sensor.sensor_location, 4),
                sensor_orientation: this.formatInlineArray(sensor.sensor_orientation, 4),
                publish_rate: Number(sensor.publish_rate.toFixed(1))
            }))
        };
    }
}

// Export the class
window.YAMLGenerator = YAMLGenerator; 