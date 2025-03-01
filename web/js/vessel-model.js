class VesselModel {
    constructor() {
        this.config = {
            name: '',
            type: 'auv',
            geometry: {
                length: 0,
                breadth: 0,
                depth: 0,
                CO: [0, 0, 0],
                CG: [0, 0, 0],
                CB: [0, 0, 0],
                gyration: [0, 0, 0]
            },
            inertia: {
                mass: 0,
                buoyancy_mass: 0,
                inertia_matrix: [
                    [0, 0, 0],
                    [0, 0, 0],
                    [0, 0, 0]
                ],
                added_mass_matrix: [
                    [0, 0, 0],
                    [0, 0, 0],
                    [0, 0, 0]
                ]
            },
            hydrodynamics: {
                hydra_file: '/workspaces/mavlab/inputs/mavymini/HydRA/MAVYMINI_hydra.json',
                dim_flag: false,
                X_u_u: 0,
                Y_v_v: 0,
                Y_r_r: 0,
                N_v_v: 0,
                N_r_r: 0,
                Z_w_w: 0
            },
            control_surfaces: {
                naca_file: '/workspaces/mavlab/inputs/mavymini/naca0015.csv',
                control_surfaces: []
            },
            initial_conditions: {
                start_velocity: [0, 0, 0, 0, 0, 0],
                start_location: [0, 0, 0],
                start_attitude: [0, 0, 0],
                use_quaternion: false
            },
            sensors: {
                sensors: []
            },
            guidance: {
                guidance_type: 'waypoint',
                waypoints: []
            },
            control: {
                control_surface_control_type: 'fixed_rudder',
                thruster_control_type: 'fixed_rpm'
            }
        };

        this.nextControlSurfaceId = 1;
        this.nextThrusterId = 1;
        this.nextSensorId = 1;
    }

    updateBasicProperties(name, type, length, breadth, depth) {
        this.config.name = name;
        this.config.type = type;
        this.config.geometry.length = length;
        this.config.geometry.breadth = breadth;
        this.config.geometry.depth = depth;
    }

    addControlSurface(type, location, orientation, area) {
        const surface = {
            control_surface_type: type,
            control_surface_id: this.nextControlSurfaceId++,
            control_surface_location: location,
            control_surface_orientation: orientation,
            control_surface_area: area,
            control_surface_T: 0.1,  // Default time constant
            control_surface_delta_max: 30 * Math.PI / 180,  // 30 degrees in radians
            control_surface_deltad_max: 10 * Math.PI / 180  // 10 degrees/s in radians/s
        };

        this.config.control_surfaces.control_surfaces.push(surface);
        return surface.control_surface_id;
    }

    addThruster(type, location, orientation) {
        const thruster = {
            thruster_id: this.nextThrusterId++,
            thruster_type: type,
            thruster_location: location,
            thruster_orientation: orientation,
            D_prop: 0.2,  // Default propeller diameter in meters
            tp: 0.1  // Default thrust deduction factor
        };

        if (!this.config.thrusters) {
            this.config.thrusters = { thrusters: [] };
        }
        this.config.thrusters.thrusters.push(thruster);
        return thruster.thruster_id;
    }

    addSensor(type, location, orientation, publishRate) {
        const sensor = {
            sensor_type: type,
            sensor_id: this.nextSensorId++,
            sensor_location: location,
            sensor_orientation: orientation,
            publish_rate: publishRate
        };

        this.config.sensors.sensors.push(sensor);
        return sensor.sensor_id;
    }

    updateInertiaProperties(mass, inertiaMatrix, addedMassMatrix) {
        this.config.inertia.mass = mass;
        this.config.inertia.inertia_matrix = inertiaMatrix;
        this.config.inertia.added_mass_matrix = addedMassMatrix;
    }

    updateHydrodynamics(coefficients) {
        Object.assign(this.config.hydrodynamics, coefficients);
    }

    updateInitialConditions(startVelocity, startLocation, startAttitude, useQuaternion = false) {
        this.config.initial_conditions.start_velocity = startVelocity;
        this.config.initial_conditions.start_location = startLocation;
        this.config.initial_conditions.start_attitude = startAttitude;
        this.config.initial_conditions.use_quaternion = useQuaternion;
    }

    removeControlSurface(id) {
        const index = this.config.control_surfaces.control_surfaces.findIndex(
            surface => surface.control_surface_id === id
        );
        if (index !== -1) {
            this.config.control_surfaces.control_surfaces.splice(index, 1);
            return true;
        }
        return false;
    }

    removeThruster(id) {
        if (!this.config.thrusters) return false;
        const index = this.config.thrusters.thrusters.findIndex(
            thruster => thruster.thruster_id === id
        );
        if (index !== -1) {
            this.config.thrusters.thrusters.splice(index, 1);
            return true;
        }
        return false;
    }

    removeSensor(id) {
        const index = this.config.sensors.sensors.findIndex(
            sensor => sensor.sensor_id === id
        );
        if (index !== -1) {
            this.config.sensors.sensors.splice(index, 1);
            return true;
        }
        return false;
    }

    generateYAMLFiles() {
        const yamlGenerator = new YAMLGenerator();
        return {
            'simulation_input.yml': yamlGenerator.generateSimulationInput(this.config),
            'geometry.yml': yamlGenerator.generateGeometryYAML(this.config.geometry),
            'inertia.yml': yamlGenerator.generateInertiaYAML(this.config.inertia),
            'hydrodynamics.yml': yamlGenerator.generateHydrodynamicsYAML(this.config.hydrodynamics),
            'control_surfaces.yml': yamlGenerator.generateControlSurfacesYAML(this.config.control_surfaces),
            'initial_conditions.yml': yamlGenerator.generateInitialConditionsYAML(this.config.initial_conditions),
            'sensors.yml': yamlGenerator.generateSensorsYAML(this.config.sensors),
            'guidance.yml': yamlGenerator.generateGuidanceYAML(this.config.guidance),
            'control.yml': yamlGenerator.generateControlYAML(this.config.control)
        };
    }
}

// Export the class
window.VesselModel = VesselModel; 