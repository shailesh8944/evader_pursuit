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
                CB: [0, 0, 0]
            },
            inertia: {
                mass: 0,
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
                X_u: 0,
                X_w: 0,
                X_q: 0,
                Y_v: 0,
                Y_p: 0,
                Y_r: 0,
                N_v: 0,
                N_p: 0,
                N_r: 0
            },
            control_surfaces: {
                naca_number: '0015',
                control_surfaces: []
            },
            initial_conditions: {
                velocity: [0, 0, 0, 0, 0, 0],
                position: [0, 0, 0],
                quaternion: [1, 0, 0, 0]
            },
            sensors: {
                sensors: []
            },
            guidance: {},
            control: {}
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
            control_surface_NACA: this.config.control_surfaces.naca_number
        };

        this.config.control_surfaces.control_surfaces.push(surface);
        return surface.control_surface_id;
    }

    addThruster(type, location, orientation, maxThrust) {
        const thruster = {
            thruster_id: this.nextThrusterId++,
            thruster_type: type,
            thruster_location: location,
            thruster_orientation: orientation,
            max_thrust: maxThrust
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

    updateInitialConditions(velocity, position, quaternion) {
        this.config.initial_conditions.velocity = velocity;
        this.config.initial_conditions.position = position;
        this.config.initial_conditions.quaternion = quaternion;
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
        const files = {
            'simulation_input.yml': this.generateSimulationInput(),
            'geometry.yml': this.generateGeometryYAML(),
            'inertia.yml': this.generateInertiaYAML(),
            'hydrodynamics.yml': this.generateHydrodynamicsYAML(),
            'control_surfaces.yml': this.generateControlSurfacesYAML(),
            'initial_conditions.yml': this.generateInitialConditionsYAML(),
            'sensors.yml': this.generateSensorsYAML(),
            'guidance.yml': this.generateGuidanceYAML(),
            'control.yml': this.generateControlYAML()
        };

        if (this.config.thrusters) {
            files['thrusters.yml'] = this.generateThrustersYAML();
        }

        return files;
    }

    generateSimulationInput() {
        return {
            sim_time: 100,
            time_step: 0.01,
            density: 1000,
            gravity: 9.80665,
            world_size: [100, 100, 'Inf'],
            gps_datum: [12.99300425860631, 80.23913114094384, -87.0],
            nagents: 1,
            agents: [{
                name: this.config.name,
                type: this.config.type,
                geometry: `/workspaces/mavlab/inputs/${this.config.name}/geometry.yml`,
                inertia: `/workspaces/mavlab/inputs/${this.config.name}/inertia.yml`,
                hydrodynamics: `/workspaces/mavlab/inputs/${this.config.name}/hydrodynamics.yml`,
                control_surfaces: `/workspaces/mavlab/inputs/${this.config.name}/control_surfaces.yml`,
                initial_conditions: `/workspaces/mavlab/inputs/${this.config.name}/initial_conditions.yml`,
                sensors: `/workspaces/mavlab/inputs/${this.config.name}/sensors.yml`,
                guidance: `/workspaces/mavlab/inputs/${this.config.name}/guidance.yml`,
                control: `/workspaces/mavlab/inputs/${this.config.name}/control.yml`
            }]
        };
    }

    generateGeometryYAML() {
        return {
            length: this.config.geometry.length,  // Dimensions of the box enclosing the vessel
            breadth: this.config.geometry.breadth,
            depth: this.config.geometry.depth,
            CO: this.config.geometry.CO || [0.0, 0.0, 0.0],
            CG: this.config.geometry.CG || [0.0, 0.0, 0.0],
            CB: this.config.geometry.CB || [0.0, 0.0, 0.0],
            geometry_file: `/workspaces/mavlab/ros2_ws/src/mav_simulator/mav_simulator/hullform/${this.config.name}/${this.config.name}.gdf`
        };
    }

    generateInertiaYAML() {
        return {
            mass: this.config.inertia.mass, // in kg
            inertia_matrix: this.config.inertia.inertia_matrix || [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], // in kg*m^2
            added_mass_matrix: this.config.inertia.added_mass_matrix || [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]] // in kg*m^2/s
        };
    }

    generateHydrodynamicsYAML() {
        return {
            X_u: this.config.hydrodynamics.X_u || 0.0,
            X_w: this.config.hydrodynamics.X_w || 0.0,
            X_q: this.config.hydrodynamics.X_q || 0.0,
            Y_v: this.config.hydrodynamics.Y_v || 0.0,
            Y_p: this.config.hydrodynamics.Y_p || 0.0,
            Y_r: this.config.hydrodynamics.Y_r || 0.0,
            N_v: this.config.hydrodynamics.N_v || 0.0,
            N_p: this.config.hydrodynamics.N_p || 0.0,
            N_r: this.config.hydrodynamics.N_r || 0.0
        };
    }

    generateControlSurfacesYAML() {
        return {
            naca_number: this.config.control_surfaces.naca_number || "0015",
            control_surfaces: this.config.control_surfaces.control_surfaces.map(surface => ({
                control_surface_type: surface.control_surface_type,
                control_surface_id: surface.control_surface_id,
                control_surface_location: surface.control_surface_location || [0.0, 0.0, 0.0],    // With respect to BODY frame
                control_surface_orientation: surface.control_surface_orientation || [0.0, 0.0, 0.0],  // With respect to BODY frame
                control_surface_area: surface.control_surface_area || 0.0,
                control_surface_NACA: "{naca_number}"
            }))
        };
    }

    generateInitialConditionsYAML() {
        return this.config.initial_conditions;
    }

    generateSensorsYAML() {
        return {
            sensors: this.config.sensors.sensors.map(sensor => ({
                sensor_type: sensor.sensor_type,
                sensor_location: sensor.sensor_location || [0.0, 0.0, 0.0],          // With respect to BODY frame
                sensor_orientation: sensor.sensor_orientation || [1.0, 0.0, 0.0, 0.0],    // With respect to BODY frame
                publish_rate: sensor.publish_rate || 50
            }))
        };
    }

    generateThrustersYAML() {
        return this.config.thrusters;
    }

    generateGuidanceYAML() {
        return this.config.guidance;
    }

    generateControlYAML() {
        return this.config.control;
    }

    loadFromYAML(files) {
        // Load configuration from YAML files
        try {
            const simInput = jsyaml.load(files['simulation_input.yml']);
            const geometry = jsyaml.load(files['geometry.yml']);
            const inertia = jsyaml.load(files['inertia.yml']);
            const hydrodynamics = jsyaml.load(files['hydrodynamics.yml']);
            const controlSurfaces = jsyaml.load(files['control_surfaces.yml']);
            const initialConditions = jsyaml.load(files['initial_conditions.yml']);
            const sensors = jsyaml.load(files['sensors.yml']);

            // Update configuration
            this.config.name = simInput.agents[0].name;
            this.config.type = simInput.agents[0].type;
            this.config.geometry = geometry;
            this.config.inertia = inertia;
            this.config.hydrodynamics = hydrodynamics;
            this.config.control_surfaces = controlSurfaces;
            this.config.initial_conditions = initialConditions;
            this.config.sensors = sensors;

            // Update IDs
            this.nextControlSurfaceId = Math.max(
                ...this.config.control_surfaces.control_surfaces.map(s => s.control_surface_id)
            ) + 1;
            this.nextSensorId = Math.max(
                ...this.config.sensors.sensors.map(s => s.sensor_id)
            ) + 1;

            if (files['thrusters.yml']) {
                const thrusters = jsyaml.load(files['thrusters.yml']);
                this.config.thrusters = thrusters;
                this.nextThrusterId = Math.max(
                    ...this.config.thrusters.thrusters.map(t => t.thruster_id)
                ) + 1;
            }

            return true;
        } catch (error) {
            console.error('Error loading YAML files:', error);
            return false;
        }
    }
}

// Export the class
window.VesselModel = VesselModel; 