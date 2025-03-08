class VesselModel {
    constructor(name = 'New Vessel', type = 'auv') {
        this.config = {
            name: name,
            type: type,
            geometry: {
                length: 1.0,
                breadth: 0.5,
                depth: 0.5,
                CO: [0, 0, 0],       // Center of origin
                CG: [0, 0, 0],       // Center of gravity
                CB: [0, 0, 0],       // Center of buoyancy
                gyration: [0.5, 0.8, 0.8], // Radii of gyration [kxx, kyy, kzz]
                scale: [1, 1, 1],    // Scale factors for visualization
                geometry_file: ''    // Path to GDF file
            },
            inertia: {
                mass: 50.0,
                buoyancy_mass: 50.0,
                inertia_matrix: [
                    [5, 0, 0],
                    [0, 10, 0],
                    [0, 0, 10]
                ],
                added_mass_matrix: null
            },
            hydrodynamics: {
                hydra_file: '',
                dim_flag: false,
                cross_flow_drag: false,
                X_u_u: -0.01,
                Y_v_v: -0.01,
                Y_r_r: 0.001,
                N_v_v: -0.001,
                N_r_r: -0.001,
                Z_w_w: -0.01
            },
            control_surfaces: {
                naca_number: '0015',
                control_surfaces: []
            },
            thrusters: {
                thrusters: []
            },
            initial_conditions: {
                start_velocity: [0, 0, 0, 0, 0, 0], // [u, v, w, p, q, r]
                start_location: [0, 0, 0],          // [x, y, z]
                start_attitude: [0, 0, 0],          // [roll, pitch, yaw]
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
            },
            simulation: {
                sim_time: 100,
                time_step: 0.01,
                density: 1025,
                gravity: 9.80665,
                world_size: [1000, 1000, 100],
                gps_datum: [12.99300425860631, 80.23913114094384, -87.0]
            }
        };

        // Track component IDs
        this.nextControlSurfaceId = 1;
        this.nextThrusterId = 1;
        this.nextSensorId = 1;
        this.nextWaypointId = 1;
        
        // Track 3D model data
        this.modelData = {
            name: name,
            type: type,
            length: 1.0,
            width: 0.5,
            height: 0.5,
            mass: 50.0,
            controlSurfaces: [],
            thrusters: [],
            sensors: [],
            stlModel: null,
            componentMap: new Map()  // For mapping 3D object UUIDs to components
        };
        
        // Track changes for undo/redo
        this.history = [];
        this.historyIndex = -1;
        this.maxHistorySize = 50;
    }
    
    // Save current state to history
    saveToHistory() {
        // Remove any future states if we're in the middle of the history
        if (this.historyIndex < this.history.length - 1) {
            this.history = this.history.slice(0, this.historyIndex + 1);
        }
        
        // Add current state to history
        const stateCopy = JSON.parse(JSON.stringify(this.config));
        this.history.push(stateCopy);
        
        // Limit history size
        if (this.history.length > this.maxHistorySize) {
            this.history.shift();
        } else {
            this.historyIndex++;
        }
    }
    
    // Undo last change
    undo() {
        if (this.historyIndex > 0) {
            this.historyIndex--;
            this.config = JSON.parse(JSON.stringify(this.history[this.historyIndex]));
            return true;
        }
        return false;
    }
    
    // Redo last undone change
    redo() {
        if (this.historyIndex < this.history.length - 1) {
            this.historyIndex++;
            this.config = JSON.parse(JSON.stringify(this.history[this.historyIndex]));
            return true;
        }
        return false;
    }

    // Set vessel name and type
    setBasicInfo(name, type) {
        if (!name || name.trim() === '') {
            throw new Error('Vessel name cannot be empty');
        }
        
        this.config.name = name.trim();
        
        if (type && ['auv', 'usv', 'ship', 'rov'].includes(type)) {
            this.config.type = type;
        }
        
        this.saveToHistory();
    }

    // Update vessel dimensions and recalculate dependent values
    updateDimensions(length, breadth, depth) {
        if (length <= 0 || breadth <= 0 || depth <= 0) {
            throw new Error('Dimensions must be positive values');
        }
        
        this.config.geometry.length = parseFloat(length);
        this.config.geometry.breadth = parseFloat(breadth);
        this.config.geometry.depth = parseFloat(depth);

        // Update gyration radii based on dimensions
        this.updateGyrationRadii();
        
        // Update inertia matrix
        this.updateInertiaMatrix();
        
        this.saveToHistory();
    }
    
    // Update gyration radii based on vessel type and dimensions
    updateGyrationRadii() {
        const { length, breadth, depth } = this.config.geometry;
        const type = this.config.type;
        
        if (type === 'auv' || type === 'usv') {
            // For AUVs and USVs, use more streamlined values
            this.config.geometry.gyration = [
                breadth * 0.35, // Roll gyration radius (kxx)
                length * 0.25,  // Pitch gyration radius (kyy)
                length * 0.25   // Yaw gyration radius (kzz)
            ];
        } else if (type === 'ship') {
            // For ships, use standard naval architecture approximations
            this.config.geometry.gyration = [
                breadth * 0.4,  // Roll gyration radius (kxx)
                length * 0.25,  // Pitch gyration radius (kyy)
                length * 0.25   // Yaw gyration radius (kzz)
            ];
        } else {
            // Default values for other vessel types
            this.config.geometry.gyration = [
                breadth * 0.35, // Roll gyration radius (kxx)
                length * 0.25,  // Pitch gyration radius (kyy)
                length * 0.25   // Yaw gyration radius (kzz)
            ];
        }
    }

    // Update inertia matrix based on mass and gyration radii
    updateInertiaMatrix() {
        const mass = this.config.inertia.mass;
        const [kxx, kyy, kzz] = this.config.geometry.gyration;
        
        // Calculate moments of inertia (I = m * k²)
        const Ixx = mass * kxx * kxx;
        const Iyy = mass * kyy * kyy;
        const Izz = mass * kzz * kzz;
        
        // Update inertia matrix (assuming no products of inertia)
        this.config.inertia.inertia_matrix = [
            [Ixx, 0, 0],
            [0, Iyy, 0],
            [0, 0, Izz]
        ];
    }

    // Update physical properties
    updatePhysicalProperties(mass, buoyancy_mass, CG, CB) {
        if (mass <= 0) {
            throw new Error('Mass must be positive');
        }
        
        this.config.inertia.mass = parseFloat(mass);
        
        if (buoyancy_mass !== undefined && buoyancy_mass > 0) {
            this.config.inertia.buoyancy_mass = parseFloat(buoyancy_mass);
        } else {
            // By default, make buoyancy mass equal to mass (neutrally buoyant)
            this.config.inertia.buoyancy_mass = this.config.inertia.mass;
        }
        
        if (CG && Array.isArray(CG) && CG.length === 3) {
            this.config.geometry.CG = CG.map(val => parseFloat(val));
        }
        
        if (CB && Array.isArray(CB) && CB.length === 3) {
            this.config.geometry.CB = CB.map(val => parseFloat(val));
        }
        
        // Update inertia matrix with new mass
        this.updateInertiaMatrix();
        
        this.saveToHistory();
    }

    // Update hydrodynamic coefficients
    updateHydrodynamics(coefficients) {
        const validKeys = [
            'dim_flag', 'cross_flow_drag',
            'X_u_u', 'Y_v_v', 'Y_r_r', 'N_v_v', 'N_r_r', 'Z_w_w'
        ];
        
        for (const [key, value] of Object.entries(coefficients)) {
            if (validKeys.includes(key)) {
                if (key === 'dim_flag' || key === 'cross_flow_drag') {
                    this.config.hydrodynamics[key] = Boolean(value);
                } else {
                    this.config.hydrodynamics[key] = parseFloat(value);
                }
            }
        }
        
        this.saveToHistory();
    }

    // Set Hydra file path
    setHydraFile(filePath) {
        this.config.hydrodynamics.hydra_file = filePath;
        this.saveToHistory();
    }

    // Update simulation parameters
    updateSimulationParams(params) {
        const validKeys = [
            'sim_time', 'time_step', 'density', 'gravity', 'world_size', 'gps_datum'
        ];
        
        for (const [key, value] of Object.entries(params)) {
            if (validKeys.includes(key)) {
                this.config.simulation[key] = value;
            }
        }
        
        this.saveToHistory();
    }

    // Add a new control surface
    addControlSurface(type, location, orientation, area = 0.1) {
        if (!type || !location || !orientation) {
            throw new Error('Missing required parameters for control surface');
        }
        
        const surface = {
            control_surface_type: type,
            control_surface_id: this.nextControlSurfaceId++,
            control_surface_location: location.map(val => parseFloat(val)),
            control_surface_orientation: orientation.map(val => parseFloat(val)),
            control_surface_area: parseFloat(area),
            control_surface_T: 0.1  // Default time constant
        };

        this.config.control_surfaces.control_surfaces.push(surface);
        this.saveToHistory();
        return surface.control_surface_id;
    }

    // Add a new thruster
    addThruster(type, location, orientation, diameter = 0.2) {
        if (!location || !orientation) {
            throw new Error('Missing required parameters for thruster');
        }
        
        const thruster = {
            thruster_name: `thruster_${this.nextThrusterId}`,
            thruster_id: this.nextThrusterId++,
            thruster_type: type || 'Propeller',
            thruster_location: location.map(val => parseFloat(val)),
            thruster_orientation: orientation.map(val => parseFloat(val)),
            D_prop: parseFloat(diameter),  // Propeller diameter in meters
            T_prop: 1.0,  // Thrust coefficient
            tp: 0.1       // Time constant
        };

        if (!this.config.thrusters) {
            this.config.thrusters = { thrusters: [] };
        }
        
        this.config.thrusters.thrusters.push(thruster);
        this.saveToHistory();
        return thruster.thruster_id;
    }

    // Add a new sensor
    addSensor(type, location, orientation, publishRate = 10) {
        if (!type || !location || !orientation) {
            throw new Error('Missing required parameters for sensor');
        }
        
        const sensor = {
            sensor_type: type,
            sensor_id: this.nextSensorId++,
            sensor_location: location.map(val => parseFloat(val)),
            sensor_orientation: orientation.map(val => parseFloat(val)),
            publish_rate: parseFloat(publishRate)
        };

        this.config.sensors.sensors.push(sensor);
        this.saveToHistory();
        return sensor.sensor_id;
    }
    
    // Add a new waypoint
    addWaypoint(x, y, z, psi = 0) {
        const waypoint = {
            id: this.nextWaypointId++,
            x: parseFloat(x),
            y: parseFloat(y),
            z: parseFloat(z),
            psi: parseFloat(psi)
        };
        
        if (!this.config.guidance.waypoints) {
            this.config.guidance.waypoints = [];
        }
        
        this.config.guidance.waypoints.push(waypoint);
        this.saveToHistory();
        return waypoint.id;
    }

    // Update initial conditions
    updateInitialConditions(startVelocity, startLocation, startAttitude, useQuaternion = false) {
        if (startVelocity && Array.isArray(startVelocity) && startVelocity.length === 6) {
            this.config.initial_conditions.start_velocity = startVelocity.map(val => parseFloat(val));
        }
        
        if (startLocation && Array.isArray(startLocation) && startLocation.length === 3) {
            this.config.initial_conditions.start_location = startLocation.map(val => parseFloat(val));
        }
        
        if (startAttitude && Array.isArray(startAttitude) && startAttitude.length === 3) {
            this.config.initial_conditions.start_attitude = startAttitude.map(val => parseFloat(val));
        }
        
        this.config.initial_conditions.use_quaternion = Boolean(useQuaternion);
        
        this.saveToHistory();
    }

    // Update control types
    updateControlTypes(surfaceControlType, thrusterControlType) {
        if (surfaceControlType) {
            this.config.control.control_surface_control_type = surfaceControlType;
        }
        
        if (thrusterControlType) {
            this.config.control.thruster_control_type = thrusterControlType;
        }
        
        this.saveToHistory();
    }

    // Get control surface by ID
    getControlSurface(id) {
        return this.config.control_surfaces.control_surfaces.find(
            surface => surface.control_surface_id === id
        );
    }

    // Get thruster by ID
    getThruster(id) {
        if (!this.config.thrusters || !this.config.thrusters.thrusters) return null;
        return this.config.thrusters.thrusters.find(
            thruster => thruster.thruster_id === id
        );
    }

    // Get sensor by ID
    getSensor(id) {
        return this.config.sensors.sensors.find(
            sensor => sensor.sensor_id === id
        );
    }
    
    // Get waypoint by ID
    getWaypoint(id) {
        if (!this.config.guidance.waypoints) return null;
        return this.config.guidance.waypoints.find(
            waypoint => waypoint.id === id
        );
    }

    // Update control surface properties
    updateControlSurface(id, properties) {
        const surface = this.getControlSurface(id);
        if (!surface) return false;
        
        const validKeys = [
            'control_surface_type', 'control_surface_location', 'control_surface_orientation',
            'control_surface_area', 'control_surface_T'
        ];
        
        for (const [key, value] of Object.entries(properties)) {
            if (validKeys.includes(key)) {
                if (key === 'control_surface_location' || key === 'control_surface_orientation') {
                    surface[key] = value.map(val => parseFloat(val));
                } else if (key === 'control_surface_area' || key === 'control_surface_T') {
                    surface[key] = parseFloat(value);
                } else {
                    surface[key] = value;
                }
            }
        }
        
        this.saveToHistory();
        return true;
    }

    // Update thruster properties
    updateThruster(id, properties) {
        const thruster = this.getThruster(id);
        if (!thruster) return false;
        
        const validKeys = [
            'thruster_name', 'thruster_type', 'thruster_location', 'thruster_orientation',
            'D_prop', 'T_prop', 'tp'
        ];
        
        for (const [key, value] of Object.entries(properties)) {
            if (validKeys.includes(key)) {
                if (key === 'thruster_location' || key === 'thruster_orientation') {
                    thruster[key] = value.map(val => parseFloat(val));
                } else if (key === 'D_prop' || key === 'T_prop' || key === 'tp') {
                    thruster[key] = parseFloat(value);
                } else {
                    thruster[key] = value;
                }
            }
        }
        
        this.saveToHistory();
        return true;
    }

    // Update sensor properties
    updateSensor(id, properties) {
        const sensor = this.getSensor(id);
        if (!sensor) return false;
        
        const validKeys = [
            'sensor_type', 'sensor_location', 'sensor_orientation', 'publish_rate'
        ];
        
        for (const [key, value] of Object.entries(properties)) {
            if (validKeys.includes(key)) {
                if (key === 'sensor_location' || key === 'sensor_orientation') {
                    sensor[key] = value.map(val => parseFloat(val));
                } else if (key === 'publish_rate') {
                    sensor[key] = parseFloat(value);
                } else {
                    sensor[key] = value;
                }
            }
        }
        
        this.saveToHistory();
        return true;
    }
    
    // Update waypoint properties
    updateWaypoint(id, properties) {
        const waypoint = this.getWaypoint(id);
        if (!waypoint) return false;
        
        const validKeys = ['x', 'y', 'z', 'psi'];
        
        for (const [key, value] of Object.entries(properties)) {
            if (validKeys.includes(key)) {
                waypoint[key] = parseFloat(value);
            }
        }
        
        this.saveToHistory();
        return true;
    }

    // Remove a control surface
    removeControlSurface(id) {
        const index = this.config.control_surfaces.control_surfaces.findIndex(
            surface => surface.control_surface_id === id
        );
        
        if (index !== -1) {
            this.config.control_surfaces.control_surfaces.splice(index, 1);
            this.saveToHistory();
            return true;
        }
        
        return false;
    }

    // Remove a thruster
    removeThruster(id) {
        if (!this.config.thrusters || !this.config.thrusters.thrusters) return false;
        
        const index = this.config.thrusters.thrusters.findIndex(
            thruster => thruster.thruster_id === id
        );
        
        if (index !== -1) {
            this.config.thrusters.thrusters.splice(index, 1);
            this.saveToHistory();
            return true;
        }
        
        return false;
    }

    // Remove a sensor
    removeSensor(id) {
        const index = this.config.sensors.sensors.findIndex(
            sensor => sensor.sensor_id === id
        );
        
        if (index !== -1) {
            this.config.sensors.sensors.splice(index, 1);
            this.saveToHistory();
            return true;
        }
        
        return false;
    }
    
    // Remove a waypoint
    removeWaypoint(id) {
        if (!this.config.guidance.waypoints) return false;
        
        const index = this.config.guidance.waypoints.findIndex(
            waypoint => waypoint.id === id
        );
        
        if (index !== -1) {
            this.config.guidance.waypoints.splice(index, 1);
            this.saveToHistory();
            return true;
        }
        
        return false;
    }
    
    // Map 3D model component to vessel component
    mapModelComponent(uuid, component) {
        if (!component.type) {
            console.error('Component type must be specified');
            return false;
        }
        
        this.modelData.componentMap.set(uuid, component);
        console.log(`Mapped component ${component.name} as ${component.type}`);
        return true;
    }
    
    // Remove mapping for a 3D model component
    unmapModelComponent(uuid) {
        if (this.modelData.componentMap.has(uuid)) {
            const component = this.modelData.componentMap.get(uuid);
            console.log(`Unmapped component ${component.name}`);
            this.modelData.componentMap.delete(uuid);
            return true;
        }
        return false;
    }
    
    // Get component data for a 3D model object
    getModelComponentData(uuid) {
        return this.modelData.componentMap.get(uuid);
    }
    
    // Check if a 3D model object is mapped to a component
    isModelComponentMapped(uuid) {
        return this.modelData.componentMap.has(uuid);
    }
    
    // Set 3D model file
    setModelFile(fileType, file) {
        if (fileType === 'fbx') {
            this.modelData.fbxFile = file;
        } else if (fileType === 'stl') {
            this.modelData.stlFile = file;
        } else if (fileType === 'gdf') {
            this.modelData.gdfFile = file;
            this.config.geometry.geometry_file = file.name;
        } else if (fileType === 'hydra') {
            this.modelData.hydraZipFile = file;
        }
    }
    
    // Create default NACA file content for control surfaces
    getDefaultNacaFile() {
        // NACA 0015 airfoil data (x, y coordinates)
        return `x,y
0.0000,0.0000
0.0125,0.0315
0.0250,0.0436
0.0500,0.0600
0.0750,0.0722
0.1000,0.0816
0.1500,0.0947
0.2000,0.1033
0.2500,0.1086
0.3000,0.1111
0.3500,0.1110
0.4000,0.1086
0.4500,0.1039
0.5000,0.0972
0.5500,0.0887
0.6000,0.0786
0.6500,0.0675
0.7000,0.0555
0.7500,0.0433
0.8000,0.0312
0.8500,0.0199
0.9000,0.0102
0.9500,0.0025
1.0000,0.0000
0.9500,-0.0025
0.9000,-0.0102
0.8500,-0.0199
0.8000,-0.0312
0.7500,-0.0433
0.7000,-0.0555
0.6500,-0.0675
0.6000,-0.0786
0.5500,-0.0887
0.5000,-0.0972
0.4500,-0.1039
0.4000,-0.1086
0.3500,-0.1110
0.3000,-0.1111
0.2500,-0.1086
0.2000,-0.1033
0.1500,-0.0947
0.1000,-0.0816
0.0750,-0.0722
0.0500,-0.0600
0.0250,-0.0436
0.0125,-0.0315
0.0000,0.0000`;
    }

    // Export the vessel configuration as JSON
    exportAsJson() {
        return JSON.stringify(this.config, null, 2);
    }

    // Import vessel configuration from JSON
    importFromJson(jsonString) {
        try {
            const config = JSON.parse(jsonString);
            
            // Validate essential structure
            if (!config.name || !config.geometry || !config.inertia) {
                throw new Error('Invalid vessel configuration format');
            }
            
            this.config = config;
            
            // Reset component IDs
            this.resetComponentIds();
            
            // Save to history
            this.saveToHistory();
            
            return true;
        } catch (error) {
            console.error('Error importing vessel configuration:', error);
            return false;
        }
    }
    
    // Reset component IDs to ensure they're sequential after import
    resetComponentIds() {
        // Reset control surface IDs
        if (this.config.control_surfaces && this.config.control_surfaces.control_surfaces) {
            let maxId = 0;
            
            this.config.control_surfaces.control_surfaces.forEach(surface => {
                if (surface.control_surface_id > maxId) {
                    maxId = surface.control_surface_id;
                }
            });
            
            this.nextControlSurfaceId = maxId + 1;
        } else {
            this.nextControlSurfaceId = 1;
        }
        
        // Reset thruster IDs
        if (this.config.thrusters && this.config.thrusters.thrusters) {
            let maxId = 0;
            
            this.config.thrusters.thrusters.forEach(thruster => {
                if (thruster.thruster_id > maxId) {
                    maxId = thruster.thruster_id;
                }
            });
            
            this.nextThrusterId = maxId + 1;
        } else {
            this.nextThrusterId = 1;
        }
        
        // Reset sensor IDs
        if (this.config.sensors && this.config.sensors.sensors) {
            let maxId = 0;
            
            this.config.sensors.sensors.forEach(sensor => {
                if (sensor.sensor_id > maxId) {
                    maxId = sensor.sensor_id;
                }
            });
            
            this.nextSensorId = maxId + 1;
        } else {
            this.nextSensorId = 1;
        }
        
        // Reset waypoint IDs
        if (this.config.guidance && this.config.guidance.waypoints) {
            let maxId = 0;
            
            this.config.guidance.waypoints.forEach(waypoint => {
                if (waypoint.id > maxId) {
                    maxId = waypoint.id;
                }
            });
            
            this.nextWaypointId = maxId + 1;
        } else {
            this.nextWaypointId = 1;
        }
    }
    
    // Generate YAML files
    generateYAMLFiles() {
        const yamlGenerator = new YAMLGenerator();
        return yamlGenerator.generateAllYAMLFiles(this);
    }
    
    // Create a ZIP file with YAML files
    async createYAMLZip() {
        const yamlGenerator = new YAMLGenerator();
        return await yamlGenerator.createZipFile(this);
    }

    // Methods for handling FBX model components
    getAllComponentsByType(type) {
        const components = [];
        this.modelData.componentMap.forEach((component, uuid) => {
            if (component.type === type) {
                components.push({uuid, ...component});
            }
        });
        return components;
    }
}

// Export the class
window.VesselModel = VesselModel; 