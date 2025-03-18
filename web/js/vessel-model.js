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
                naca_file: '',
                delta_max: 35.0,
                deltad_max: 1.0,
                area: 0.1,
                control_surfaces: []
            },
            thrusters: {
                thrusters: []
            },
            initial_conditions: {
                start_velocity: [0, 0, 0, 0, 0, 0], // [u, v, w, p, q, r]
                start_location: [0, 0, 0],          // [x, y, z]
                start_orientation: [0, 0, 0],          // [roll, pitch, yaw]
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
        if (!this.config || !this.config.geometry) {
            console.error("Vessel model configuration is not properly initialized");
            return this;
        }
        
        // Validate dimensions
        if (parseFloat(length) <= 0 || parseFloat(breadth) <= 0 || parseFloat(depth) <= 0) {
            console.warn('Dimensions must be positive values');
            return this;
        }
        
        this.config.geometry.length = parseFloat(length);
        this.config.geometry.breadth = parseFloat(breadth);
        this.config.geometry.depth = parseFloat(depth);
        
        console.log(`Updated vessel dimensions: L=${length}m, B=${breadth}m, D=${depth}m`);
        
        // Update gyration radii based on dimensions
        if (typeof this.updateGyrationRadii === 'function') {
            this.updateGyrationRadii();
        }
        
        // Update inertia matrix
        if (typeof this.updateInertiaMatrix === 'function') {
            this.updateInertiaMatrix();
        }
        
        // Update any UI elements if needed
        if (typeof this.updateUIElements === 'function') {
            this.updateUIElements();
        }
        
        // Save to history
        this.saveToHistory();
        
        return this;
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
        console.log("Updating hydrodynamics with:", coefficients);
        
        // Preserve the hydra_file value
        const hydraFile = this.config.hydrodynamics.hydra_file || '';
        
        // Start with a fresh hydrodynamics object to remove any old coefficients
        this.config.hydrodynamics = {
            hydra_file: hydraFile
        };
        
        // Add all the new coefficients
        for (const [key, value] of Object.entries(coefficients)) {
            // Skip hydra_file as it's already handled
            if (key === 'hydra_file') continue;
            
            if (key === 'dim_flag' || key === 'cross_flow_drag') {
                this.config.hydrodynamics[key] = Boolean(value);
            } else {
                // Any other key is treated as a numeric coefficient
                this.config.hydrodynamics[key] = parseFloat(value);
            }
        }
        
        console.log("Updated hydrodynamics:", this.config.hydrodynamics);
        this.saveToHistory();
    }

    // Set Hydra file path
    setHydraFile(filePath) {
        this.config.hydrodynamics.hydra_file = filePath;
        this.saveToHistory();
    }

    // Update simulation parameters
    updateSimulationParams(params) {
        console.log("Updating simulation parameters with:", params);
        
        // For complete parameter objects, replace the entire simulation config
        if (params.fullReplace) {
            // Remove the fullReplace flag and assign the rest
            const { fullReplace, ...newParams } = params;
            this.config.simulation = newParams;
        } else {
            // For individual updates, only update the specified keys
            for (const [key, value] of Object.entries(params)) {
                this.config.simulation[key] = value;
            }
        }
        
        console.log("Updated simulation config:", this.config.simulation);
        this.saveToHistory();
    }

    // Add a new control surface
    addControlSurface(type, location, orientation, area = 0.1, id = null, naca = '0015', timeConstant = 0.1, deltaMax = 35.0, deltadMax = 1.0) {
        if (!type || !location || !orientation) {
            throw new Error('Missing required parameters for control surface');
        }
        
        const surfaceId = id || this.nextControlSurfaceId++;
        
        const surface = {
            control_surface_type: type,
            control_surface_id: surfaceId,
            control_surface_location: location.map(val => parseFloat(val)),
            control_surface_orientation: orientation.map(val => parseFloat(val)),
            control_surface_area: parseFloat(area),
            control_surface_NACA: naca,
            control_surface_T: parseFloat(timeConstant),
            control_surface_delta_max: parseFloat(deltaMax),
            control_surface_deltad_max: parseFloat(deltadMax)
        };

        // Ensure control_surfaces structure exists
        if (!this.config.control_surfaces) {
            this.config.control_surfaces = {
                naca_number: naca,
                naca_file: '',
                delta_max: 35.0,
                deltad_max: 1.0,
                area: 0.1,
                control_surfaces: []
            };
        }

        this.config.control_surfaces.control_surfaces.push(surface);
        this.saveToHistory();
        return surface.control_surface_id;
    }

    // Add a new thruster
    addThruster(name, location, orientation, id = null, diameter = 0.1, tProp = 1.0, tp = 0.1, jVsKtFile = 'J_vs_KT.csv') {
        if (!location || !orientation) {
            throw new Error('Missing required parameters for thruster');
        }
        
        const thrusterId = id || this.nextThrusterId++;
        const thrusterName = name || `thruster_${thrusterId}`;
        
        const thruster = {
            thruster_name: thrusterName,
            thruster_id: thrusterId,
            thruster_location: location.map(val => parseFloat(val)),
            thruster_orientation: orientation.map(val => parseFloat(val)),
            T_prop: parseFloat(tProp),
            D_prop: parseFloat(diameter),
            tp: parseFloat(tp),
            J_vs_KT: `${this.config.name}/J_vs_KT.csv` // This will be updated with proper path in YAML generation
        };

        // Ensure thrusters structure exists
        if (!this.config.thrusters) {
            this.config.thrusters = { thrusters: [] };
        }
        
        this.config.thrusters.thrusters.push(thruster);
        this.saveToHistory();
        return thruster.thruster_id;
    }

    // Add a new sensor
    addSensor(type, location, orientation, publishRate = 10, useNoneLocation = false, useNoneOrientation = false) {
        if (!type) {
            throw new Error('Missing sensor type');
        }
        
        const sensor = {
            sensor_type: type,
            sensor_id: this.nextSensorId++,
            sensor_location: useNoneLocation ? 'None' : location.map(val => parseFloat(val)),
            sensor_orientation: useNoneOrientation ? 'None' : orientation.map(val => parseFloat(val)),
            publish_rate: parseFloat(publishRate)
        };

        // Ensure sensors structure exists
        if (!this.config.sensors) {
            this.config.sensors = { sensors: [] };
        }

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
    updateInitialConditions(startVelocity, startLocation, startOrientation, useQuaternion = false) {
        // Validate inputs
        if (startVelocity && Array.isArray(startVelocity) && startVelocity.length === 6) {
            this.config.initial_conditions.start_velocity = startVelocity.map(val => parseFloat(val));
        }
        
        if (startLocation && Array.isArray(startLocation) && startLocation.length === 3) {
            this.config.initial_conditions.start_location = startLocation.map(val => parseFloat(val));
        }
        
        if (startOrientation && Array.isArray(startOrientation) && startOrientation.length === 3) {
            this.config.initial_conditions.start_orientation = startOrientation.map(val => parseFloat(val));
        }
        
        this.config.initial_conditions.use_quaternion = !!useQuaternion;
        
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
        if (!surface) {
            console.error(`Control surface with ID ${id} not found`);
            return false;
        }
        
        const validKeys = [
            'control_surface_type', 'control_surface_location', 'control_surface_orientation',
            'control_surface_area', 'control_surface_T'
        ];
        
        console.log(`Updating control surface ${id} with:`, properties);
        
        for (const [key, value] of Object.entries(properties)) {
            if (validKeys.includes(key)) {
                if (key === 'control_surface_location' || key === 'control_surface_orientation') {
                    if (!Array.isArray(value)) {
                        console.error(`Invalid ${key} value:`, value);
                        continue;
                    }
                    
                    const parsedValues = value.map(val => parseFloat(val));
                    console.log(`Setting ${key} to:`, parsedValues);
                    surface[key] = parsedValues;
                } else if (key === 'control_surface_area' || key === 'control_surface_T') {
                    surface[key] = parseFloat(value);
                } else {
                    surface[key] = value;
                }
            }
        }
        
        console.log(`Control surface ${id} after update:`, surface);
        this.saveToHistory();
        return true;
    }

    // Update thruster properties
    updateThruster(id, properties) {
        const thruster = this.getThruster(id);
        if (!thruster) {
            console.error(`Thruster with ID ${id} not found`);
            return false;
        }
        
        const validKeys = [
            'thruster_name', 'thruster_type', 'thruster_location', 'thruster_orientation',
            'D_prop', 'T_prop', 'tp'
        ];
        
        console.log(`Updating thruster ${id} with:`, properties);
        
        for (const [key, value] of Object.entries(properties)) {
            if (validKeys.includes(key)) {
                if (key === 'thruster_location' || key === 'thruster_orientation') {
                    if (!Array.isArray(value)) {
                        console.error(`Invalid ${key} value:`, value);
                        continue;
                    }
                    
                    const parsedValues = value.map(val => parseFloat(val));
                    console.log(`Setting ${key} to:`, parsedValues);
                    thruster[key] = parsedValues;
                } else if (key === 'D_prop' || key === 'T_prop' || key === 'tp') {
                    thruster[key] = parseFloat(value);
                } else {
                    thruster[key] = value;
                }
            }
        }
        
        console.log(`Thruster ${id} after update:`, thruster);
        this.saveToHistory();
        return true;
    }

    // Update sensor properties
    updateSensor(id, properties) {
        const sensor = this.getSensor(id);
        if (!sensor) {
            console.error(`Sensor with ID ${id} not found`);
            return false;
        }
        
        const validKeys = [
            'sensor_type', 'sensor_location', 'sensor_orientation', 'publish_rate'
        ];
        
        console.log(`Updating sensor ${id} with:`, properties);
        
        for (const [key, value] of Object.entries(properties)) {
            if (validKeys.includes(key)) {
                if (key === 'sensor_location' || key === 'sensor_orientation') {
                    if (!Array.isArray(value)) {
                        console.error(`Invalid ${key} value:`, value);
                        continue;
                    }
                    
                    const parsedValues = value.map(val => parseFloat(val));
                    console.log(`Setting ${key} to:`, parsedValues);
                    sensor[key] = parsedValues;
                } else if (key === 'publish_rate') {
                    sensor[key] = parseFloat(value);
                } else {
                    sensor[key] = value;
                }
            }
        }
        
        console.log(`Sensor ${id} after update:`, sensor);
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
    
    // Set model file (3D, Hydra, NACA)
    setModelFile(fileType, file) {
        if (!file) return;
        
        console.log(`Setting ${fileType} file:`, file.name);
        
        if (fileType === 'fbx') {
            // Store the FBX file
            this.modelData.fbxFile = file;
        } else if (fileType === 'hydra') {
            this.modelData.hydraZipFile = file;
        } else if (fileType === 'naca') {
            // Validate file type (should be CSV)
            if (!file.name.toLowerCase().endsWith('.csv')) {
                console.warn(`NACA file should be a CSV file. Received: ${file.type}`);
                // We'll still accept it, but warn
            }
            
            // Store the NACA file
            this.modelData.nacaFile = file;
            
            // Try to parse file name to extract NACA profile number
            const fileNameMatch = file.name.match(/NACA(\d{4})/i);
            if (fileNameMatch && fileNameMatch[1]) {
                // Extract NACA number from file name
                const nacaNumber = fileNameMatch[1];
                this.setNacaProfile(nacaNumber);
            }
            
            console.log(`NACA file set: ${file.name}`);
        }
    }
    
    // Set NACA profile number
    setNacaProfile(nacaNumber) {
        if (!nacaNumber) return;
        
        this.config.control_surfaces.naca_number = nacaNumber;
        console.log(`NACA profile set to: ${nacaNumber}`);
    }
    
    // Get NACA file content (uploaded file or default data)
    getDefaultNacaFile() {
        // If we have a custom NACA file, return its content
        if (this.modelData.nacaFile) {
            console.log(`Reading uploaded NACA file: ${this.modelData.nacaFile.name}`);
            
            // Return a promise that resolves to the file content
            return new Promise((resolve, reject) => {
                const reader = new FileReader();
                
                reader.onload = (e) => {
                    try {
                        const content = e.target.result;
                        
                        // Basic validation to ensure it looks like CSV
                        if (!content.includes(',')) {
                            console.warn('NACA file does not appear to be a valid CSV.');
                            // We'll still use it but warn
                        }
                        
                        console.log(`Successfully read NACA file (${content.length} bytes)`);
                        resolve(content);
                    } catch (err) {
                        console.error('Error processing NACA file content:', err);
                        reject(err);
                    }
                };
                
                reader.onerror = (e) => {
                    console.error('Failed to read NACA file:', e);
                    reject(new Error('Failed to read NACA file'));
                };
                
                reader.readAsText(this.modelData.nacaFile);
            });
        }
        
        // Default NACA 0015 airfoil data (Alpha, CL, CD format)
        console.log('Using default NACA airfoil data');
        // If window.showNotification exists, use it to inform the user
        if (window.showNotification && !this.hasShownDefaultNacaNotification) {
            window.showNotification('Using default NACA 0015 airfoil data. Upload a custom CSV file if needed.', 'info');
            this.hasShownDefaultNacaNotification = true; // Only show once per session
        }
        
        return `Alpha,CL,CD
-15.0,-1.45,0.0528
-12.5,-1.30,0.0347
-10.0,-1.10,0.0233
-7.5,-0.85,0.0141
-5.0,-0.55,0.0086
-2.5,-0.25,0.0060
0.0,0.00,0.0052
2.5,0.25,0.0060
5.0,0.55,0.0086
7.5,0.85,0.0141
10.0,1.10,0.0233
12.5,1.30,0.0347
15.0,1.45,0.0528`;
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