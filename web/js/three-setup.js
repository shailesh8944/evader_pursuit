class ThreeScene {
    constructor() {
        // Initialize properties
        this.scene = new THREE.Scene();
        this.camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.1, 1000);
        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.selectedObject = null;
        this.raycaster = new THREE.Raycaster();
        this.mouse = new THREE.Vector2();
        this.needsRender = true;
        this.modelMeshes = new Map();
        
        // Initialize component maps
        this.vessel = null;
        this.controlSurfaces = new Map();
        this.thrusters = new Map();
        this.sensors = new Map();
        this.transformMode = 'translate';
        this.transformControls = null;
        
        // Ensure DOM is loaded before initializing
        if (document.readyState === 'loading') {
            document.addEventListener('DOMContentLoaded', () => {
                this.init();
            });
        } else {
            try {
                this.init();
            } catch (error) {
                console.error('Error initializing ThreeScene:', error);
                // Attempt delayed initialization as a fallback
                setTimeout(() => {
                    try {
                        this.init();
                    } catch (retryError) {
                        console.error('Failed to initialize ThreeScene after retry:', retryError);
                    }
                }, 500);
            }
        }
    }

    init() {
        // Get container
        this.container = document.getElementById('viewer3D');
        
        // Initialize scene
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x1e293b);
        
        // Create renderer with hardware acceleration
        this.renderer = new THREE.WebGLRenderer({ 
            antialias: true,
            powerPreference: 'high-performance',
            precision: 'highp'
        });
        this.renderer.setPixelRatio(window.devicePixelRatio); // Adapt to device
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        this.renderer.outputEncoding = THREE.sRGBEncoding;
        this.renderer.physicallyCorrectLights = true;
        this.container.appendChild(this.renderer.domElement);
        
        // Performance optimization - set fixed size buffer based on container size
        const containerRect = this.container.getBoundingClientRect();
        this.renderer.setSize(containerRect.width, containerRect.height);
        
        // Initialize camera
        this.camera = new THREE.PerspectiveCamera(45, containerRect.width / containerRect.height, 0.1, 1000);
        this.camera.position.set(5, 5, 5);
        this.camera.lookAt(0, 0, 0);
        
        // Initialize orbit controls
        this.orbitControls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
        this.orbitControls.enableDamping = true;
        this.orbitControls.dampingFactor = 0.25;
        this.orbitControls.screenSpacePanning = false;
        this.orbitControls.maxPolarAngle = Math.PI / 1.5;
        this.orbitControls.update();
        
        // Initialize transform controls
        this.transformControls = new THREE.TransformControls(this.camera, this.renderer.domElement);
        this.transformControls.addEventListener('dragging-changed', (event) => {
            this.orbitControls.enabled = !event.value;
        });
        this.scene.add(this.transformControls);
        
        // Setup raycaster for object selection
        this.raycaster = new THREE.Raycaster();
        this.mouse = new THREE.Vector2();
        
        // Add basic scene elements
        this.addGrid();
        this.addLights();
        
        // Initialize event listeners
        this.initializeEventListeners();
        
        // Add performance monitoring
        if (window.location.search.includes('stats=1')) {
            this.addPerformanceMonitor();
        }
        
        // Flag for controlling render frequency
        this.needsRender = true;
        
        // Start animation loop
        this.animate();
    }

    addGrid() {
        // Remove existing grid if any
        if (this.grid) {
            this.scene.remove(this.grid);
        }
        
        // Create a grid helper
        const size = 20;
        const divisions = 20;
        this.grid = new THREE.GridHelper(size, divisions);
        this.grid.material.opacity = 0.2;
        this.grid.material.transparent = true;
        this.scene.add(this.grid);
    }

    addLights() {
        // Add ambient light
        const ambientLight = new THREE.AmbientLight(0x404040, 0.5);
        this.scene.add(ambientLight);

        // Add directional lights from different angles
        const createDirLight = (x, y, z) => {
            const dirLight = new THREE.DirectionalLight(0xffffff, 0.5);
            dirLight.position.set(x, y, z);
            return dirLight;
        };

        this.scene.add(createDirLight(5, 5, 5));
        this.scene.add(createDirLight(-5, 5, -5));
        this.scene.add(createDirLight(0, -5, 0));
        
        // Add a hemisphere light
        const hemiLight = new THREE.HemisphereLight(0xffffff, 0x404040, 0.6);
        this.scene.add(hemiLight);
    }

    async loadModelGeometry(file) {
        try {
            console.log('Starting model file load...');
            
            // Remove old vessel if it exists
            if (this.vessel) {
                this.scene.remove(this.vessel);
            }

            // Create main vessel group
            this.vessel = new THREE.Group();
            this.vessel.userData.type = 'vessel';
            this.vessel.userData.isTransformable = true;

            // Load STL file
            const loader = new THREE.STLLoader();
            const geometry = await new Promise((resolve, reject) => {
                loader.load(
                    URL.createObjectURL(file),
                    resolve,
                    undefined,
                    reject
                );
            });

            console.log('STL loaded, vertex count:', geometry.attributes.position.count);

            // Compute vertex normals for proper lighting
            geometry.computeVertexNormals();

            // Calculate bounding box and center
            geometry.computeBoundingBox();
            const bbox = geometry.boundingBox;
            const size = new THREE.Vector3();
            bbox.getSize(size);
            const center = new THREE.Vector3();
            bbox.getCenter(center);

            // Calculate scale to normalize size
            const maxDim = Math.max(size.x, size.y, size.z);
            const scale = 2 / maxDim;

            // Create a container group for the STL and its bounding box
            const modelGroup = new THREE.Group();
            modelGroup.userData.type = 'modelGroup';
            modelGroup.userData.isTransformable = true;

            // Create STL mesh with a bright, solid material
            const stlMaterial = new THREE.MeshStandardMaterial({
                color: 0x3498db,
                metalness: 0.3,
                roughness: 0.4,
                side: THREE.DoubleSide
            });

            const stlMesh = new THREE.Mesh(geometry, stlMaterial);
            stlMesh.castShadow = true;
            stlMesh.receiveShadow = true;
            
            // Center the geometry at origin
            geometry.center();
            
            // Create bounding box mesh
            const boxGeometry = new THREE.BoxGeometry(
                size.x,
                size.y,
                size.z
            );
            const boxMaterial = new THREE.LineBasicMaterial({
                color: 0x00ff00,
                transparent: true,
                opacity: 0.5
            });
            const boxMesh = new THREE.LineSegments(
                new THREE.EdgesGeometry(boxGeometry),
                boxMaterial
            );

            // Add both meshes to model group
            modelGroup.add(stlMesh);
            modelGroup.add(boxMesh);

            // Scale the entire model group
            modelGroup.scale.multiplyScalar(scale);

            // Create vessel coordinate axes (marine convention)
            const axesLength = maxDim * scale * 0.5;
            const axesGroup = new THREE.Group();
            axesGroup.userData.isAxes = true;

            // X-axis (surge - forward) - Red
            const xAxis = new THREE.ArrowHelper(
                new THREE.Vector3(1, 0, 0),
                new THREE.Vector3(0, 0, 0),
                axesLength,
                0xff0000,
                axesLength * 0.2,
                axesLength * 0.1
            );
            axesGroup.add(xAxis);

            // Y-axis (sway - starboard) - Green
            const yAxis = new THREE.ArrowHelper(
                new THREE.Vector3(0, 1, 0),
                new THREE.Vector3(0, 0, 0),
                axesLength,
                0x00ff00,
                axesLength * 0.2,
                axesLength * 0.1
            );
            axesGroup.add(yAxis);

            // Z-axis (heave - down) - Blue
            const zAxis = new THREE.ArrowHelper(
                new THREE.Vector3(0, 0, 1),
                new THREE.Vector3(0, 0, 0),
                axesLength,
                0x0000ff,
                axesLength * 0.2,
                axesLength * 0.1
            );
            axesGroup.add(zAxis);

            // Add model group and axes to vessel group
            this.vessel.add(modelGroup);
            this.vessel.add(axesGroup);

            // Store references
            this.vessel.userData.stlMesh = stlMesh;
            this.vessel.userData.boxMesh = boxMesh;
            this.vessel.userData.axesGroup = axesGroup;
            this.vessel.userData.modelGroup = modelGroup;
            this.vessel.userData.isSTLModel = true;

            // Add vessel to scene
            this.scene.add(this.vessel);

            // Make the vessel selectable initially
            this.selectObject(this.vessel);

            // Reset camera and reattach components
            this.resetCamera();
            this.reattachComponents();

            console.log('Model loaded successfully:', {
                vertices: geometry.attributes.position.count,
                size: size.toArray(),
                scale: scale,
                center: center.toArray()
            });

            // Add more lighting to better see the model
            if (!this.spotLight) {
                this.spotLight = new THREE.SpotLight(0xffffff, 1);
                this.spotLight.position.set(5, 5, 5);
                this.spotLight.angle = Math.PI / 4;
                this.spotLight.penumbra = 0.1;
                this.spotLight.decay = 2;
                this.spotLight.distance = 200;
                this.scene.add(this.spotLight);
            }

            // Set initial visibility states
            const stlToggle = document.getElementById('toggleSTLVisibility');
            const boxToggle = document.getElementById('toggleBoxVisibility');
            if (stlToggle) stlToggle.checked = true;
            if (boxToggle) boxToggle.checked = true;

            this.render();
            this.updateSceneHierarchy();
            return {
                success: true,
                dimensions: {
                    length: size.x,
                    breadth: size.y,
                    depth: size.z
                }
            };
        } catch (error) {
            console.error('Error loading STL file:', error);
            return {
                success: false,
                error: error.message
            };
        }
    }

    createVessel(length, breadth, depth) {
        try {
            console.log(`Creating vessel with dimensions: ${length} x ${breadth} x ${depth}`);
            
            // Don't create default vessel if we have an STL model loaded
            if (this.vessel && this.vessel.userData.isSTLModel) {
                console.log('Skipping default vessel creation as STL model is loaded');
                return;
            }

            // Remove existing vessel if any
            if (this.vessel) {
                this.scene.remove(this.vessel);
            }

            // Create vessel container group
            this.vessel = new THREE.Group();
            this.vessel.userData.type = 'vessel';
            this.vessel.userData.isSTLModel = false;
            this.vessel.name = 'Vessel';

            // Create vessel box for visualization
            const geometry = new THREE.BoxGeometry(length, depth, breadth);
            const material = new THREE.MeshPhongMaterial({
                color: 0x3c78d8,
                transparent: true,
                opacity: 0.7,
                side: THREE.DoubleSide
            });
            const vesselMesh = new THREE.Mesh(geometry, material);
            vesselMesh.userData.type = 'vesselBody';
            vesselMesh.name = 'Hull';

            // Add wireframe
            const edgeGeometry = new THREE.EdgesGeometry(geometry);
            const edgeMaterial = new THREE.LineBasicMaterial({ color: 0x000000 });
            const wireframe = new THREE.LineSegments(edgeGeometry, edgeMaterial);
            wireframe.userData.isWireframe = true;
            vesselMesh.add(wireframe);

            // Store original size
            this.vessel.userData.dimensions = { length, breadth, depth };

            // Add mesh to vessel container
            this.vessel.add(vesselMesh);

            // Add vessel to scene
            this.scene.add(this.vessel);

            // Add local coordinate axes
            const axes = this.createLocalAxes(Math.max(length, breadth, depth) * 0.5);
            this.vessel.add(axes);

            // Reattach components if any
            this.reattachComponents();

            // Update scene
            this.render();
            this.updateSceneHierarchy();
            
            return this.vessel;
        } catch (error) {
            console.error('Error creating vessel:', error);
            return null;
        }
    }

    updateVesselDimensions(length, breadth, depth) {
        if (!this.vessel || this.vessel.userData.isSTLModel) {
            return;
        }

        // Update stored dimensions
        this.vessel.userData.dimensions = {
            length: length || 1,
            breadth: breadth || 1,
            depth: depth || 1
        };

        // Update vessel mesh geometry
        const vesselMesh = this.vessel.userData.vesselMesh;
        if (vesselMesh) {
            const newGeometry = new THREE.BoxGeometry(length || 1, depth || 1, breadth || 1);
            vesselMesh.geometry.dispose();
            vesselMesh.geometry = newGeometry;
        }

        // Update axes size
        const axesGroup = this.vessel.userData.axesGroup;
        if (axesGroup) {
            const axesLength = Math.max(length, breadth, depth) * 0.5;
            axesGroup.children.forEach((axis, index) => {
                if (axis instanceof THREE.ArrowHelper) {
                    axis.setLength(axesLength, axesLength * 0.2, axesLength * 0.1);
                }
            });
        }

        // Reposition components based on new dimensions
        this.reattachComponents();

        // Update scene
        this.render();
        this.updateSceneHierarchy();
    }

    reattachComponents() {
        try {
            // Only proceed if vessel exists
            if (!this.vessel) {
                console.warn('Cannot reattach components: vessel does not exist');
                return;
            }
            
            // Reattach control surfaces
            if (this.controlSurfaces) {
                this.controlSurfaces.forEach((surface, id) => {
                    if (surface.parent) {
                        surface.parent.remove(surface);
                    }
                    this.vessel.add(surface);
                });
            }

            // Reattach thrusters
            if (this.thrusters) {
                this.thrusters.forEach((thruster, id) => {
                    if (thruster.parent) {
                        thruster.parent.remove(thruster);
                    }
                    this.vessel.add(thruster);
                });
            }

            // Reattach sensors
            if (this.sensors) {
                this.sensors.forEach((sensor, id) => {
                    if (sensor.parent) {
                        sensor.parent.remove(sensor);
                    }
                    this.vessel.add(sensor);
                });
            }
            
            // Mark for rendering
            this.render();
        } catch (error) {
            console.error('Error in reattachComponents:', error);
        }
    }

    createLocalAxes(size = 0.3) {
        const axes = new THREE.Group();
        axes.userData.isAxes = true;
        
        // Create a container for all axes elements
        const axesContainer = new THREE.Group();
        
        // X axis (red)
        const xGeometry = new THREE.CylinderGeometry(0.01, 0.01, size);
        const xMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 });
        const xAxis = new THREE.Mesh(xGeometry, xMaterial);
        xAxis.rotation.z = -Math.PI / 2;
        xAxis.position.x = size / 2;
        axesContainer.add(xAxis);

        // Y axis (green)
        const yGeometry = new THREE.CylinderGeometry(0.01, 0.01, size);
        const yMaterial = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
        const yAxis = new THREE.Mesh(yGeometry, yMaterial);
        yAxis.position.y = size / 2;
        axesContainer.add(yAxis);

        // Z axis (blue)
        const zGeometry = new THREE.CylinderGeometry(0.01, 0.01, size);
        const zMaterial = new THREE.MeshBasicMaterial({ color: 0x0000ff });
        const zAxis = new THREE.Mesh(zGeometry, zMaterial);
        zAxis.rotation.x = Math.PI / 2;
        zAxis.position.z = size / 2;
        axesContainer.add(zAxis);

        // Add labels
        const createLabel = (text, position, color) => {
            const canvas = document.createElement('canvas');
            const context = canvas.getContext('2d');
            canvas.width = 64;
            canvas.height = 32;
            context.fillStyle = color;
            context.font = 'bold 16px Arial';
            context.fillText(text, 0, 24);
            
            const texture = new THREE.CanvasTexture(canvas);
            const spriteMaterial = new THREE.SpriteMaterial({ map: texture });
            const sprite = new THREE.Sprite(spriteMaterial);
            sprite.position.copy(position);
            sprite.scale.set(0.5, 0.25, 1);
            return sprite;
        };

        axesContainer.add(createLabel('X', new THREE.Vector3(size * 1.2, 0, 0), '#ff0000'));
        axesContainer.add(createLabel('Y', new THREE.Vector3(0, size * 1.2, 0), '#00ff00'));
        axesContainer.add(createLabel('Z', new THREE.Vector3(0, 0, size * 1.2), '#0000ff'));

        axes.add(axesContainer);
        return axes;
    }

    addComponentAxes(component) {
        const axes = this.createLocalAxes();
        axes.userData.isAxes = true;
        axes.userData.parentId = component.userData.id;
        component.add(axes);  // Add axes directly to the component
        return axes;
    }

    addControlSurface(id, type, position, orientation, area = 0.1) {
        // Create a simple mesh to represent the control surface
        const width = Math.sqrt(area);
        const height = width;
        const depth = width * 0.1;
        
        const geometry = new THREE.BoxGeometry(width, height, depth);
        const material = new THREE.MeshStandardMaterial({ 
            color: 0x6699cc, 
            transparent: true,
            opacity: 0.7,
            side: THREE.DoubleSide
        });
        
        const controlSurface = new THREE.Mesh(geometry, material);
        controlSurface.name = `${type}_${id}`;
        controlSurface.userData.type = 'controlSurface';
        controlSurface.userData.componentId = id;
        
        // Position and orient the control surface
        controlSurface.position.set(position[0], position[1], position[2]);
        controlSurface.rotation.set(orientation[0], orientation[1], orientation[2]);
        
        // Add to scene
        this.scene.add(controlSurface);
        
        // Store reference
        if (!this.controlSurfaces) {
            this.controlSurfaces = new Map();
        }
        this.controlSurfaces.set(id, controlSurface);
        
        this.render();
        return controlSurface;
    }

    updateControlSurfaceDimensions(id, width, height) {
        const surface = this.getObjectById(id);
        if (surface && surface.userData.type === 'control_surface') {
            const mesh = surface.children.find(child => child.userData.isControlSurface);
            if (mesh) {
                const geometry = new THREE.PlaneGeometry(width, height);
                mesh.geometry.dispose();
                mesh.geometry = geometry;
                surface.userData.dimensions = { width, height };
                surface.userData.parameters.area = width * height;
                this.render();
            }
        }
    }

    updateControlSurfaceParameters(id, parameters) {
        const surface = this.getObjectById(id);
        if (surface && surface.userData.type === 'control_surface') {
            surface.userData.parameters = {
                ...surface.userData.parameters,
                ...parameters
            };
            // Update visual representation if needed
            if (parameters.width || parameters.height) {
                this.updateControlSurfaceDimensions(id, 
                    parameters.width || surface.userData.dimensions.width,
                    parameters.height || surface.userData.dimensions.height
                );
            }
            this.render();
        }
    }

    addThruster(id, position, orientation, diameter = 0.2) {
        // Create a cylinder to represent the thruster
        const geometry = new THREE.CylinderGeometry(diameter / 2, diameter / 2, diameter, 16);
        const material = new THREE.MeshStandardMaterial({ 
            color: 0xcc6633, 
            transparent: true,
            opacity: 0.7 
        });
        
        const thruster = new THREE.Mesh(geometry, material);
        thruster.name = `Thruster_${id}`;
        thruster.userData.type = 'thruster';
        thruster.userData.componentId = id;
        
        // Position and orient the thruster
        thruster.position.set(position[0], position[1], position[2]);
        
        // Adjust rotation for proper orientation
        // Cylinders in Three.js have their axis along y, but thrusters typically have their axis along z
        thruster.rotation.x = Math.PI / 2;
        thruster.rotation.x += orientation[0];
        thruster.rotation.y += orientation[1];
        thruster.rotation.z += orientation[2];
        
        // Add to scene
        this.scene.add(thruster);
        
        // Store reference
        if (!this.thrusters) {
            this.thrusters = new Map();
        }
        this.thrusters.set(id, thruster);
        
        this.render();
        return thruster;
    }

    getWorldPositionFromMouse(event) {
        // Get mouse position in normalized device coordinates (-1 to +1)
        const rect = this.renderer.domElement.getBoundingClientRect();
        this.mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
        this.mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

        // Create a ray from the camera through the mouse position
        this.raycaster.setFromCamera(this.mouse, this.camera);

        // Find intersection with the grid plane
        const gridPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
        const point = new THREE.Vector3();
        this.raycaster.ray.intersectPlane(gridPlane, point);

        return point;
    }

    addSensor(id, type, position, orientation) {
        // Create a small box to represent the sensor
        const size = 0.1;
        const geometry = new THREE.BoxGeometry(size, size, size);
        
        // Color based on sensor type
        let color;
        switch (type.toLowerCase()) {
            case 'imu':
                color = 0x33cc33; // Green
                break;
            case 'gps':
                color = 0x3366cc; // Blue
                break;
            case 'dvl':
                color = 0xcc3333; // Red
                break;
            case 'camera':
                color = 0xffcc00; // Yellow
                break;
            case 'sonar':
                color = 0x9933cc; // Purple
                break;
            default:
                color = 0xaaaaaa; // Grey
        }
        
        const material = new THREE.MeshStandardMaterial({ 
            color, 
            transparent: true, 
            opacity: 0.7 
        });

        const sensor = new THREE.Mesh(geometry, material);
        sensor.name = `${type}_${id}`;
        sensor.userData.type = 'sensor';
        sensor.userData.componentId = id;
        
        // Position and orient the sensor
        sensor.position.set(position[0], position[1], position[2]);
        sensor.rotation.set(orientation[0], orientation[1], orientation[2]);
        
        // Add to scene
        this.scene.add(sensor);
        
        // Store reference
        if (!this.sensors) {
            this.sensors = new Map();
        }
        this.sensors.set(id, sensor);
        
        this.render();
        return sensor;
    }

    removeControlSurface(id) {
        const surface = this.controlSurfaces.get(id);
        if (surface) {
            this.vessel.remove(surface);
            this.controlSurfaces.delete(id);
            if (this.selectedObject === surface) {
                this.transformControls.detach();
                this.selectedObject = null;
            }
        }
    }

    removeThruster(id) {
        const thruster = this.thrusters.get(id);
        if (thruster) {
            this.vessel.remove(thruster);
            this.thrusters.delete(id);
            if (this.selectedObject === thruster) {
                this.transformControls.detach();
                this.selectedObject = null;
            }
        }
    }

    removeSensor(id) {
        const sensor = this.sensors.get(id);
        if (sensor) {
            this.vessel.remove(sensor);
            this.sensors.delete(id);
            if (this.selectedObject === sensor) {
                this.transformControls.detach();
                this.selectedObject = null;
            }
        }
    }

    setTransformMode(mode) {
        this.transformMode = mode;
        if (this.selectedObject) {
            this.transformControls.setMode(mode);
        }
        
        // Update button states
        document.getElementById('translateMode').classList.toggle('active', mode === 'translate');
        document.getElementById('rotateMode').classList.toggle('active', mode === 'rotate');
        document.getElementById('scaleMode').classList.toggle('active', mode === 'scale');
    }

    updateTransformInfo() {
        if (!this.selectedObject) return;

        const position = this.selectedObject.position || { x: 0, y: 0, z: 0 };
        const rotation = this.selectedObject.rotation || { x: 0, y: 0, z: 0 };
        const scale = this.selectedObject.scale || { x: 1, y: 1, z: 1 };

        // Update transform info panel
        const posX = document.getElementById('pos_x');
        const posY = document.getElementById('pos_y');
        const posZ = document.getElementById('pos_z');
        if (posX) posX.value = (position.x || 0).toFixed(3);
        if (posY) posY.value = (position.y || 0).toFixed(3);
        if (posZ) posZ.value = (position.z || 0).toFixed(3);

        const rotX = document.getElementById('rot_x');
        const rotY = document.getElementById('rot_y');
        const rotZ = document.getElementById('rot_z');
        if (rotX) rotX.value = (rotation.x || 0).toFixed(3);
        if (rotY) rotY.value = (rotation.y || 0).toFixed(3);
        if (rotZ) rotZ.value = (rotation.z || 0).toFixed(3);

        const scaleX = document.getElementById('scale_x');
        const scaleY = document.getElementById('scale_y');
        const scaleZ = document.getElementById('scale_z');
        if (scaleX) scaleX.value = (scale.x || 1).toFixed(3);
        if (scaleY) scaleY.value = (scale.y || 1).toFixed(3);
        if (scaleZ) scaleZ.value = (scale.z || 1).toFixed(3);

        // Update component card if it exists
        if (this.selectedObject.userData.card) {
            const card = this.selectedObject.userData.card;
            const posInputs = {
                x: card.querySelector('.pos-x'),
                y: card.querySelector('.pos-y'),
                z: card.querySelector('.pos-z')
            };

            const rotInputs = {
                x: card.querySelector('.rot-x'),
                y: card.querySelector('.rot-y'),
                z: card.querySelector('.rot-z')
            };

            // Update position inputs
            Object.entries(posInputs).forEach(([axis, input]) => {
                if (input) {
                    input.value = position[axis].toFixed(3);
                }
            });

            // Update rotation inputs
            Object.entries(rotInputs).forEach(([axis, input]) => {
                if (input) {
                    input.value = rotation[axis].toFixed(3);
                }
            });
        }

        // Trigger the onObjectTransformed callback
        if (this.onObjectTransformed) {
            this.onObjectTransformed(this.selectedObject);
        }
    }

    onObjectTransformed(object) {
        // This method will be overridden by the main application
        // to update the vessel model when objects are transformed
    }

    onWindowResize() {
        this.updateRendererSize();
        this.render();
    }

    onMouseMove(event) {
        // Update normalized mouse position
        this.mouse.x = (event.offsetX / this.renderer.domElement.clientWidth) * 2 - 1;
        this.mouse.y = -(event.offsetY / this.renderer.domElement.clientHeight) * 2 + 1;
    }

    onClick(event) {
        // Update raycaster with camera and mouse position
        this.raycaster.setFromCamera(this.mouse, this.camera);
        
        // Find intersected objects (include all scene objects for FBX component selection)
        const intersects = this.raycaster.intersectObjects(this.scene.children, true);
        
        if (intersects.length > 0) {
            // Find the first selectable object
            const selectedIntersect = intersects.find(intersect => 
                intersect.object.isMesh && 
                intersect.object.userData && 
                intersect.object.userData.selectable
            );
            
            if (selectedIntersect) {
                console.log('Selected object:', selectedIntersect.object.name);
                this.selectObject(selectedIntersect.object);
            } else {
                // No selectable object found, deselect current
                this.selectObject(null);
            }
        } else {
            // No intersection, deselect current
            this.selectObject(null);
        }
    }

    selectObject(object) {
        // Reset previous selection
        if (this.selectedObject && this.selectedObject !== object) {
            // Restore original material
            this.restoreOriginalMaterial(this.selectedObject);
        }
        
        // Update selection
        this.selectedObject = object;
        
        // Clear transform controls
        if (this.transformControls) {
            this.transformControls.detach();
        }
        
        // Setup new selection
        if (object && object.isMesh) {
            // Highlight the object
            this.highlightObject(object);
            
            // Attach transform controls
            if (this.transformControls) {
                this.transformControls.attach(object);
            }
            
            // Show component info in the UI
            this.showComponentInfo(object);
        } else {
            // If nothing is selected, clear the property panel
            document.getElementById('object-properties').innerHTML = `
                <div class="no-selection-message">
                    Select an object to view its properties
                </div>
            `;
        }
        
        // Mark scene for rendering
        this.render();
    }

    // Helper method to restore the original material
    restoreOriginalMaterial(object) {
        if (object && object.isMesh) {
            if (Array.isArray(object.material)) {
                // For multi-material objects, restore each material
                if (object.userData.originalMaterials) {
                    object.material = object.userData.originalMaterials.map(m => m.clone());
                }
            } else {
                // For single material objects
                if (object.userData.originalMaterial) {
                    object.material = object.userData.originalMaterial.clone();
                }
            }
        }
    }

    // Helper method to highlight an object
    highlightObject(object) {
        if (object && object.isMesh) {
            if (Array.isArray(object.material)) {
                // For multi-material objects, highlight each material
                object.material.forEach(material => {
                    material.emissive = new THREE.Color(0x333333);
                    material.emissiveIntensity = 0.5;
                });
            } else {
                // For single material objects
                object.material.emissive = new THREE.Color(0x333333);
                object.material.emissiveIntensity = 0.5;
            }
        }
    }

    // Display component info in the UI
    showComponentInfo(object) {
        // Get component data if it exists
        const vesselModel = window.currentVesselModel;
        const componentData = vesselModel?.getModelComponentData(object.uuid);
        
        // Create HTML for component info
        let html = `
            <div class="property-group">
                <div class="property-group-title">Component Properties</div>
                <div class="property-row">
                    <div class="property-label">Name</div>
                    <div class="property-value">${object.name || 'Unnamed'}</div>
                </div>
                <div class="property-row">
                    <div class="property-label">Type</div>
                    <div class="property-value">
                        <select id="componentTypeSelect" class="form-control">
                            <option value="none" ${!componentData ? 'selected' : ''}>Regular Component</option>
                            <option value="controlSurface" ${componentData?.type === 'controlSurface' ? 'selected' : ''}>Control Surface</option>
                            <option value="thruster" ${componentData?.type === 'thruster' ? 'selected' : ''}>Thruster</option>
                            <option value="sensor" ${componentData?.type === 'sensor' ? 'selected' : ''}>Sensor</option>
                        </select>
                    </div>
                </div>
            </div>
            
            <div class="property-group">
                <div class="property-group-title">Transform</div>
                <div class="property-row">
                    <div class="property-label">Position</div>
                    <div class="property-value">
                        <div class="input-group">
                            <input type="number" class="form-control" id="posX" value="${object.position.x.toFixed(3)}" step="0.1">
                            <input type="number" class="form-control" id="posY" value="${object.position.y.toFixed(3)}" step="0.1">
                            <input type="number" class="form-control" id="posZ" value="${object.position.z.toFixed(3)}" step="0.1">
                        </div>
                    </div>
                </div>
                <div class="property-row">
                    <div class="property-label">Rotation (deg)</div>
                    <div class="property-value">
                        <div class="input-group">
                            <input type="number" class="form-control" id="rotX" value="${THREE.MathUtils.radToDeg(object.rotation.x).toFixed(1)}" step="1">
                            <input type="number" class="form-control" id="rotY" value="${THREE.MathUtils.radToDeg(object.rotation.y).toFixed(1)}" step="1">
                            <input type="number" class="form-control" id="rotZ" value="${THREE.MathUtils.radToDeg(object.rotation.z).toFixed(1)}" step="1">
                        </div>
                    </div>
                </div>
            </div>
        `;
        
        // Add button to apply/configure component
        html += `
            <div class="mt-3">
                <button id="btnConfigureComponent" class="btn btn-primary">Configure Component</button>
            </div>
        `;
        
        // Update UI
        const propertiesPanel = document.getElementById('object-properties');
        if (propertiesPanel) {
            propertiesPanel.innerHTML = html;
            
            // Add event listener for type selection
            const typeSelect = document.getElementById('componentTypeSelect');
            if (typeSelect) {
                typeSelect.addEventListener('change', () => {
                    this.showComponentConfigModal(object, typeSelect.value);
                });
            }
            
            // Add event listener for configure button
            const configBtn = document.getElementById('btnConfigureComponent');
            if (configBtn) {
                configBtn.addEventListener('click', () => {
                    const typeValue = document.getElementById('componentTypeSelect').value;
                    this.showComponentConfigModal(object, typeValue);
                });
            }
            
            // Add event listeners for transform inputs
            ['posX', 'posY', 'posZ'].forEach(id => {
                document.getElementById(id)?.addEventListener('change', () => {
                    const x = parseFloat(document.getElementById('posX').value);
                    const y = parseFloat(document.getElementById('posY').value);
                    const z = parseFloat(document.getElementById('posZ').value);
                    object.position.set(x, y, z);
                    this.render();
                });
            });
            
            ['rotX', 'rotY', 'rotZ'].forEach(id => {
                document.getElementById(id)?.addEventListener('change', () => {
                    const x = THREE.MathUtils.degToRad(parseFloat(document.getElementById('rotX').value));
                    const y = THREE.MathUtils.degToRad(parseFloat(document.getElementById('rotY').value));
                    const z = THREE.MathUtils.degToRad(parseFloat(document.getElementById('rotZ').value));
                    object.rotation.set(x, y, z);
                    this.render();
                });
            });
        }
    }

    // Show modal to configure component based on selected type
    showComponentConfigModal(object, type) {
        // Get relevant modal based on component type
        let modalId;
        if (type === 'controlSurface') {
            modalId = 'fbxComponentModal';
            // Show control surface settings
            document.getElementById('fbxControlSurfaceSettings').style.display = 'block';
            document.getElementById('fbxThrusterSettings').style.display = 'none';
            document.getElementById('fbxSensorSettings').style.display = 'none';
        } else if (type === 'thruster') {
            modalId = 'fbxComponentModal';
            // Show thruster settings
            document.getElementById('fbxControlSurfaceSettings').style.display = 'none';
            document.getElementById('fbxThrusterSettings').style.display = 'block';
            document.getElementById('fbxSensorSettings').style.display = 'none';
        } else if (type === 'sensor') {
            modalId = 'fbxComponentModal';
            // Show sensor settings
            document.getElementById('fbxControlSurfaceSettings').style.display = 'none';
            document.getElementById('fbxThrusterSettings').style.display = 'none';
            document.getElementById('fbxSensorSettings').style.display = 'block';
        } else {
            // If none/regular, just remove any component mapping
            const vesselModel = window.currentVesselModel;
            if (vesselModel && vesselModel.isModelComponentMapped(object.uuid)) {
                vesselModel.unmapModelComponent(object.uuid);
                this.showComponentInfo(object); // Refresh UI
            }
            return;
        }
        
        // Set the component type in the modal
        document.getElementById('fbxComponentType').value = type;
        
        // Store the reference to the object in the modal for later use
        const modal = document.getElementById(modalId);
        if (modal) {
            modal.dataset.objectUuid = object.uuid;
            
            // Show the modal
            const bsModal = new bootstrap.Modal(modal);
            bsModal.show();
        }
    }

    toggleGrid() {
        if (this.grid) {
        this.grid.visible = !this.grid.visible;
            this.render();
            return this.grid.visible;
        }
        return false;
    }

    toggleWireframe() {
        let changed = false;
        this.scene.traverse((object) => {
            if (object.isMesh && object.material) {
                if (Array.isArray(object.material)) {
                    object.material.forEach(material => {
                        material.wireframe = !material.wireframe;
                    });
            } else {
                    object.material.wireframe = !object.material.wireframe;
                }
                changed = true;
            }
        });
        
        if (changed) {
            this.render();
        }
    }

    resetCamera() {
        // Reset camera position and target
        this.camera.position.set(5, 5, 5);
        this.camera.lookAt(0, 0, 0);
        this.orbitControls.target.set(0, 0, 0);
        this.orbitControls.update();
        this.render();
    }

    animate() {
        // Use requestAnimationFrame with the proper binding
        requestAnimationFrame(this.animate.bind(this));
        
        // Performance monitoring
        if (this.stats) this.stats.begin();
        
        // Only update controls if they exist and are enabled
        if (this.orbitControls && this.orbitControls.enabled) {
            this.orbitControls.update();
            // Orbit controls update should trigger a render
            this.needsRender = true;
        }
        
        // Only render when needed (needsRender flag is true)
        if (this.needsRender) {
            this.renderer.render(this.scene, this.camera);
            this.needsRender = false;
        }
        
        // Performance monitoring
        if (this.stats) this.stats.end();
    }

    updateRendererSize() {
        if (!this.container) return;
        
        const rect = this.container.getBoundingClientRect();
        
        // Only update if the size actually changed
        if (this.lastWidth !== rect.width || this.lastHeight !== rect.height) {
            this.lastWidth = rect.width;
            this.lastHeight = rect.height;
            
            this.camera.aspect = rect.width / rect.height;
            this.camera.updateProjectionMatrix();
            this.renderer.setSize(rect.width, rect.height);
            
            // Force a render to update the viewport
            this.forceRender();
        }
    }

    render() {
        this.needsRender = true;
    }

    forceRender() {
        this.renderer.render(this.scene, this.camera);
        this.needsRender = false;
    }

    getObjectById(id) {
        // Check control surfaces
        if (this.controlSurfaces.has(id)) {
            return this.controlSurfaces.get(id);
        }
        
        // Check thrusters
        if (this.thrusters.has(id)) {
            return this.thrusters.get(id);
        }
        
        // Check sensors
        if (this.sensors.has(id)) {
            return this.sensors.get(id);
        }
        
        return null;
    }

    updateControlSurfaceSize(id, area) {
        const surface = this.controlSurfaces.get(id);
        if (surface) {
            const scale = Math.sqrt(area);
            surface.scale.set(scale, scale, scale);
            this.render();
        }
    }

    updateThrusterSize(id, thrust) {
        const thruster = this.thrusters.get(id);
        if (thruster) {
            const scale = 0.2 + (thrust / 5000) * 0.8; // Scale between 0.2 and 1.0 based on thrust
            thruster.scale.set(scale, scale, scale);
            this.render();
        }
    }

    toggleBox(visible = null) {
        if (!this.vessel) return;
        
        const box = this.vessel.getObjectByName('boundingBox');
        if (box) {
            if (visible === null) {
                box.visible = !box.visible;
            } else {
                box.visible = visible;
            }
            
            // Update checkbox state
            const checkbox = document.getElementById('toggleBoxVisibility');
            if (checkbox) {
                checkbox.checked = box.visible;
            }
            
            this.render();
        }
    }

    toggleSTL(visible = null) {
        if (!this.vessel) return;
        
        const stlModel = this.vessel.getObjectByName('stlModel');
        if (stlModel) {
            if (visible === null) {
                stlModel.visible = !stlModel.visible;
            } else {
                stlModel.visible = visible;
            }
            
            // Update checkbox state
            const checkbox = document.getElementById('toggleSTLVisibility');
            if (checkbox) {
                checkbox.checked = stlModel.visible;
            }
            
            this.render();
        }
    }

    // Add new method to create body center coordinate system
    addBodyCenterSystem(position = new THREE.Vector3(0, 0, 0)) {
        // Remove existing BCS if any
        if (this.bodyCenter) {
            this.scene.remove(this.bodyCenter);
        }

        const bodyCenterGroup = new THREE.Group();
        bodyCenterGroup.userData.isBodyCenter = true;
        bodyCenterGroup.userData.isTransformable = true;
        bodyCenterGroup.userData.type = 'bodyCenter';
        bodyCenterGroup.name = 'Body Center System';

        // Get reference size from vessel if it exists
        let axesLength = 1;
        if (this.vessel && this.vessel.userData.axesGroup) {
            const bbox = new THREE.Box3().setFromObject(this.vessel);
            const size = new THREE.Vector3();
            bbox.getSize(size);
            axesLength = Math.max(size.x, size.y, size.z) * 0.3;
        }

        // Create body center axes
        const bcAxesLength = axesLength;
        
        // Create a container for axes and sphere to keep them together
        const bcContainer = new THREE.Group();
        bcContainer.name = 'BC Container';
        
        // X-axis (surge)
        const bcXAxis = new THREE.ArrowHelper(
            new THREE.Vector3(1, 0, 0),
            new THREE.Vector3(0, 0, 0),
            bcAxesLength,
            0xff0000,
            bcAxesLength * 0.25,
            bcAxesLength * 0.15
        );
        bcContainer.add(bcXAxis);

        // Y-axis (sway)
        const bcYAxis = new THREE.ArrowHelper(
            new THREE.Vector3(0, 1, 0),
            new THREE.Vector3(0, 0, 0),
            bcAxesLength,
            0x00ff00,
            bcAxesLength * 0.25,
            bcAxesLength * 0.15
        );
        bcContainer.add(bcYAxis);

        // Z-axis (heave)
        const bcZAxis = new THREE.ArrowHelper(
            new THREE.Vector3(0, 0, 1),
            new THREE.Vector3(0, 0, 0),
            bcAxesLength,
            0x0000ff,
            bcAxesLength * 0.25,
            bcAxesLength * 0.15
        );
        bcContainer.add(bcZAxis);

        // Add center sphere (made much smaller)
        const bcSphereGeometry = new THREE.SphereGeometry(bcAxesLength * 0.05);
        const bcSphereMaterial = new THREE.MeshStandardMaterial({
            color: 0xffffff,
            metalness: 0.7,
            roughness: 0.3,
            emissive: 0x444444
        });
        const bcSphere = new THREE.Mesh(bcSphereGeometry, bcSphereMaterial);
        bcSphere.name = 'BC Sphere';
        bcContainer.add(bcSphere);

        // Add container to body center group
        bodyCenterGroup.add(bcContainer);
        
        // Set position
        bodyCenterGroup.position.copy(position);

        // Add to scene and store reference
        this.scene.add(bodyCenterGroup);
        this.bodyCenter = bodyCenterGroup;

        this.updateSceneHierarchy();

        return bodyCenterGroup;
    }

    removeBodyCenterSystem() {
        if (this.bodyCenterSystem) {
            this.scene.remove(this.bodyCenterSystem);
            this.bodyCenterSystem = null;
            this.render();
        }
    }

    // Add new method to update scene hierarchy
    updateSceneHierarchy() {
        // Placeholder for scene hierarchy update
        console.log("Scene hierarchy updated");
    }

    async loadFBXModel(file) {
        return new Promise((resolve, reject) => {
            const reader = new FileReader();
            reader.onload = (event) => {
                const fileContents = event.target.result;
                
                const loader = new THREE.FBXLoader();
                try {
                    const object = loader.parse(fileContents);
                    
                    // Clean up any existing model
                    if (this.vessel) {
                        this.scene.remove(this.vessel);
                        this.vessel = null;
                        this.updateSceneHierarchy();
                    }
                    
                    // Set up the vessel container
                    this.vessel = new THREE.Group();
                    this.vessel.name = 'Vessel';
                    
                    // Create a mapping of original meshes
                    this.modelMeshes = new Map();
                    
                    // Process the loaded model to make every mesh selectable
                    let index = 0;
                    object.traverse(child => {
                        if (child.isMesh) {
                            // Store original materials for later reference
                            if (Array.isArray(child.material)) {
                                child.userData.originalMaterials = child.material.map(m => m.clone());
                            } else {
                                child.userData.originalMaterial = child.material.clone();
                            }
                            
                            // Make sure the mesh has a name for identification
                            if (!child.name || child.name === '') {
                                child.name = `Component_${index++}`;
                            }
                            
                            // Store in our mesh map
                            this.modelMeshes.set(child.uuid, child);
                            
                            // Make mesh selectable
                            child.userData.selectable = true;
                            
                            // Log the component for debugging
                            console.log(`FBX component found: ${child.name} (${child.uuid})`);
                        }
                    });
                    
                    // Add the model to the vessel container
                    this.vessel.add(object);
                    
                    // Calculate the bounding box
                    const boundingBox = new THREE.Box3().setFromObject(this.vessel);
                    const size = new THREE.Vector3();
                    boundingBox.getSize(size);
                    
                    // Determine scale to normalize the model
                    const maxDimension = Math.max(size.x, size.y, size.z);
                    const scale = 5 / maxDimension;
                    
                    // Center the model
                    const center = new THREE.Vector3();
                    boundingBox.getCenter(center);
                    this.vessel.position.sub(center.multiplyScalar(scale));
                    
                    // Scale the model
                    this.vessel.scale.set(scale, scale, scale);
                    
                    // Add vessel to scene
                    this.scene.add(this.vessel);
                    
                    // Add axes to help visualize orientation
                    const axes = this.createLocalAxes(2);
                    this.vessel.add(axes);
                    
                    // Update scene hierarchy
                    this.updateSceneHierarchy();
                    
                    // Update camera to focus on the model
                    this.resetCamera();
                    
                    // Force a render to show the model immediately
                    this.forceRender();
                    
                    // Return the loaded model
                    resolve(this.vessel);
                } catch (error) {
                    console.error('Error loading FBX model:', error);
                    reject(error);
                }
            };
            reader.onerror = reject;
            reader.readAsArrayBuffer(file);
        });
    }

    selectModelComponent(object) {
        // Deselect previous object if any
        if (this.selectedObject) {
            // Restore original material
            if (this.selectedObject.isMesh) {
                this.selectedObject.material = this.selectedObject.userData.originalMaterial.clone();
            }
            
            // Remove transform controls
            this.transformControls.detach();
        }
        
        // Update selected object
        this.selectedObject = object;
        
        if (object) {
            // Highlight selected object
            if (object.isMesh) {
                // Clone the original material and modify it to show selection
                const highlightMaterial = object.userData.originalMaterial.clone();
                highlightMaterial.emissive = new THREE.Color(0x333333);
                highlightMaterial.emissiveIntensity = 0.5;
                object.material = highlightMaterial;
            }
            
            // Attach transform controls
            this.transformControls.attach(object);
            
            // Update UI with selection info
            this.showComponentSettings(object);
            } else {
            // Clear UI if no selection
            document.getElementById('object-properties').innerHTML = `
                <div class="no-selection-message">
                    Select an object to view its properties
                </div>
            `;
        }
        
        // Update scene
        this.render();
    }

    showComponentSettings(object) {
        // First check if this object is already mapped to a component
        const vesselModel = window.currentVesselModel; // This should be set in the main.js
        
        if (!vesselModel) return;
        
        const componentData = vesselModel.getModelComponentData(object.uuid);
        let componentType = componentData ? componentData.type : 'none';
        
        // Create HTML for properties panel
        let html = `
            <div class="property-group">
                <div class="property-group-title">Object Properties</div>
                <div class="property-row">
                    <div class="property-label">Name</div>
                    <div class="property-value">${object.name || 'Unnamed Object'}</div>
                </div>
                <div class="property-row">
                    <div class="property-label">Type</div>
                    <div class="property-value">
                        <select id="componentTypeSelect" class="form-control">
                            <option value="none" ${componentType === 'none' ? 'selected' : ''}>Regular Object</option>
                            <option value="controlSurface" ${componentType === 'controlSurface' ? 'selected' : ''}>Control Surface</option>
                            <option value="thruster" ${componentType === 'thruster' ? 'selected' : ''}>Thruster</option>
                            <option value="sensor" ${componentType === 'sensor' ? 'selected' : ''}>Sensor</option>
                        </select>
                    </div>
                </div>
            </div>
            
            <div class="property-group">
                <div class="property-group-title">Transform</div>
                <div class="property-row">
                    <div class="property-label">Position</div>
                    <div class="property-value">
                        <div class="input-group">
                            <input type="number" class="form-control" id="posX" value="${object.position.x.toFixed(2)}" step="0.1">
                            <input type="number" class="form-control" id="posY" value="${object.position.y.toFixed(2)}" step="0.1">
                            <input type="number" class="form-control" id="posZ" value="${object.position.z.toFixed(2)}" step="0.1">
                        </div>
                    </div>
                </div>
                <div class="property-row">
                    <div class="property-label">Rotation (deg)</div>
                    <div class="property-value">
                        <div class="input-group">
                            <input type="number" class="form-control" id="rotX" value="${THREE.MathUtils.radToDeg(object.rotation.x).toFixed(1)}" step="1">
                            <input type="number" class="form-control" id="rotY" value="${THREE.MathUtils.radToDeg(object.rotation.y).toFixed(1)}" step="1">
                            <input type="number" class="form-control" id="rotZ" value="${THREE.MathUtils.radToDeg(object.rotation.z).toFixed(1)}" step="1">
                        </div>
                    </div>
                </div>
                <div class="property-row">
                    <div class="property-label">Scale</div>
                    <div class="property-value">
                        <div class="input-group">
                            <input type="number" class="form-control" id="scaleX" value="${object.scale.x.toFixed(2)}" step="0.1" min="0.1">
                            <input type="number" class="form-control" id="scaleY" value="${object.scale.y.toFixed(2)}" step="0.1" min="0.1">
                            <input type="number" class="form-control" id="scaleZ" value="${object.scale.z.toFixed(2)}" step="0.1" min="0.1">
                        </div>
                    </div>
                </div>
            </div>
        `;
        
        // Add specific component settings if this is a component
        if (componentType !== 'none' && componentData) {
            // Add component-specific properties based on the component type
            html += this.createComponentSpecificProperties(componentType, componentData.id, vesselModel);
        }
        
        // Add button to apply changes
        html += `
            <div class="mt-3">
                <button id="btnApplyComponentChanges" class="btn btn-primary">Apply Changes</button>
                <button id="btnRemoveComponent" class="btn btn-danger" ${componentType === 'none' ? 'style="display:none"' : ''}>Remove Component</button>
            </div>
        `;
        
        // Update the properties panel
        document.getElementById('object-properties').innerHTML = html;
        
        // Set up event listeners for the form fields
        this.setupComponentPropertyListeners(object);
    }

    createComponentSpecificProperties(componentType, componentId, vesselModel) {
        let html = '';
        
        switch (componentType) {
            case 'controlSurface':
                const surface = vesselModel.getControlSurface(componentId);
                if (surface) {
                    html += `
                        <div class="property-group">
                            <div class="property-group-title">Control Surface Properties</div>
                            <div class="property-row">
                                <div class="property-label">Type</div>
                                <div class="property-value">
                                    <select id="csSurfaceType" class="form-control">
                                        <option value="Rudder" ${surface.control_surface_type === 'Rudder' ? 'selected' : ''}>Rudder</option>
                                        <option value="Fins" ${surface.control_surface_type === 'Fins' ? 'selected' : ''}>Fins</option>
                                        <option value="Elevator" ${surface.control_surface_type === 'Elevator' ? 'selected' : ''}>Elevator</option>
                                    </select>
                                </div>
                            </div>
                            <div class="property-row">
                                <div class="property-label">Area (m²)</div>
                                <div class="property-value">
                                    <input type="number" class="form-control" id="csSurfaceArea" value="${surface.control_surface_area}" step="0.01" min="0.01">
                                </div>
                            </div>
                            <div class="property-row">
                                <div class="property-label">Time Constant</div>
                                <div class="property-value">
                                    <input type="number" class="form-control" id="csSurfaceTimeConstant" value="${surface.control_surface_T}" step="0.01" min="0.01">
                                </div>
                            </div>
                        </div>
                    `;
                }
                break;
                
            case 'thruster':
                const thruster = vesselModel.getThruster(componentId);
                if (thruster) {
                    html += `
                        <div class="property-group">
                            <div class="property-group-title">Thruster Properties</div>
                            <div class="property-row">
                                <div class="property-label">Name</div>
                                <div class="property-value">
                                    <input type="text" class="form-control" id="thrusterName" value="${thruster.thruster_name}">
                                </div>
                            </div>
                            <div class="property-row">
                                <div class="property-label">Type</div>
                                <div class="property-value">
                                    <select id="thrusterType" class="form-control">
                                        <option value="Propeller" ${thruster.thruster_type === 'Propeller' ? 'selected' : ''}>Propeller</option>
                                        <option value="Jet" ${thruster.thruster_type === 'Jet' ? 'selected' : ''}>Jet</option>
                                        <option value="Tunnel" ${thruster.thruster_type === 'Tunnel' ? 'selected' : ''}>Tunnel</option>
                                    </select>
                                </div>
                            </div>
                            <div class="property-row">
                                <div class="property-label">Diameter (m)</div>
                                <div class="property-value">
                                    <input type="number" class="form-control" id="thrusterDiameter" value="${thruster.D_prop}" step="0.01" min="0.01">
                                </div>
                            </div>
                            <div class="property-row">
                                <div class="property-label">Thrust Coefficient</div>
                                <div class="property-value">
                                    <input type="number" class="form-control" id="thrusterCoefficient" value="${thruster.T_prop}" step="0.1" min="0.1">
                                </div>
                            </div>
                        </div>
                    `;
                }
                break;
                
            case 'sensor':
                const sensor = vesselModel.getSensor(componentId);
                if (sensor) {
                    html += `
                        <div class="property-group">
                            <div class="property-group-title">Sensor Properties</div>
                            <div class="property-row">
                                <div class="property-label">Type</div>
                                <div class="property-value">
                                    <select id="sensorType" class="form-control">
                                        <option value="IMU" ${sensor.sensor_type === 'IMU' ? 'selected' : ''}>IMU</option>
                                        <option value="GPS" ${sensor.sensor_type === 'GPS' ? 'selected' : ''}>GPS</option>
                                        <option value="DVL" ${sensor.sensor_type === 'DVL' ? 'selected' : ''}>DVL</option>
                                        <option value="Depth" ${sensor.sensor_type === 'Depth' ? 'selected' : ''}>Depth</option>
                                        <option value="Camera" ${sensor.sensor_type === 'Camera' ? 'selected' : ''}>Camera</option>
                                        <option value="Sonar" ${sensor.sensor_type === 'Sonar' ? 'selected' : ''}>Sonar</option>
                                    </select>
                                </div>
                            </div>
                            <div class="property-row">
                                <div class="property-label">Publish Rate (Hz)</div>
                                <div class="property-value">
                                    <input type="number" class="form-control" id="sensorPublishRate" value="${sensor.publish_rate}" step="0.1" min="0.1">
                                </div>
                            </div>
                        </div>
                    `;
                }
                break;
        }
        
        return html;
    }

    setupComponentPropertyListeners(object) {
        const vesselModel = window.currentVesselModel;
        if (!vesselModel) return;
        
        // Component type selection change
        const componentTypeSelect = document.getElementById('componentTypeSelect');
        if (componentTypeSelect) {
            componentTypeSelect.addEventListener('change', (e) => {
                const newType = e.target.value;
                const oldType = vesselModel.getModelComponentData(object.uuid)?.type;
                
                // If changing from one type to another, remove the old mapping
                if (oldType && oldType !== 'none') {
                    const componentData = vesselModel.getModelComponentData(object.uuid);
                    if (componentData) {
                        const componentId = componentData.id;
                        
                        // Remove the old component
                        if (oldType === 'controlSurface') {
                            vesselModel.removeControlSurface(componentId);
                        } else if (oldType === 'thruster') {
                            vesselModel.removeThruster(componentId);
                        } else if (oldType === 'sensor') {
                            vesselModel.removeSensor(componentId);
                        }
                        
                        // Remove the mapping
                        vesselModel.unmapModelComponent(object.uuid);
                    }
                }
                
                // If changing to a component type, create a new component
                if (newType !== 'none') {
                    // Get the object's world position and orientation
                    const worldPosition = new THREE.Vector3();
                    object.getWorldPosition(worldPosition);
                    
                    // Convert to world rotation (Euler angles)
                    const worldQuaternion = new THREE.Quaternion();
                    object.getWorldQuaternion(worldQuaternion);
                    const worldEuler = new THREE.Euler().setFromQuaternion(worldQuaternion);
                    
                    // Create the component based on type
                    let componentId;
                    if (newType === 'controlSurface') {
                        componentId = vesselModel.addControlSurface(
                            'Rudder',
                            [worldPosition.x, worldPosition.y, worldPosition.z],
                            [worldEuler.x, worldEuler.y, worldEuler.z],
                            0.1
                        );
                    } else if (newType === 'thruster') {
                        componentId = vesselModel.addThruster(
                            'Propeller',
                            [worldPosition.x, worldPosition.y, worldPosition.z],
                            [worldEuler.x, worldEuler.y, worldEuler.z],
                            0.2
                        );
                    } else if (newType === 'sensor') {
                        componentId = vesselModel.addSensor(
                            'IMU',
                            [worldPosition.x, worldPosition.y, worldPosition.z],
                            [worldEuler.x, worldEuler.y, worldEuler.z],
                            10
                        );
                    }
                    
                    // Map the object to the component
                    if (componentId) {
                        vesselModel.mapModelComponent(object.uuid, newType, componentId);
                    }
                }
                
                // Refresh the component settings display
                this.showComponentSettings(object);
                
                // Make the remove button visible if needed
                const removeBtn = document.getElementById('btnRemoveComponent');
                if (removeBtn) {
                    removeBtn.style.display = newType !== 'none' ? 'inline-block' : 'none';
                }
            });
        }
        
        // Transform properties change
        ['posX', 'posY', 'posZ'].forEach(id => {
            const input = document.getElementById(id);
            if (input) {
                input.addEventListener('change', () => {
                    const x = parseFloat(document.getElementById('posX').value);
                    const y = parseFloat(document.getElementById('posY').value);
                    const z = parseFloat(document.getElementById('posZ').value);
                    
                    object.position.set(x, y, z);
                    this.updateComponentPositionFromObject(object);
                    this.render();
                });
            }
        });
        
        ['rotX', 'rotY', 'rotZ'].forEach(id => {
            const input = document.getElementById(id);
            if (input) {
                input.addEventListener('change', () => {
                    const x = THREE.MathUtils.degToRad(parseFloat(document.getElementById('rotX').value));
                    const y = THREE.MathUtils.degToRad(parseFloat(document.getElementById('rotY').value));
                    const z = THREE.MathUtils.degToRad(parseFloat(document.getElementById('rotZ').value));
                    
                    object.rotation.set(x, y, z);
                    this.updateComponentOrientationFromObject(object);
                    this.render();
                });
            }
        });
        
        ['scaleX', 'scaleY', 'scaleZ'].forEach(id => {
            const input = document.getElementById(id);
            if (input) {
                input.addEventListener('change', () => {
                    const x = parseFloat(document.getElementById('scaleX').value);
                    const y = parseFloat(document.getElementById('scaleY').value);
                    const z = parseFloat(document.getElementById('scaleZ').value);
                    
                    object.scale.set(x, y, z);
                    this.render();
                });
            }
        });
        
        // Component-specific properties change
        const componentData = vesselModel.getModelComponentData(object.uuid);
        if (componentData) {
            const componentType = componentData.type;
            const componentId = componentData.id;
            
            // Control Surface properties
            if (componentType === 'controlSurface') {
                const surfaceTypeSelect = document.getElementById('csSurfaceType');
                const surfaceAreaInput = document.getElementById('csSurfaceArea');
                const timeConstantInput = document.getElementById('csSurfaceTimeConstant');
                
                if (surfaceTypeSelect) {
                    surfaceTypeSelect.addEventListener('change', () => {
                        vesselModel.updateControlSurface(componentId, {
                            control_surface_type: surfaceTypeSelect.value
                        });
                    });
                }
                
                if (surfaceAreaInput) {
                    surfaceAreaInput.addEventListener('change', () => {
                        vesselModel.updateControlSurface(componentId, {
                            control_surface_area: parseFloat(surfaceAreaInput.value)
                        });
                    });
                }
                
                if (timeConstantInput) {
                    timeConstantInput.addEventListener('change', () => {
                        vesselModel.updateControlSurface(componentId, {
                            control_surface_T: parseFloat(timeConstantInput.value)
                        });
                    });
                }
            }
            
            // Thruster properties
            else if (componentType === 'thruster') {
                const thrusterNameInput = document.getElementById('thrusterName');
                const thrusterTypeSelect = document.getElementById('thrusterType');
                const thrusterDiameterInput = document.getElementById('thrusterDiameter');
                const thrusterCoefficientInput = document.getElementById('thrusterCoefficient');
                
                if (thrusterNameInput) {
                    thrusterNameInput.addEventListener('change', () => {
                        vesselModel.updateThruster(componentId, {
                            thruster_name: thrusterNameInput.value
                        });
                    });
                }
                
                if (thrusterTypeSelect) {
                    thrusterTypeSelect.addEventListener('change', () => {
                        vesselModel.updateThruster(componentId, {
                            thruster_type: thrusterTypeSelect.value
                        });
                    });
                }
                
                if (thrusterDiameterInput) {
                    thrusterDiameterInput.addEventListener('change', () => {
                        vesselModel.updateThruster(componentId, {
                            D_prop: parseFloat(thrusterDiameterInput.value)
                        });
                    });
                }
                
                if (thrusterCoefficientInput) {
                    thrusterCoefficientInput.addEventListener('change', () => {
                        vesselModel.updateThruster(componentId, {
                            T_prop: parseFloat(thrusterCoefficientInput.value)
                        });
                    });
                }
            }
            
            // Sensor properties
            else if (componentType === 'sensor') {
                const sensorTypeSelect = document.getElementById('sensorType');
                const sensorPublishRateInput = document.getElementById('sensorPublishRate');
                
                if (sensorTypeSelect) {
                    sensorTypeSelect.addEventListener('change', () => {
                        vesselModel.updateSensor(componentId, {
                            sensor_type: sensorTypeSelect.value
                        });
                    });
                }
                
                if (sensorPublishRateInput) {
                    sensorPublishRateInput.addEventListener('change', () => {
                        vesselModel.updateSensor(componentId, {
                            publish_rate: parseFloat(sensorPublishRateInput.value)
                        });
                    });
                }
            }
        }
        
        // Apply changes button
        const applyButton = document.getElementById('btnApplyComponentChanges');
        if (applyButton) {
            applyButton.addEventListener('click', () => {
                // Update transform
                const x = parseFloat(document.getElementById('posX').value);
                const y = parseFloat(document.getElementById('posY').value);
                const z = parseFloat(document.getElementById('posZ').value);
                object.position.set(x, y, z);
                
                const rotX = THREE.MathUtils.degToRad(parseFloat(document.getElementById('rotX').value));
                const rotY = THREE.MathUtils.degToRad(parseFloat(document.getElementById('rotY').value));
                const rotZ = THREE.MathUtils.degToRad(parseFloat(document.getElementById('rotZ').value));
                object.rotation.set(rotX, rotY, rotZ);
                
                const scaleX = parseFloat(document.getElementById('scaleX').value);
                const scaleY = parseFloat(document.getElementById('scaleY').value);
                const scaleZ = parseFloat(document.getElementById('scaleZ').value);
                object.scale.set(scaleX, scaleY, scaleZ);
                
                // Update component position & orientation if applicable
                this.updateComponentPositionFromObject(object);
                this.updateComponentOrientationFromObject(object);
                
                // Refresh view
                this.render();
            });
        }
        
        // Remove component button
        const removeButton = document.getElementById('btnRemoveComponent');
        if (removeButton) {
            removeButton.addEventListener('click', () => {
                const componentData = vesselModel.getModelComponentData(object.uuid);
                if (componentData) {
                    const componentType = componentData.type;
                    const componentId = componentData.id;
                    
                    // Remove the component
                    if (componentType === 'controlSurface') {
                        vesselModel.removeControlSurface(componentId);
                    } else if (componentType === 'thruster') {
                        vesselModel.removeThruster(componentId);
                    } else if (componentType === 'sensor') {
                        vesselModel.removeSensor(componentId);
                    }
                    
                    // Remove the mapping
                    vesselModel.unmapModelComponent(object.uuid);
                    
                    // Update UI
                    componentTypeSelect.value = 'none';
                    removeButton.style.display = 'none';
                    
                    // Refresh component settings
                    this.showComponentSettings(object);
                }
            });
        }
    }

    updateComponentPositionFromObject(object) {
        const vesselModel = window.currentVesselModel;
        if (!vesselModel) return;
        
        const componentData = vesselModel.getModelComponentData(object.uuid);
        if (!componentData) return;
        
        // Get world position
        const worldPosition = new THREE.Vector3();
        object.getWorldPosition(worldPosition);
        
        // Update component position based on type
        const componentType = componentData.type;
        const componentId = componentData.id;
        
        if (componentType === 'controlSurface') {
            vesselModel.updateControlSurface(componentId, {
                control_surface_location: [worldPosition.x, worldPosition.y, worldPosition.z]
            });
        } else if (componentType === 'thruster') {
            vesselModel.updateThruster(componentId, {
                thruster_location: [worldPosition.x, worldPosition.y, worldPosition.z]
            });
        } else if (componentType === 'sensor') {
            vesselModel.updateSensor(componentId, {
                sensor_location: [worldPosition.x, worldPosition.y, worldPosition.z]
            });
        }
    }

    updateComponentOrientationFromObject(object) {
        const vesselModel = window.currentVesselModel;
        if (!vesselModel) return;
        
        const componentData = vesselModel.getModelComponentData(object.uuid);
        if (!componentData) return;
        
        // Get world orientation
        const worldQuaternion = new THREE.Quaternion();
        object.getWorldQuaternion(worldQuaternion);
        const worldEuler = new THREE.Euler().setFromQuaternion(worldQuaternion);
        
        // Update component orientation based on type
        const componentType = componentData.type;
        const componentId = componentData.id;
        
        if (componentType === 'controlSurface') {
            vesselModel.updateControlSurface(componentId, {
                control_surface_orientation: [worldEuler.x, worldEuler.y, worldEuler.z]
            });
        } else if (componentType === 'thruster') {
            vesselModel.updateThruster(componentId, {
                thruster_orientation: [worldEuler.x, worldEuler.y, worldEuler.z]
            });
        } else if (componentType === 'sensor') {
            vesselModel.updateSensor(componentId, {
                sensor_orientation: [worldEuler.x, worldEuler.y, worldEuler.z]
            });
        }
    }

    // Selection methods
    setTransformMode(mode) {
        if (this.transformControls && mode) {
            this.transformControls.setMode(mode);
        }
    }

    // Add performance monitoring if needed
    addPerformanceMonitor() {
        // Import Stats.js dynamically
        const script = document.createElement('script');
        script.onload = () => {
            this.stats = new Stats();
            this.stats.showPanel(0); // 0: fps, 1: ms, 2: mb, 3+: custom
            document.body.appendChild(this.stats.dom);
            console.log('Performance monitor enabled');
        };
        script.src = 'https://cdnjs.cloudflare.com/ajax/libs/stats.js/r17/Stats.min.js';
        document.head.appendChild(script);
    }

    // Implement batched updates for transform inputs
    updateTransformFromInputs(object, position, rotation) {
        if (!object) return;
        
        // Update in a single batch to minimize renders
        object.position.set(position.x, position.y, position.z);
        object.rotation.set(
            THREE.MathUtils.degToRad(rotation.x),
            THREE.MathUtils.degToRad(rotation.y),
            THREE.MathUtils.degToRad(rotation.z)
        );
        
        // Trigger a single render
        this.render();
    }

    // Initialize event listeners
    initializeEventListeners() {
        // Window resize event - debounced for performance
        let resizeTimeout;
        window.addEventListener('resize', () => {
            if (resizeTimeout) clearTimeout(resizeTimeout);
            resizeTimeout = setTimeout(() => {
                this.updateRendererSize();
            }, 100);
        });
        
        // Mouse events for object selection
        this.renderer.domElement.addEventListener('mousemove', (event) => {
            // Calculate mouse position in normalized device coordinates
            const rect = this.renderer.domElement.getBoundingClientRect();
            this.mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
            this.mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
        });
        
        this.renderer.domElement.addEventListener('click', this.onClick.bind(this));
        
        // Transform controls mode switching
        window.addEventListener('keydown', (event) => {
            // Only process if the 3D view is active (not typing in an input field)
            if (event.target.tagName === 'INPUT' || event.target.tagName === 'TEXTAREA') {
                return;
            }
            
            switch (event.key) {
                case 'g': // Translate
                    this.setTransformMode('translate');
                    break;
                case 'r': // Rotate
                    this.setTransformMode('rotate');
                    break;
                case 's': // Scale
                    this.setTransformMode('scale');
                    break;
                case 'Escape': // Deselect
                    this.selectObject(null);
                    break;
            }
        });
        
        // Optimize rendering during scroll events
        this.renderer.domElement.addEventListener('wheel', () => {
            this.needsRender = true;
        });
        
        // Handle low-level GL context loss
        this.renderer.domElement.addEventListener('webglcontextlost', (event) => {
            event.preventDefault();
            console.warn('WebGL context lost. Trying to restore...');
        });
        
        this.renderer.domElement.addEventListener('webglcontextrestored', () => {
            console.log('WebGL context restored');
            this.forceRender();
        });
    }

    // Set the enabled state of the controls
    setControlsEnabled(enabled) {
        if (this.orbitControls) {
            this.orbitControls.enabled = enabled;
        }
    }

    // Update the target position of orbit controls
    updateOrbitTarget(position) {
        if (this.orbitControls) {
            this.orbitControls.target.copy(position);
            this.orbitControls.update();
            this.render();
        }
    }
}

// Export the class
window.ThreeScene = ThreeScene; 