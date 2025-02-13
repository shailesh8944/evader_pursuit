class ThreeScene {
    constructor() {
        this.scene = new THREE.Scene();
        const aspect = window.innerWidth / window.innerHeight;
        this.camera = new THREE.PerspectiveCamera(75, aspect, 0.1, 1000);
        this.camera.position.set(5, 5, 5);
        this.camera.lookAt(0, 0, 0);
        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.renderer.setPixelRatio(window.devicePixelRatio);
        this.updateRendererSize();
        this.controls = null;
        this.grid = null;
        this.vessel = null;
        this.controlSurfaces = new Map();
        this.thrusters = new Map();
        this.sensors = new Map();
        this.transformControls = null;
        this.raycaster = new THREE.Raycaster();
        this.mouse = new THREE.Vector2();
        this.selectedObject = null;
        this.transformMode = 'translate'; // 'translate', 'rotate', 'scale'
        
        this.updateSceneHierarchy = this.updateSceneHierarchy.bind(this);
        
        this.init();
    }

    init() {
        // Setup renderer
        this.renderer.setSize(window.innerWidth * 0.75, window.innerHeight);
        this.renderer.setClearColor(0x1a1a1a);
        this.renderer.shadowMap.enabled = true;
        document.getElementById('viewer3D').appendChild(this.renderer.domElement);

        // Setup camera
        this.camera.position.set(5, 5, 5);
        this.camera.lookAt(0, 0, 0);

        // Setup orbit controls
        this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;

        // Setup transform controls
        this.transformControls = new THREE.TransformControls(this.camera, this.renderer.domElement);
        this.scene.add(this.transformControls);

        // Disable orbit controls when using transform controls
        this.transformControls.addEventListener('dragging-changed', (event) => {
            this.controls.enabled = !event.value;
            if (event.value) {
                // Disable other controls while dragging
                this.renderer.domElement.style.cursor = 'move';
            } else {
                this.renderer.domElement.style.cursor = 'auto';
                // Update transform info after dragging
                if (this.selectedObject) {
                    this.updateTransformInfo();
                    if (this.onObjectTransformed) {
                        this.onObjectTransformed(this.selectedObject);
                    }
                }
            }
        });

        // Listen for changes during transform
        this.transformControls.addEventListener('change', () => {
            if (this.selectedObject) {
                requestAnimationFrame(() => {
                    this.updateTransformInfo();
                    this.render();
                });
            }
        });

        // Listen for object changes
        this.transformControls.addEventListener('objectChange', () => {
            if (this.selectedObject) {
                requestAnimationFrame(() => {
                    this.updateTransformInfo();
                    if (this.onObjectTransformed) {
                        this.onObjectTransformed(this.selectedObject);
                    }
                    this.render();
                });
            }
        });

        // Add key listeners for transform modes
        window.addEventListener('keydown', (event) => {
            if (this.selectedObject) {
                switch (event.key.toLowerCase()) {
                    case 'g':
                        this.setTransformMode('translate');
                        break;
                    case 'r':
                        this.setTransformMode('rotate');
                        break;
                    case 's':
                        this.setTransformMode('scale');
                        break;
                    case 'escape':
                        this.transformControls.detach();
                        this.selectedObject = null;
                        this.render();
                        break;
                }
            }
        });

        // Add grid
        this.addGrid();

        // Add lights
        this.addLights();

        // Add event listeners
        window.addEventListener('resize', this.onWindowResize.bind(this));
        this.renderer.domElement.addEventListener('mousemove', (event) => this.onMouseMove(event), false);
        this.renderer.domElement.addEventListener('click', (event) => this.onClick(event), false);

        // Setup transform mode buttons
        document.getElementById('translateMode').addEventListener('click', () => this.setTransformMode('translate'));
        document.getElementById('rotateMode').addEventListener('click', () => this.setTransformMode('rotate'));
        document.getElementById('scaleMode').addEventListener('click', () => this.setTransformMode('scale'));

        // Setup hierarchy panel collapse
        const hierarchyPanel = document.querySelector('.scene-hierarchy');
        const collapseBtn = document.getElementById('collapseHierarchy');
        collapseBtn.addEventListener('click', () => {
            hierarchyPanel.classList.toggle('collapsed');
            const icon = collapseBtn.querySelector('i');
            if (hierarchyPanel.classList.contains('collapsed')) {
                icon.className = 'bi bi-chevron-down';
            } else {
                icon.className = 'bi bi-chevron-up';
            }
        });

        // Setup visibility toggles
        const stlToggle = document.getElementById('toggleSTLVisibility');
        const boxToggle = document.getElementById('toggleBoxVisibility');

        if (stlToggle) {
            stlToggle.addEventListener('change', () => {
                if (this.vessel?.userData.isSTLModel && this.vessel.userData.stlMesh) {
                    this.vessel.userData.stlMesh.visible = stlToggle.checked;
                    this.render();
                }
            });
        }

        if (boxToggle) {
            boxToggle.addEventListener('change', () => {
                if (this.vessel?.userData.isSTLModel && this.vessel.userData.boxMesh) {
                    this.vessel.userData.boxMesh.visible = boxToggle.checked;
                    this.render();
                }
            });
        }

        // Start animation loop
        this.animate();
    }

    addGrid() {
        this.grid = new THREE.GridHelper(20, 20);
        this.scene.add(this.grid);

        // Add axes helper
        const axesHelper = new THREE.AxesHelper(5);
        this.scene.add(axesHelper);
    }

    addLights() {
        // Ambient light
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
        this.scene.add(ambientLight);

        // Directional lights from different angles
        const createDirLight = (x, y, z) => {
            const light = new THREE.DirectionalLight(0xffffff, 0.8);
            light.position.set(x, y, z);
            light.castShadow = true;
            return light;
        };

        this.scene.add(createDirLight(5, 5, 5));
        this.scene.add(createDirLight(-5, -5, -5));
        this.scene.add(createDirLight(5, -5, 5));

        // Hemisphere light for better ambient illumination
        const hemisphereLight = new THREE.HemisphereLight(0xffffbb, 0x080820, 0.5);
        this.scene.add(hemisphereLight);
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
        // Don't create default vessel if we have an STL model loaded
        if (this.vessel && this.vessel.userData.isSTLModel) {
            return;
        }

        // Remove existing vessel if any
        if (this.vessel) {
            this.scene.remove(this.vessel);
        }

        // Create vessel geometry
        const geometry = new THREE.BoxGeometry(length || 1, depth || 1, breadth || 1);
        const material = new THREE.MeshStandardMaterial({
            color: 0x3498db,
            metalness: 0.3,
            roughness: 0.4,
            transparent: true,
            opacity: 0.8
        });

        this.vessel = new THREE.Mesh(geometry, material);
        this.vessel.castShadow = true;
        this.vessel.receiveShadow = true;
        this.vessel.userData.type = 'vessel';
        this.vessel.userData.isSTLModel = false;  // Flag to indicate this is not an STL model
        this.scene.add(this.vessel);

        // Reset camera
        this.resetCamera();

        // Reattach existing components
        this.reattachComponents();
    }

    reattachComponents() {
        // Reattach control surfaces
        this.controlSurfaces.forEach((surface, id) => {
            if (surface.parent) {
                surface.parent.remove(surface);
            }
            this.vessel.add(surface);
        });

        // Reattach thrusters
        this.thrusters.forEach((thruster, id) => {
            if (thruster.parent) {
                thruster.parent.remove(thruster);
            }
            this.vessel.add(thruster);
        });

        // Reattach sensors
        this.sensors.forEach((sensor, id) => {
            if (sensor.parent) {
                sensor.parent.remove(sensor);
            }
            this.vessel.add(sensor);
        });
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

    addControlSurface(id, type, position, orientation) {
        const group = new THREE.Group();
        group.userData.id = id;
        group.userData.type = 'control_surface';
        group.userData.isTransformable = true;
        group.name = `${type} ${id}`;
        
        // Create the control surface geometry
        const width = 0.2;
        const height = 0.3;
        const geometry = new THREE.PlaneGeometry(width, height);
        const material = new THREE.MeshPhongMaterial({
            color: 0x3498db,
            transparent: true,
            opacity: 0.7,
            side: THREE.DoubleSide
        });
        
        const mesh = new THREE.Mesh(geometry, material);
        mesh.userData.isControlSurface = true;
        mesh.userData.parentId = id;
        group.add(mesh);

        // Add axes to the group
        this.addComponentAxes(group);

        // Store dimensions in userData
        group.userData.dimensions = { width, height };
        group.userData.parameters = {
            type: type,
            area: width * height,
            naca: "0015"
        };

        // Set position and orientation
        group.position.copy(position);
        if (orientation) {
            group.rotation.setFromVector3(orientation);
        }

        this.scene.add(group);
        return group;
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

    addThruster(id, position, orientation) {
        if (!this.vessel) {
            this.createVessel(1, 1, 1);
        }

        const geometry = new THREE.CylinderGeometry(0.2, 0.3, 0.6);
        const material = new THREE.MeshPhongMaterial({
            color: 0x2ecc71,
            transparent: true,
            opacity: 0.8
        });

        const thruster = new THREE.Mesh(geometry, material);
        thruster.userData.type = 'thruster';
        thruster.userData.id = id;

        // Create container with thruster and axes
        const container = this.addComponentAxes(thruster);
        container.position.set(...position);
        container.rotation.set(...orientation);

        this.thrusters.set(id, container);
        this.vessel.add(container);
        return container;
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

    addSensor(id, type, position) {
        if (!this.vessel) {
            this.createVessel(1, 1, 1);
        }

        let geometry, material;
        switch (type) {
            case 'IMU':
                geometry = new THREE.BoxGeometry(0.2, 0.2, 0.2);
                material = new THREE.MeshPhongMaterial({ color: 0xf1c40f });
                break;
            case 'GPS':
                geometry = new THREE.SphereGeometry(0.1);
                material = new THREE.MeshPhongMaterial({ color: 0x9b59b6 });
                break;
            case 'DVL':
                geometry = new THREE.ConeGeometry(0.1, 0.2);
                material = new THREE.MeshPhongMaterial({ color: 0xe67e22 });
                break;
            default:
                geometry = new THREE.BoxGeometry(0.1, 0.1, 0.1);
                material = new THREE.MeshPhongMaterial({ color: 0x95a5a6 });
        }

        const sensor = new THREE.Mesh(geometry, material);
        sensor.userData.type = 'sensor';
        sensor.userData.id = id;
        sensor.userData.sensorType = type;
        sensor.name = `${type} ${id}`;  // Set initial name

        // Create container with sensor and axes
        const container = new THREE.Group();
        container.userData = { ...sensor.userData };  // Copy userData to container
        container.name = sensor.name;  // Copy name to container
        container.add(sensor);
        
        // Add axes to container
        const axes = this.createLocalAxes();
        axes.userData.isAxes = true;
        axes.userData.parentId = id;
        container.add(axes);

        // Set position
        container.position.set(...position);

        this.sensors.set(id, container);
        this.vessel.add(container);
        return container;
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
        const rect = this.renderer.domElement.getBoundingClientRect();
        this.mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
        this.mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
    }

    onClick(event) {
        if (event.target !== this.renderer.domElement) return;

        this.raycaster.setFromCamera(this.mouse, this.camera);
        
        // Create array of selectable objects
        const selectableObjects = [];
        
        if (this.vessel) {
            // For STL models, add both the group and the STL mesh
            if (this.vessel.userData.isSTLModel) {
                selectableObjects.push(this.vessel.userData.stlMesh);
            }
            selectableObjects.push(this.vessel);
            
            // Add components
            this.controlSurfaces.forEach(surface => selectableObjects.push(surface));
            this.thrusters.forEach(thruster => selectableObjects.push(thruster));
            this.sensors.forEach(sensor => selectableObjects.push(sensor));
        }

        const intersects = this.raycaster.intersectObjects(selectableObjects, true);

        if (intersects.length > 0) {
            let selectedObject = intersects[0].object;
            
            // If we clicked the STL mesh, select the vessel group
            if (this.vessel?.userData.isSTLModel && selectedObject === this.vessel.userData.stlMesh) {
                selectedObject = this.vessel;
            }
            
            this.selectObject(selectedObject);
        } else {
            if (this.selectedObject) {
                this.transformControls.detach();
                this.selectedObject = null;
                document.querySelectorAll('.component-card').forEach(card => {
                    card.classList.remove('selected');
                });
            }
        }
        
        this.render();
    }

    selectObject(object) {
        if (this.selectedObject === object) return;
        
        // Update selection
        if (this.selectedObject) {
            this.selectedObject.userData.previousColor = undefined;
        }
        
        this.selectedObject = object;
        
        // Highlight selected object
        if (object && object.material) {
            object.userData.previousColor = object.material.color.clone();
            object.material.color.setHex(0x3498db);
        }
        
        // Update transform controls
        if (object && object.userData.isTransformable) {
            this.transformControls.attach(object);
        } else {
            this.transformControls.detach();
        }
        
        // Update UI
        this.updateTransformInfo();
        this.updateSceneHierarchy();
        
        // Show appropriate parameter panel
        this.showComponentParams(object);
    }

    showComponentParams(object) {
        const paramsContainer = document.querySelector('.component-params');
        if (!paramsContainer) return;
        
        paramsContainer.innerHTML = '';
        
        if (!object || !object.userData.type) return;
        
        switch (object.userData.type) {
            case 'controlSurface':
                this.createControlSurfaceParams(object);
                break;
            case 'thruster':
                this.createThrusterParams(object);
                break;
            case 'sensor':
                this.createSensorParams(object);
                break;
        }
    }

    createControlSurfaceParams(object) {
        const container = document.createElement('div');
        container.className = 'component-params';
        
        container.innerHTML = `
            <div class="mb-3">
                <label>Type</label>
                <select class="form-select" id="surfaceType">
                    <option value="Rudder" ${object.userData.surfaceType === 'Rudder' ? 'selected' : ''}>Rudder</option>
                    <option value="Fin" ${object.userData.surfaceType === 'Fin' ? 'selected' : ''}>Fin</option>
                </select>
            </div>
            <div class="mb-3">
                <label>Name</label>
                <input type="text" class="form-control" id="surfaceName" value="${object.name || ''}">
            </div>
            <div class="mb-3">
                <label>NACA Profile</label>
                <input type="text" class="form-control" id="nacaProfile" value="${object.userData.nacaProfile || '0015'}">
            </div>
        `;
        
        // Add event listeners
        container.querySelector('#surfaceType').addEventListener('change', (e) => {
            object.userData.surfaceType = e.target.value;
            this.updateSceneHierarchy();
        });
        
        container.querySelector('#surfaceName').addEventListener('change', (e) => {
            object.name = e.target.value;
            this.updateSceneHierarchy();
        });
        
        container.querySelector('#nacaProfile').addEventListener('change', (e) => {
            object.userData.nacaProfile = e.target.value;
        });
        
        document.querySelector('.component-params').replaceWith(container);
    }

    createThrusterParams(object) {
        const paramsDiv = document.createElement('div');
        paramsDiv.className = 'component-params';

        // Type selection
        const typeGroup = document.createElement('div');
        typeGroup.className = 'param-group';
        const typeLabel = document.createElement('label');
        typeLabel.textContent = 'Thruster Type';
        const typeSelect = document.createElement('select');
        typeSelect.className = 'form-select form-select-sm';
        ['Propeller', 'Tunnel', 'Vectored', 'Jet'].forEach(type => {
            const option = document.createElement('option');
            option.value = type;
            option.textContent = type;
            if (object.userData.thrusterType === type) {
                option.selected = true;
            }
            typeSelect.appendChild(option);
        });
        typeSelect.addEventListener('change', (e) => {
            object.userData.thrusterType = e.target.value;
            object.name = `${e.target.value} #${object.userData.id}`;
            this.updateSceneHierarchy();
        });
        typeGroup.appendChild(typeLabel);
        typeGroup.appendChild(typeSelect);
        paramsDiv.appendChild(typeGroup);

        // Max Thrust
        const thrustGroup = document.createElement('div');
        thrustGroup.className = 'param-group';
        const thrustLabel = document.createElement('label');
        thrustLabel.textContent = 'Max Thrust (N)';
        const thrustInput = document.createElement('input');
        thrustInput.type = 'number';
        thrustInput.className = 'form-control form-control-sm';
        thrustInput.value = object.userData.maxThrust || 1000;
        thrustInput.min = 0;
        thrustInput.step = 100;
        thrustInput.addEventListener('change', (e) => {
            object.userData.maxThrust = parseFloat(e.target.value);
            this.updateThrusterSize(object.userData.id, e.target.value);
        });
        thrustGroup.appendChild(thrustLabel);
        thrustGroup.appendChild(thrustInput);
        paramsDiv.appendChild(thrustGroup);

        // Name input
        const nameGroup = document.createElement('div');
        nameGroup.className = 'param-group';
        const nameLabel = document.createElement('label');
        nameLabel.textContent = 'Name';
        const nameInput = document.createElement('input');
        nameInput.type = 'text';
        nameInput.className = 'form-control form-control-sm';
        nameInput.value = object.name || '';
        nameInput.addEventListener('change', (e) => {
            object.name = e.target.value;
            this.updateSceneHierarchy();
        });
        nameGroup.appendChild(nameLabel);
        nameGroup.appendChild(nameInput);
        paramsDiv.appendChild(nameGroup);

        // Replace the current parameter container in DOM
        const container = document.querySelector('.component-params');
        if (container) {
            container.replaceWith(paramsDiv);
        }
    }

    createSensorParams(object) {
        const paramsDiv = document.createElement('div');
        paramsDiv.className = 'component-params';

        // Type selection
        const typeGroup = document.createElement('div');
        typeGroup.className = 'param-group mb-3';
        const typeLabel = document.createElement('label');
        typeLabel.textContent = 'Sensor Type';
        const typeSelect = document.createElement('select');
        typeSelect.className = 'form-select form-select-sm';
        ['IMU', 'GPS', 'DVL', 'Pressure', 'Camera', 'Sonar'].forEach(type => {
            const option = document.createElement('option');
            option.value = type;
            option.textContent = type;
            if (object.userData.sensorType === type) {
                option.selected = true;
            }
            typeSelect.appendChild(option);
        });
        typeSelect.addEventListener('change', (e) => {
            object.userData.sensorType = e.target.value;
            // Update name if it follows the default pattern
            if (object.name.match(/^[A-Za-z]+ #\d+$/)) {
                object.name = `${e.target.value} #${object.userData.id}`;
            }
            this.updateSceneHierarchy();
        });
        typeGroup.appendChild(typeLabel);
        typeGroup.appendChild(typeSelect);
        paramsDiv.appendChild(typeGroup);

        // Sensor Name input
        const nameGroup = document.createElement('div');
        nameGroup.className = 'param-group mb-3';
        const nameLabel = document.createElement('label');
        nameLabel.textContent = 'Name';
        const nameInput = document.createElement('input');
        nameInput.type = 'text';
        nameInput.className = 'form-control form-control-sm';
        nameInput.value = object.name || `${object.userData.sensorType} #${object.userData.id}`;
        nameInput.addEventListener('change', (e) => {
            object.name = e.target.value;
            this.updateSceneHierarchy();
        });
        nameGroup.appendChild(nameLabel);
        nameGroup.appendChild(nameInput);
        paramsDiv.appendChild(nameGroup);

        // Update Rate
        const rateGroup = document.createElement('div');
        rateGroup.className = 'param-group mb-3';
        const rateLabel = document.createElement('label');
        rateLabel.textContent = 'Update Rate (Hz)';
        const rateInput = document.createElement('input');
        rateInput.type = 'number';
        rateInput.className = 'form-control form-control-sm';
        rateInput.value = object.userData.updateRate || 50;
        rateInput.min = 1;
        rateInput.max = 1000;
        rateInput.step = 1;
        rateInput.addEventListener('change', (e) => {
            object.userData.updateRate = parseFloat(e.target.value);
        });
        rateGroup.appendChild(rateLabel);
        rateGroup.appendChild(rateInput);
        paramsDiv.appendChild(rateGroup);

        // Replace the current parameter container in DOM
        const container = document.querySelector('.component-params');
        if (container) {
            container.replaceWith(paramsDiv);
        }
    }

    toggleGrid() {
        this.grid.visible = !this.grid.visible;
    }

    toggleWireframe() {
        if (this.vessel) {
            if (this.vessel.userData.isSTLModel) {
                if (this.vessel.userData.stlMesh) {
                    this.vessel.userData.stlMesh.material.wireframe = !this.vessel.userData.stlMesh.material.wireframe;
                }
            } else {
                this.vessel.material.wireframe = !this.vessel.material.wireframe;
            }
            
            // Also toggle wireframe for components
            this.controlSurfaces.forEach(surface => {
                surface.material.wireframe = !surface.material.wireframe;
            });
            this.thrusters.forEach(thruster => {
                thruster.material.wireframe = !thruster.material.wireframe;
            });
            this.sensors.forEach(sensor => {
                sensor.material.wireframe = !sensor.material.wireframe;
            });
            
            this.render();
        }
    }

    resetCamera() {
        // Calculate bounding box of the entire scene
        const bbox = new THREE.Box3().setFromObject(this.vessel);
        const center = new THREE.Vector3();
        bbox.getCenter(center);
        const size = new THREE.Vector3();
        bbox.getSize(size);
        
        // Calculate camera position
        const maxDim = Math.max(size.x, size.y, size.z);
        const fov = this.camera.fov * (Math.PI / 180);
        const cameraZ = Math.abs(maxDim / Math.tan(fov / 2)) * 2.5; // Multiply by 2.5 to add some padding
        
        // Position camera
        this.camera.position.set(
            center.x + cameraZ * 0.5,
            center.y + cameraZ * 0.5,
            center.z + cameraZ
        );
        
        // Look at center of model
        this.controls.target.copy(center);
        this.camera.lookAt(center);
        
        // Update camera
        this.camera.near = cameraZ / 100;
        this.camera.far = cameraZ * 100;
        this.camera.updateProjectionMatrix();
        
        // Update controls
        this.controls.update();
        
        console.log('Camera reset:', {
            position: this.camera.position.clone(),
            target: this.controls.target.clone(),
            distance: cameraZ
        });
    }

    animate() {
        requestAnimationFrame(() => this.animate());
        this.controls.update();
        this.renderer.render(this.scene, this.camera);
    }

    updateRendererSize() {
        const container = document.getElementById('viewer3D');
        const width = container.clientWidth;
        const height = container.clientHeight;
        
        this.renderer.setSize(width, height);
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
    }

    render() {
        this.renderer.render(this.scene, this.camera);
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
        if (this.bodyCenter) {
            this.scene.remove(this.bodyCenter);
            this.bodyCenter = null;
        }
        this.updateSceneHierarchy();
    }

    // Add new method to update scene hierarchy
    updateSceneHierarchy() {
        const sceneHierarchy = document.getElementById('sceneHierarchy');
        if (!sceneHierarchy) return;
        
        // Clear existing content
        sceneHierarchy.innerHTML = '';
        
        const createHierarchyItem = (object, level = 0) => {
            const item = document.createElement('div');
            item.className = 'hierarchy-item';
            if (object === this.selectedObject) {
                item.classList.add('selected');
            }
            
            // Add toggle button if object has children
            if (object.children.length > 0 && !object.userData.isAxes) {
                const toggleBtn = document.createElement('button');
                toggleBtn.className = 'toggle-children';
                toggleBtn.innerHTML = '<i class="bi bi-chevron-right"></i>';
                item.appendChild(toggleBtn);
                
                // Create container for children
                const childrenDiv = document.createElement('div');
                childrenDiv.className = 'hierarchy-children';
                childrenDiv.style.display = 'none';
                
                // Add children that aren't axes
                object.children.forEach(child => {
                    if (!child.userData.isAxes) {
                        childrenDiv.appendChild(createHierarchyItem(child, level + 1));
                    }
                });
                
                // Toggle children visibility
                toggleBtn.addEventListener('click', (e) => {
                    e.stopPropagation();
                    const isExpanded = childrenDiv.style.display !== 'none';
                    childrenDiv.style.display = isExpanded ? 'none' : 'block';
                    toggleBtn.innerHTML = isExpanded ? 
                        '<i class="bi bi-chevron-right"></i>' : 
                        '<i class="bi bi-chevron-down"></i>';
                });
                
                // Append children container
                item.appendChild(childrenDiv);
            }
            
            // Add icon based on object type
            const icon = document.createElement('i');
            if (object.userData.type === 'controlSurface') {
                icon.className = 'bi bi-box';
            } else if (object.userData.type === 'thruster') {
                icon.className = 'bi bi-circle';
            } else if (object.userData.type === 'sensor') {
                icon.className = 'bi bi-cpu';
            } else {
                icon.className = 'bi bi-box';
            }
            item.appendChild(icon);
            
            // Add label
            const label = document.createElement('span');
            label.textContent = object.name || 'Unnamed Object';
            item.appendChild(label);
            
            // Add click handler
            item.addEventListener('click', (e) => {
                e.stopPropagation();
                this.selectObject(object);
            });
            
            return item;
        };
        
        // Add vessel and its components
        if (this.vessel) {
            sceneHierarchy.appendChild(createHierarchyItem(this.vessel));
        }
    }
}

// Export the class
window.ThreeScene = ThreeScene; 