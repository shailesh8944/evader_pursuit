document.addEventListener('DOMContentLoaded', () => {
    // Initialize Three.js scene and vessel model
    const threeScene = new ThreeScene();
    const vesselModel = new VesselModel();
    const yamlGenerator = new YAMLGenerator();

    // Create initial vessel with default dimensions
    threeScene.createVessel(1, 1, 1);
    vesselModel.updateBasicProperties('New Vessel', 'auv', 1, 1, 1);

    // UI Elements
    const vesselNameInput = document.getElementById('vesselName');
    const vesselTypeSelect = document.getElementById('vesselType');
    const lengthInput = document.getElementById('length');
    const breadthInput = document.getElementById('breadth');
    const depthInput = document.getElementById('depth');

    // Physical Properties
    const massInput = document.getElementById('mass');
    const cgInputs = {
        x: document.getElementById('cg_x'),
        y: document.getElementById('cg_y'),
        z: document.getElementById('cg_z')
    };

    // Inertia Matrix
    const inertiaInputs = {
        xx: document.getElementById('i_xx'),
        xy: document.getElementById('i_xy'),
        xz: document.getElementById('i_xz'),
        yx: document.getElementById('i_yx'),
        yy: document.getElementById('i_yy'),
        yz: document.getElementById('i_yz'),
        zx: document.getElementById('i_zx'),
        zy: document.getElementById('i_zy'),
        zz: document.getElementById('i_zz')
    };

    // Transform Info Inputs
    const positionInputs = {
        x: document.getElementById('pos_x'),
        y: document.getElementById('pos_y'),
        z: document.getElementById('pos_z')
    };

    const rotationInputs = {
        x: document.getElementById('rot_x'),
        y: document.getElementById('rot_y'),
        z: document.getElementById('rot_z')
    };

    const scaleInputs = {
        x: document.getElementById('scale_x'),
        y: document.getElementById('scale_y'),
        z: document.getElementById('scale_z')
    };

    // Buttons
    const addControlSurfaceBtn = document.getElementById('addControlSurface');
    const addThrusterBtn = document.getElementById('addThruster');
    const addSensorBtn = document.getElementById('addSensor');
    const generateYAMLBtn = document.getElementById('generateYAML');
    const saveConfigBtn = document.getElementById('saveConfig');
    const loadConfigBtn = document.getElementById('loadConfig');
    const resetCameraBtn = document.getElementById('resetCamera');
    const toggleGridBtn = document.getElementById('toggleGrid');
    const toggleWireframeBtn = document.getElementById('toggleWireframe');
    const sidebarToggle = document.querySelector('.sidebar-toggle');

    // Context Menu
    const contextMenu = document.querySelector('.context-menu');
    let contextMenuPosition = null;

    // Sidebar resize functionality
    const sidebar = document.querySelector('.sidebar');
    const resizeHandle = document.querySelector('.sidebar-resize-handle');
    const mainContent = document.querySelector('.main-content');
    let isResizing = false;
    let lastDownX = 0;

    resizeHandle.addEventListener('mousedown', (e) => {
        isResizing = true;
        lastDownX = e.clientX;
        resizeHandle.classList.add('active');
        document.body.style.cursor = 'col-resize';
        document.body.style.userSelect = 'none';
    });

    document.addEventListener('mousemove', (e) => {
        if (!isResizing) return;

        const delta = e.clientX - lastDownX;
        const newWidth = sidebar.offsetWidth + delta;

        // Constrain width between min and max values
        if (newWidth >= 300 && newWidth <= 600) {
            sidebar.style.width = `${newWidth}px`;
            lastDownX = e.clientX;
            
            // Update Three.js renderer size
            threeScene.onWindowResize();
        }
    });

    document.addEventListener('mouseup', () => {
        if (isResizing) {
            isResizing = false;
            resizeHandle.classList.remove('active');
            document.body.style.cursor = '';
            document.body.style.userSelect = '';
        }
    });

    // Sidebar Toggle
    sidebarToggle.addEventListener('click', () => {
        sidebar.classList.toggle('collapsed');
        mainContent.classList.toggle('expanded');
        sidebarToggle.querySelector('i').classList.toggle('bi-chevron-right');
        sidebarToggle.querySelector('i').classList.toggle('bi-chevron-left');
        threeScene.onWindowResize();
    });

    // Setup visibility toggles
    const stlToggle = document.getElementById('toggleSTLVisibility');
    const boxToggle = document.getElementById('toggleBoxVisibility');
    
    stlToggle.addEventListener('change', (e) => {
        threeScene.toggleSTL(e.target.checked);
    });
    
    boxToggle.addEventListener('change', (e) => {
        threeScene.toggleBox(e.target.checked);
    });

    // Setup context menu
    threeScene.renderer.domElement.addEventListener('contextmenu', (e) => {
        e.preventDefault();
        contextMenu.style.display = 'block';
        contextMenu.style.left = `${e.clientX}px`;
        contextMenu.style.top = `${e.clientY}px`;
        contextMenuPosition = new THREE.Vector2(
            ((e.clientX - threeScene.renderer.domElement.offsetLeft) / threeScene.renderer.domElement.clientWidth) * 2 - 1,
            -((e.clientY - threeScene.renderer.domElement.offsetTop) / threeScene.renderer.domElement.clientHeight) * 2 + 1
        );
    });
    
    document.addEventListener('click', (e) => {
        if (!contextMenu.contains(e.target)) {
            contextMenu.style.display = 'none';
        }
    });
    
    // Setup context menu actions
    document.querySelectorAll('.context-menu-item').forEach(item => {
        item.addEventListener('click', (e) => {
            const action = e.currentTarget.dataset.action;
            const position = threeScene.getWorldPositionFromMouse(contextMenuPosition);
            
            switch(action) {
                case 'add-rudder':
                    threeScene.addControlSurface(vesselModel.nextControlSurfaceId++, 'Rudder', position);
                    break;
                case 'add-fin':
                    threeScene.addControlSurface(vesselModel.nextControlSurfaceId++, 'Fin', position);
                    break;
                case 'add-thruster':
                    threeScene.addThruster(vesselModel.nextThrusterId++, position);
                    break;
                case 'add-sensor':
                    threeScene.showSensorDialog(position);
                    break;
            }
            
            contextMenu.style.display = 'none';
        });
    });

    // Transform Controls Event Listeners
    Object.entries(positionInputs).forEach(([axis, input]) => {
        input.addEventListener('change', () => {
            if (threeScene.selectedObject) {
                threeScene.selectedObject.position[axis] = parseFloat(input.value) || 0;
                onObjectTransformed(threeScene.selectedObject);
            }
        });
    });

    Object.entries(rotationInputs).forEach(([axis, input]) => {
        input.addEventListener('change', () => {
            if (threeScene.selectedObject) {
                threeScene.selectedObject.rotation[axis] = parseFloat(input.value) || 0;
                onObjectTransformed(threeScene.selectedObject);
            }
        });
    });

    Object.entries(scaleInputs).forEach(([axis, input]) => {
        input.addEventListener('change', () => {
            if (threeScene.selectedObject) {
                threeScene.selectedObject.scale[axis] = parseFloat(input.value) || 1;
                onObjectTransformed(threeScene.selectedObject);
            }
        });
    });

    // Event Listeners for Basic Properties
    function updateBasicProperties() {
        const name = vesselNameInput.value || 'New Vessel';
        const type = vesselTypeSelect.value || 'auv';
        const length = parseFloat(lengthInput.value) || 1;
        const breadth = parseFloat(breadthInput.value) || 1;
        const depth = parseFloat(depthInput.value) || 1;

        vesselModel.updateBasicProperties(name, type, length, breadth, depth);
        threeScene.createVessel(length, breadth, depth);
    }

    vesselNameInput.addEventListener('input', updateBasicProperties);
    vesselTypeSelect.addEventListener('change', updateBasicProperties);
    lengthInput.addEventListener('input', updateBasicProperties);
    breadthInput.addEventListener('input', updateBasicProperties);
    depthInput.addEventListener('input', updateBasicProperties);

    // Physical Properties Event Listeners
    function updatePhysicalProperties() {
        const mass = parseFloat(massInput.value) || 0;
        const cg = [
            parseFloat(cgInputs.x.value) || 0,
            parseFloat(cgInputs.y.value) || 0,
            parseFloat(cgInputs.z.value) || 0
        ];

        const inertiaMatrix = [
            [parseFloat(inertiaInputs.xx.value) || 0, parseFloat(inertiaInputs.xy.value) || 0, parseFloat(inertiaInputs.xz.value) || 0],
            [parseFloat(inertiaInputs.yx.value) || 0, parseFloat(inertiaInputs.yy.value) || 0, parseFloat(inertiaInputs.yz.value) || 0],
            [parseFloat(inertiaInputs.zx.value) || 0, parseFloat(inertiaInputs.zy.value) || 0, parseFloat(inertiaInputs.zz.value) || 0]
        ];

        vesselModel.updateInertiaProperties(mass, inertiaMatrix);
    }

    [massInput, ...Object.values(cgInputs), ...Object.values(inertiaInputs)]
        .forEach(input => input.addEventListener('change', updatePhysicalProperties));

    // Control Surface Management
    function addControlSurface(type, position = { x: 0, y: 0, z: 0 }) {
        try {
            const id = vesselModel.addControlSurface(
                type,
                [position.x, position.y, position.z],
                [0, 0, 0],
                type === 'Rudder' ? 0.5 : 0.3
            );

            const surface = threeScene.addControlSurface(id, type, [position.x, position.y, position.z], [0, 0, 0]);
            updateControlSurfacesList();
            return surface;
        } catch (error) {
            console.error('Error adding control surface:', error);
            return null;
        }
    }

    // Thruster Management
    function addThruster(position = { x: 0, y: 0, z: 0 }) {
        try {
            const id = vesselModel.addThruster(
                'Propeller',
                [position.x, position.y, position.z],
                [0, 0, 0],
                1000
            );

            const thruster = threeScene.addThruster(id, [position.x, position.y, position.z], [0, 0, 0]);
            updateThrustersList();
            return thruster;
        } catch (error) {
            console.error('Error adding thruster:', error);
            return null;
        }
    }

    // Sensor Management
    function showSensorDialog(position) {
        try {
            const types = ['IMU', 'GPS', 'DVL'];
            const type = types[Math.floor(Math.random() * types.length)];
            
            const id = vesselModel.addSensor(
                type,
                [position.x, position.y, position.z],
                [1, 0, 0, 0],
                100
            );

            const sensor = threeScene.addSensor(id, type, [position.x, position.y, position.z]);
            updateSensorsList();
            return sensor;
        } catch (error) {
            console.error('Error adding sensor:', error);
            return null;
        }
    }

    // Update Transform Info
    function updateTransformInputs(object) {
        if (!object) return;

        positionInputs.x.value = object.position.x.toFixed(3);
        positionInputs.y.value = object.position.y.toFixed(3);
        positionInputs.z.value = object.position.z.toFixed(3);

        rotationInputs.x.value = object.rotation.x.toFixed(3);
        rotationInputs.y.value = object.rotation.y.toFixed(3);
        rotationInputs.z.value = object.rotation.z.toFixed(3);

        scaleInputs.x.value = object.scale.x.toFixed(3);
        scaleInputs.y.value = object.scale.y.toFixed(3);
        scaleInputs.z.value = object.scale.z.toFixed(3);
    }

    function updateComponentTransform(object) {
        if (!object || !object.userData.card) return;
        
        const card = object.userData.card;
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
                input.value = object.position[axis].toFixed(3);
            }
        });

        // Update rotation inputs
        Object.entries(rotInputs).forEach(([axis, input]) => {
            if (input) {
                input.value = object.rotation[axis].toFixed(3);
            }
        });
    }

    // Update the onObjectTransformed callback
    function onObjectTransformed(object) {
        if (!object || !object.userData.type) return;

        updateComponentTransform(object);
        updateTransformInputs(object);

        switch (object.userData.type) {
            case 'controlSurface':
                vesselModel.updateControlSurfaceTransform(
                    object.userData.id,
                    [object.position.x, object.position.y, object.position.z],
                    [object.rotation.x, object.rotation.y, object.rotation.z]
                );
                updateControlSurfacesList();
                break;

            case 'thruster':
                vesselModel.updateThrusterTransform(
                    object.userData.id,
                    [object.position.x, object.position.y, object.position.z],
                    [object.rotation.x, object.rotation.y, object.rotation.z]
                );
                updateThrustersList();
                break;

            case 'sensor':
                vesselModel.updateSensorTransform(
                    object.userData.id,
                    [object.position.x, object.position.y, object.position.z],
                    [1, 0, 0, 0] // TODO: Convert Euler to Quaternion
                );
                updateSensorsList();
                break;
        }
    }

    // Assign the callback to the Three.js scene
    threeScene.onObjectTransformed = onObjectTransformed;

    // Component List Updates
    function updateControlSurfacesList() {
        const list = document.getElementById('controlSurfacesList');
        list.innerHTML = '';
        vesselModel.config.control_surfaces.control_surfaces.forEach(surface => {
            const object = threeScene.getObjectById(surface.control_surface_id);
            if (!object) return;

            const card = createComponentCard(
                `${surface.control_surface_type} #${surface.control_surface_id}`,
                `<div class="component-info">
                    <div class="info-row">
                        <label>Area:</label>
                        <input type="number" class="form-control form-control-sm area-input" 
                               value="${surface.control_surface_area.toFixed(2)}" step="0.1" min="0">
                        <span class="unit">m²</span>
                    </div>
                </div>`,
                () => {
                    vesselModel.removeControlSurface(surface.control_surface_id);
                    threeScene.removeControlSurface(surface.control_surface_id);
                    updateControlSurfacesList();
                },
                object
            );

            // Add area change handler
            const areaInput = card.querySelector('.area-input');
            areaInput.addEventListener('input', () => {
                const area = parseFloat(areaInput.value) || 0;
                vesselModel.updateControlSurfaceArea(surface.control_surface_id, area);
                threeScene.updateControlSurfaceSize(surface.control_surface_id, area);
            });

            list.appendChild(card);
        });
    }

    function updateThrustersList() {
        const list = document.getElementById('thrustersList');
        list.innerHTML = '';
        if (vesselModel.config.thrusters) {
            vesselModel.config.thrusters.thrusters.forEach(thruster => {
                const object = threeScene.getObjectById(thruster.thruster_id);
                if (!object) return;

                const card = createComponentCard(
                    `${thruster.thruster_type} #${thruster.thruster_id}`,
                    `<div class="component-info">
                        <div class="info-row">
                            <label>Max Thrust:</label>
                            <input type="number" class="form-control form-control-sm thrust-input" 
                                   value="${thruster.max_thrust.toFixed(0)}" step="100" min="0">
                            <span class="unit">N</span>
                        </div>
                    </div>`,
                    () => {
                        vesselModel.removeThruster(thruster.thruster_id);
                        threeScene.removeThruster(thruster.thruster_id);
                        updateThrustersList();
                    },
                    object
                );

                // Add thrust change handler
                const thrustInput = card.querySelector('.thrust-input');
                thrustInput.addEventListener('input', () => {
                    const thrust = parseFloat(thrustInput.value) || 0;
                    vesselModel.updateThrusterThrust(thruster.thruster_id, thrust);
                    threeScene.updateThrusterSize(thruster.thruster_id, thrust);
                });

                list.appendChild(card);
            });
        }
    }

    function updateSensorsList() {
        const list = document.getElementById('sensorsList');
        list.innerHTML = '';
        vesselModel.config.sensors.sensors.forEach(sensor => {
            const object = threeScene.getObjectById(sensor.sensor_id);
            if (!object) return;

            const card = createComponentCard(
                `${sensor.sensor_type} #${sensor.sensor_id}`,
                `<div class="component-info">
                    <div class="info-row">
                        <label>Rate:</label>
                        <input type="number" class="form-control form-control-sm rate-input" 
                               value="${sensor.publish_rate.toFixed(1)}" step="1" min="0">
                        <span class="unit">Hz</span>
                    </div>
                </div>`,
                () => {
                    vesselModel.removeSensor(sensor.sensor_id);
                    threeScene.removeSensor(sensor.sensor_id);
                    updateSensorsList();
                },
                object
            );

            // Add rate change handler
            const rateInput = card.querySelector('.rate-input');
            rateInput.addEventListener('input', () => {
                const rate = parseFloat(rateInput.value) || 0;
                vesselModel.updateSensorRate(sensor.sensor_id, rate);
            });

            list.appendChild(card);
        });
    }

    function createComponentCard(title, content, onDelete, object) {
        const card = document.createElement('div');
        card.className = 'component-card fade-in';
        
        // Create editable fields for the component
        const transformInputs = `
            <div class="transform-inputs mt-3">
                <div class="transform-group">
                    <label>Position (m)</label>
                    <div class="d-flex gap-2">
                        <input type="number" class="form-control form-control-sm pos-x" value="${object.position.x.toFixed(3)}" step="0.1">
                        <input type="number" class="form-control form-control-sm pos-y" value="${object.position.y.toFixed(3)}" step="0.1">
                        <input type="number" class="form-control form-control-sm pos-z" value="${object.position.z.toFixed(3)}" step="0.1">
                    </div>
                </div>
                <div class="transform-group">
                    <label>Rotation (rad)</label>
                    <div class="d-flex gap-2">
                        <input type="number" class="form-control form-control-sm rot-x" value="${object.rotation.x.toFixed(3)}" step="0.1">
                        <input type="number" class="form-control form-control-sm rot-y" value="${object.rotation.y.toFixed(3)}" step="0.1">
                        <input type="number" class="form-control form-control-sm rot-z" value="${object.rotation.z.toFixed(3)}" step="0.1">
                    </div>
                </div>
            </div>
        `;

        card.innerHTML = `
            <div class="header">
                <h5>${title}</h5>
                <div class="component-controls">
                    <button class="btn btn-outline-primary btn-sm edit-btn me-2" title="Edit Component">
                        <i class="bi bi-pencil"></i>
                    </button>
                    <button class="btn btn-outline-danger btn-sm delete-btn" title="Delete Component">
                        <i class="bi bi-trash"></i>
                    </button>
                </div>
            </div>
            <div class="content">
                ${content}
                ${transformInputs}
            </div>
        `;

        // Add event listeners for transform inputs
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

        // Update 3D object when sidebar inputs change
        Object.entries(posInputs).forEach(([axis, input]) => {
            input.addEventListener('input', () => {
                object.position[axis] = parseFloat(input.value) || 0;
                onObjectTransformed(object);
                threeScene.render();
            });
        });

        Object.entries(rotInputs).forEach(([axis, input]) => {
            input.addEventListener('input', () => {
                object.rotation[axis] = parseFloat(input.value) || 0;
                onObjectTransformed(object);
                threeScene.render();
            });
        });

        // Edit button handler
        card.querySelector('.edit-btn').addEventListener('click', () => {
            threeScene.selectObject(object);
            updateTransformInputs(object);
        });

        card.querySelector('.delete-btn').addEventListener('click', onDelete);

        // Store reference to the card for updates
        object.userData.card = card;
        
        return card;
    }

    // YAML Generation and File Management
    generateYAMLBtn.addEventListener('click', () => {
        const files = vesselModel.generateYAMLFiles();
        const zip = new JSZip();
        
        Object.entries(files).forEach(([filename, content]) => {
            zip.file(filename, jsyaml.dump(content));
        });

        zip.generateAsync({ type: 'blob' }).then(blob => {
            const url = URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            a.download = `${vesselModel.config.name}_config.zip`;
            document.body.appendChild(a);
            a.click();
            document.body.removeChild(a);
            URL.revokeObjectURL(url);
        });
    });

    saveConfigBtn.addEventListener('click', () => {
        const config = JSON.stringify(vesselModel.config, null, 2);
        const blob = new Blob([config], { type: 'application/json' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `${vesselModel.config.name}_config.json`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
    });

    loadConfigBtn.addEventListener('click', () => {
        const input = document.createElement('input');
        input.type = 'file';
        input.accept = '.json';
        input.addEventListener('change', (e) => {
            const file = e.target.files[0];
            const reader = new FileReader();
            reader.onload = (e) => {
                const config = JSON.parse(e.target.result);
                vesselModel.config = config;
                
                // Update UI
                vesselNameInput.value = config.name;
                vesselTypeSelect.value = config.type;
                lengthInput.value = config.geometry.length;
                breadthInput.value = config.geometry.breadth;
                depthInput.value = config.geometry.depth;

                // Update physical properties
                massInput.value = config.inertia.mass;
                cgInputs.x.value = config.geometry.CG[0];
                cgInputs.y.value = config.geometry.CG[1];
                cgInputs.z.value = config.geometry.CG[2];

                // Update inertia matrix
                const im = config.inertia.inertia_matrix;
                inertiaInputs.xx.value = im[0][0];
                inertiaInputs.xy.value = im[0][1];
                inertiaInputs.xz.value = im[0][2];
                inertiaInputs.yx.value = im[1][0];
                inertiaInputs.yy.value = im[1][1];
                inertiaInputs.yz.value = im[1][2];
                inertiaInputs.zx.value = im[2][0];
                inertiaInputs.zy.value = im[2][1];
                inertiaInputs.zz.value = im[2][2];

                // Update 3D scene
                threeScene.createVessel(
                    config.geometry.length,
                    config.geometry.breadth,
                    config.geometry.depth
                );

                // Recreate all components
                config.control_surfaces.control_surfaces.forEach(surface => {
                    threeScene.addControlSurface(
                        surface.control_surface_id,
                        surface.control_surface_type,
                        surface.control_surface_location,
                        surface.control_surface_orientation
                    );
                });

                if (config.thrusters) {
                    config.thrusters.thrusters.forEach(thruster => {
                        threeScene.addThruster(
                            thruster.thruster_id,
                            thruster.thruster_location,
                            thruster.thruster_orientation
                        );
                    });
                }

                config.sensors.sensors.forEach(sensor => {
                    threeScene.addSensor(
                        sensor.sensor_id,
                        sensor.sensor_type,
                        sensor.sensor_location
                    );
                });

                // Update component lists
                updateControlSurfacesList();
                updateThrustersList();
                updateSensorsList();
            };
            reader.readAsText(file);
        });
        input.click();
    });

    // Button Event Listeners
    addControlSurfaceBtn.addEventListener('click', () => addControlSurface('Rudder'));
    addThrusterBtn.addEventListener('click', () => addThruster());
    addSensorBtn.addEventListener('click', () => showSensorDialog({ x: 0, y: 0, z: 0 }));
    resetCameraBtn.addEventListener('click', () => threeScene.resetCamera());
    toggleGridBtn.addEventListener('click', () => threeScene.toggleGrid());
    toggleWireframeBtn.addEventListener('click', () => threeScene.toggleWireframe());
    
    // Get toggle elements
    const toggleBoxBtn = document.getElementById('toggleBoxBtn');
    const toggleSTLBtn = document.getElementById('toggleSTLBtn');
    const toggleBoxVisibility = document.getElementById('toggleBoxVisibility');
    const toggleSTLVisibility = document.getElementById('toggleSTLVisibility');

    // Function to sync visibility states
    function syncVisibilityStates(type) {
        if (type === 'box') {
            const isVisible = toggleBoxVisibility.checked;
            toggleBoxBtn.classList.toggle('active', isVisible);
            threeScene.toggleBox(isVisible);
        } else if (type === 'stl') {
            const isVisible = toggleSTLVisibility.checked;
            toggleSTLBtn.classList.toggle('active', isVisible);
            threeScene.toggleSTL(isVisible);
        }
    }

    // Add event listeners for toggle buttons
    toggleBoxBtn.addEventListener('click', () => {
        toggleBoxVisibility.checked = !toggleBoxVisibility.checked;
        syncVisibilityStates('box');
    });

    toggleSTLBtn.addEventListener('click', () => {
        toggleSTLVisibility.checked = !toggleSTLVisibility.checked;
        syncVisibilityStates('stl');
    });

    // Add event listeners for visibility checkboxes
    toggleBoxVisibility.addEventListener('change', () => syncVisibilityStates('box'));
    toggleSTLVisibility.addEventListener('change', () => syncVisibilityStates('stl'));

    // Add STL file input handler
    const modelFileInput = document.getElementById('modelFile');
    modelFileInput.addEventListener('change', async (event) => {
        const file = event.target.files[0];
        if (file) {
            try {
                // Show loading indicator
                const loadingDiv = document.createElement('div');
                loadingDiv.className = 'loading';
                loadingDiv.textContent = 'Loading 3D model...';
                document.body.appendChild(loadingDiv);

                // Load the model file
                const result = await threeScene.loadModelGeometry(file);
                
                if (result.success) {
                    // Update input fields without triggering vessel recreation
                    lengthInput.removeEventListener('input', updateBasicProperties);
                    breadthInput.removeEventListener('input', updateBasicProperties);
                    depthInput.removeEventListener('input', updateBasicProperties);

                    // Update the dimensions
                    lengthInput.value = result.dimensions.length.toFixed(3);
                    breadthInput.value = result.dimensions.breadth.toFixed(3);
                    depthInput.value = result.dimensions.depth.toFixed(3);

                    // Update vessel model without recreating geometry
                    vesselModel.updateBasicProperties(
                        vesselNameInput.value || 'New Vessel',
                        vesselTypeSelect.value || 'auv',
                        result.dimensions.length,
                        result.dimensions.breadth,
                        result.dimensions.depth
                    );

                    // Reattach event listeners
                    lengthInput.addEventListener('input', updateBasicProperties);
                    breadthInput.addEventListener('input', updateBasicProperties);
                    depthInput.addEventListener('input', updateBasicProperties);

                    // Reset toggle buttons to active state
                    toggleBoxBtn.classList.add('active');
                    toggleSTLBtn.classList.add('active');

                    // Enable transform controls for the vessel
                    threeScene.selectObject(threeScene.vessel);
                } else {
                    alert('Failed to load 3D model: ' + result.error);
                }
            } catch (error) {
                console.error('Error loading 3D model:', error);
                alert('Error loading 3D model: ' + error.message);
            } finally {
                // Remove loading indicator
                const loadingDiv = document.querySelector('.loading');
                if (loadingDiv) {
                    document.body.removeChild(loadingDiv);
                }
            }
        }
    });

    // Add GDF file input handler for hydrodynamics
    const gdfFileInput = document.getElementById('gdfFile');
    if (gdfFileInput) {
        gdfFileInput.addEventListener('change', async (event) => {
            const file = event.target.files[0];
            if (file) {
                try {
                    // Show loading indicator
                    showLoading('Processing GDF file...');
                    
                    const reader = new FileReader();
                    reader.onload = (e) => {
                        try {
                            // Process GDF file content
                            const content = e.target.result;
                            // Add your GDF processing logic here
                            
                            hideLoading();
                            showNotification('GDF file processed successfully', 'success');
                        } catch (error) {
                            console.error('Error processing GDF file:', error);
                            showNotification('Error processing GDF file: ' + error.message, 'error');
                            hideLoading();
                        }
                    };
                    
                    reader.onerror = (error) => {
                        console.error('Error reading GDF file:', error);
                        showNotification('Error reading GDF file', 'error');
                        hideLoading();
                    };
                    
                    reader.readAsText(file);
                } catch (error) {
                    console.error('Error handling GDF file:', error);
                    showNotification('Error handling GDF file: ' + error.message, 'error');
                    hideLoading();
                }
            }
        });
    }

    // Transform Controls
    const transformControls = new THREE.TransformControls(threeScene.camera, threeScene.renderer.domElement);
    transformControls.addEventListener('change', () => {
        if (transformControls.object) {
            // Update transform info inputs
            updateTransformInputs(transformControls.object);
            
            // If moving the body center, update the CG inputs
            if (transformControls.object.userData.isBodyCenter) {
                const pos = transformControls.object.position;
                cgInputs.x.value = pos.x.toFixed(3);
                cgInputs.y.value = pos.y.toFixed(3);
                cgInputs.z.value = pos.z.toFixed(3);
                
                // Update vessel model
                vesselModel.config.geometry.CO = [pos.x, pos.y, pos.z];
            }
            
            threeScene.render();
        }
    });

    transformControls.addEventListener('mouseDown', () => {
        threeScene.controls.enabled = false;
    });

    transformControls.addEventListener('mouseUp', () => {
        threeScene.controls.enabled = true;
    });

    // Add event listeners for transform mode buttons
    document.getElementById('translateMode').addEventListener('click', () => {
        transformControls.setMode('translate');
        document.getElementById('translateMode').classList.add('active');
        document.getElementById('rotateMode').classList.remove('active');
        document.getElementById('scaleMode').classList.remove('active');
    });

    document.getElementById('rotateMode').addEventListener('click', () => {
        transformControls.setMode('rotate');
        document.getElementById('translateMode').classList.remove('active');
        document.getElementById('rotateMode').classList.add('active');
        document.getElementById('scaleMode').classList.remove('active');
    });

    document.getElementById('scaleMode').addEventListener('click', () => {
        transformControls.setMode('scale');
        document.getElementById('translateMode').classList.remove('active');
        document.getElementById('rotateMode').classList.remove('active');
        document.getElementById('scaleMode').classList.add('active');
    });

    // Add event listener for CG inputs to update body center position
    Object.entries(cgInputs).forEach(([axis, input]) => {
        input.addEventListener('change', () => {
            const bodyCenterGroup = threeScene.vessel.userData.bodyCenterGroup;
            if (bodyCenterGroup) {
                const value = parseFloat(input.value) || 0;
                bodyCenterGroup.position[axis] = value;
                vesselModel.config.geometry.CO[axis === 'x' ? 0 : axis === 'y' ? 1 : 2] = value;
                threeScene.render();
            }
        });
    });

    // Add keyboard shortcuts for transform modes
    document.addEventListener('keydown', (event) => {
        if (event.target.tagName === 'INPUT') return; // Don't handle if typing in input field
        
        switch (event.key.toLowerCase()) {
            case 'g':
                document.getElementById('translateMode').click();
                break;
            case 'r':
                document.getElementById('rotateMode').click();
                break;
            case 's':
                document.getElementById('scaleMode').click();
                break;
            case 'b':
                // Toggle body center selection
                const bodyCenterGroup = threeScene.vessel.userData.bodyCenterGroup;
                if (bodyCenterGroup) {
                    if (transformControls.object === bodyCenterGroup) {
                        transformControls.detach();
                    } else {
                        transformControls.attach(bodyCenterGroup);
                    }
                }
                break;
            case 'v':
                // Toggle vessel selection
                if (transformControls.object === threeScene.vessel) {
                    transformControls.detach();
                } else {
                    transformControls.attach(threeScene.vessel);
                }
                break;
        }
    });

    // Loading indicator functions
    function showLoading(message = 'Loading...') {
        const overlay = document.getElementById('loadingOverlay');
        const text = overlay.querySelector('.loading-text');
        text.textContent = message;
        overlay.style.display = 'flex';
    }

    function hideLoading() {
        const overlay = document.getElementById('loadingOverlay');
        overlay.style.display = 'none';
    }

    function showNotification(message, type = 'info') {
        // Create notification element
        const notification = document.createElement('div');
        notification.className = `notification notification-${type}`;
        notification.textContent = message;
        
        // Add to document
        document.body.appendChild(notification);
        
        // Remove after 5 seconds
        setTimeout(() => {
            notification.remove();
        }, 5000);
    }

    // Handle HydRA results zip file
    document.getElementById('hydraResults').addEventListener('change', async (event) => {
        const file = event.target.files[0];
        if (!file) return;

        try {
            showLoading('Processing HydRA results...');
            
            const zip = new JSZip();
            const zipContents = await zip.loadAsync(file);
            
            // Process each file in the zip
            for (const [filename, zipEntry] of Object.entries(zipContents.files)) {
                if (!zipEntry.dir) {
                    const content = await zipEntry.async('string');
                    try {
                        // Parse YAML content
                        const data = jsyaml.load(content);
                        
                        // Update vessel model with hydrodynamic data
                        if (filename.includes('hydro')) {
                            vesselModel.updateHydrodynamics(data);
                            showNotification('Hydrodynamic coefficients updated successfully', 'success');
                        }
                    } catch (error) {
                        console.error(`Error parsing ${filename}:`, error);
                    }
                }
            }
            
            hideLoading();
        } catch (error) {
            console.error('Error processing HydRA results:', error);
            showNotification('Error processing HydRA results: ' + error.message, 'error');
            hideLoading();
        }
    });
}); 