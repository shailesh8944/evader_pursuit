// Main application logic for the Marine Vessel CAD Configurator
document.addEventListener('DOMContentLoaded', function() {
    console.log("Initializing Marine Vessel CAD Configurator...");
    
    // Initialize core objects
    try {
        initializeApplication();
    } catch (error) {
        console.error("Error initializing application:", error);
        // Try again with a delay in case DOM elements aren't fully ready
        setTimeout(initializeApplication, 100);
    }
    
    // Main initialization function
    function initializeApplication() {
        try {
            console.log("Creating ThreeScene...");
            const threeScene = new ThreeScene();
            window.threeScene = threeScene; // Make it globally accessible
            
            console.log("Creating VesselModel...");
            const vesselModel = new VesselModel('New Vessel', 'auv');
            window.currentVesselModel = vesselModel;
            
            // Create initial vessel with default dimensions
            if (typeof threeScene.createVessel === 'function') {
                console.log("Creating default vessel...");
                const length = 1.0;
                const breadth = 0.5;
                const depth = 0.5;
                const vessel = threeScene.createVessel(length, breadth, depth);
                if (vessel) {
                    console.log("Default vessel created successfully");
                } else {
                    console.warn("Failed to create default vessel");
                }
            } else {
                console.error("createVessel method not found on threeScene");
            }
            
            // Populate form fields with initial values
            const nameInput = document.getElementById('vesselName');
            const typeSelect = document.getElementById('vesselType');
            
            if (nameInput) nameInput.value = vesselModel.config.name || 'New Vessel';
            if (typeSelect) typeSelect.value = vesselModel.config.type || 'auv';
            
            console.log("Starting animation loop...");
            threeScene.animate();
            
            // Initialize UI components
            initializeSidebars();
            initializeToolbar();
            initializeViewportControls(threeScene);
            initializeModals(threeScene);
            initializeFileHandlers(threeScene);
            initializeYAMLGeneration();
            
            console.log("Application initialized successfully");
        } catch (error) {
            console.error("Error in initializeApplication:", error);
            // Display a user-friendly error message
            const errorDiv = document.createElement('div');
            errorDiv.style.position = 'fixed';
            errorDiv.style.top = '50%';
            errorDiv.style.left = '50%';
            errorDiv.style.transform = 'translate(-50%, -50%)';
            errorDiv.style.padding = '20px';
            errorDiv.style.backgroundColor = '#f8d7da';
            errorDiv.style.color = '#721c24';
            errorDiv.style.borderRadius = '5px';
            errorDiv.style.zIndex = '9999';
            errorDiv.style.maxWidth = '80%';
            errorDiv.innerHTML = `
                <h4>Application Error</h4>
                <p>There was an error initializing the application: ${error.message}</p>
                <p>Please check the console for more details.</p>
                <button style="padding: 5px 10px; background: #dc3545; color: white; border: none; border-radius: 3px; cursor: pointer;">
                    Retry
                </button>
            `;
            document.body.appendChild(errorDiv);
            
            // Add retry functionality
            const retryButton = errorDiv.querySelector('button');
            if (retryButton) {
                retryButton.addEventListener('click', () => {
                    errorDiv.remove();
                    setTimeout(initializeApplication, 100);
                });
            }
        }
    }
    
    // --------- UI Initialization Functions ---------
    
    // Initialize sidebars
    function initializeSidebars() {
        // Tab switching in sidebars
        document.querySelectorAll('.sidebar-tab').forEach(tab => {
            tab.addEventListener('click', function() {
                const tabId = this.getAttribute('data-tab');
                const sidebar = this.closest('.left-sidebar, .right-sidebar');
                
                // Remove active class from all tabs and contents in this sidebar
                sidebar.querySelectorAll('.sidebar-tab').forEach(t => t.classList.remove('active'));
                sidebar.querySelectorAll('.sidebar-content').forEach(c => c.classList.remove('active'));
                
                // Add active class to selected tab and content
                this.classList.add('active');
                sidebar.querySelector(`#${tabId}`).classList.add('active');
            });
        });
        
        // Hierarchy expand/collapse
        document.getElementById('expandAll')?.addEventListener('click', function() {
            document.querySelectorAll('.hierarchy-children').forEach(el => {
                el.style.display = 'block';
                el.previousElementSibling.querySelector('.hierarchy-toggle').innerHTML = '<i class="bi bi-caret-down-fill"></i>';
            });
        });
        
        document.getElementById('collapseAll')?.addEventListener('click', function() {
            document.querySelectorAll('.hierarchy-children').forEach(el => {
                el.style.display = 'none';
                el.previousElementSibling.querySelector('.hierarchy-toggle').innerHTML = '<i class="bi bi-caret-right-fill"></i>';
            });
        });
    }
    
    // Initialize toolbar
    function initializeToolbar() {
        // New vessel
        document.getElementById('btn-new-vessel')?.addEventListener('click', function() {
            if (confirm('Create a new vessel? Any unsaved changes will be lost.')) {
                window.currentVesselModel = new VesselModel();
                
                // Clear any loaded model
                if (threeScene.vessel) {
                    threeScene.scene.remove(threeScene.vessel);
                    threeScene.vessel = null;
                }
                
                // Update scene hierarchy
                threeScene.updateSceneHierarchy();
                threeScene.render();
                
                // Clear form fields
                document.getElementById('vesselName').value = '';
                document.getElementById('vesselType').value = 'auv';
                
                // Show notification
                showNotification('New vessel created', 'success');
            }
        });
        
        // Save vessel configuration
        document.getElementById('btn-save-vessel')?.addEventListener('click', function() {
            const vesselName = window.currentVesselModel.config.name;
            if (!vesselName) {
                showNotification('Please enter a vessel name before saving', 'error');
                return;
            }
            
            const configJson = window.currentVesselModel.exportAsJson();
            const blob = new Blob([configJson], { type: 'application/json' });
            const url = URL.createObjectURL(blob);
            
            const a = document.createElement('a');
            a.href = url;
            a.download = `${vesselName}_config.json`;
            a.click();
            
            URL.revokeObjectURL(url);
            showNotification('Vessel configuration saved', 'success');
        });
        
        // Load vessel configuration
        document.getElementById('btn-load-vessel')?.addEventListener('click', function() {
            const input = document.createElement('input');
            input.type = 'file';
            input.accept = '.json';
            
            input.addEventListener('change', function() {
                if (this.files && this.files[0]) {
                    const file = this.files[0];
                    const reader = new FileReader();
                    
                    reader.onload = function(e) {
                        try {
                            const result = window.currentVesselModel.importFromJson(e.target.result);
                            
                            if (result) {
                                // Update UI with loaded values
                                document.getElementById('vesselName').value = window.currentVesselModel.config.name;
                                document.getElementById('vesselType').value = window.currentVesselModel.config.type;
                                
                                // Clear any loaded model
                                if (threeScene.vessel) {
                                    threeScene.scene.remove(threeScene.vessel);
                                    threeScene.vessel = null;
                                }
                                
                                // Update scene hierarchy
                                threeScene.updateSceneHierarchy();
                                threeScene.render();
                                
                                showNotification('Vessel configuration loaded', 'success');
                            } else {
                                showNotification('Failed to load configuration', 'error');
                            }
                        } catch (error) {
                            console.error('Error loading vessel configuration:', error);
                            showNotification('Invalid configuration file', 'error');
                        }
                    };
                    
                    reader.readAsText(file);
                }
            });
            
            input.click();
        });
        
        // Undo
        document.getElementById('btn-undo')?.addEventListener('click', function() {
            if (window.currentVesselModel.undo()) {
                showNotification('Undo successful', 'success');
                
                // Update UI if needed
                document.getElementById('vesselName').value = window.currentVesselModel.config.name;
                document.getElementById('vesselType').value = window.currentVesselModel.config.type;
            } else {
                showNotification('Nothing to undo', 'info');
            }
        });
        
        // Redo
        document.getElementById('btn-redo')?.addEventListener('click', function() {
            if (window.currentVesselModel.redo()) {
                showNotification('Redo successful', 'success');
                
                // Update UI if needed
                document.getElementById('vesselName').value = window.currentVesselModel.config.name;
                document.getElementById('vesselType').value = window.currentVesselModel.config.type;
            } else {
                showNotification('Nothing to redo', 'info');
            }
        });
        
        // Generate YAML
        document.getElementById('btn-generate-yaml')?.addEventListener('click', function() {
            // Create bootstrap modal and show it
            const yamlModal = new bootstrap.Modal(document.getElementById('yamlModal'));
            
            // Generate YAML previews before showing the modal
            generateYAMLPreviews();
            
            yamlModal.show();
        });
    }
    
    // Initialize viewport controls
    function initializeViewportControls(threeScene) {
        // Transform mode buttons
        document.getElementById('btn-translate')?.addEventListener('click', function() {
            threeScene.setTransformMode('translate');
            setActiveViewportButton(this);
        });
        
        document.getElementById('btn-rotate')?.addEventListener('click', function() {
            threeScene.setTransformMode('rotate');
            setActiveViewportButton(this);
        });
        
        document.getElementById('btn-scale')?.addEventListener('click', function() {
            threeScene.setTransformMode('scale');
            setActiveViewportButton(this);
        });
        
        // Toggle grid
        document.getElementById('btn-toggle-grid')?.addEventListener('click', function() {
            threeScene.toggleGrid();
            this.classList.toggle('active');
        });
        
        // Toggle wireframe
        document.getElementById('btn-toggle-wireframe')?.addEventListener('click', function() {
            threeScene.toggleWireframe();
            this.classList.toggle('active');
        });
        
        // Center view
        document.getElementById('btn-center-view')?.addEventListener('click', function() {
            threeScene.resetCamera();
        });
        
        // Set active button
        function setActiveViewportButton(button) {
            document.querySelectorAll('.viewport-btn').forEach(btn => {
                btn.classList.remove('active');
            });
            button.classList.add('active');
        }
        
        // Set translate as default
        document.getElementById('btn-translate')?.classList.add('active');
        
        // Add keyboard shortcuts
        window.addEventListener('keydown', function(event) {
            if (document.activeElement.tagName === 'INPUT' || 
                document.activeElement.tagName === 'TEXTAREA' || 
                document.activeElement.tagName === 'SELECT') {
                return; // Don't trigger shortcuts when editing form fields
            }
            
            switch(event.key.toLowerCase()) {
                case 'g':
                    threeScene.setTransformMode('translate');
                    setActiveViewportButton(document.getElementById('btn-translate'));
                    break;
                case 'r':
                    threeScene.setTransformMode('rotate');
                    setActiveViewportButton(document.getElementById('btn-rotate'));
                    break;
                case 's':
                    threeScene.setTransformMode('scale');
                    setActiveViewportButton(document.getElementById('btn-scale'));
                    break;
                case 'escape':
                    if (threeScene.selectedObject) {
                        threeScene.selectObject(null);
                    }
                    break;
            }
        });
    }
    
    // Initialize modals
    function initializeModals(threeScene) {
        // Control Surface Modal
        const controlSurfaceModal = document.getElementById('controlSurfaceModal');
        document.getElementById('btn-add-control-surface')?.addEventListener('click', function() {
            const modal = new bootstrap.Modal(controlSurfaceModal);
            modal.show();
        });
        
        document.getElementById('btnAddControlSurface')?.addEventListener('click', function() {
            // Get values from form
            const type = document.getElementById('controlSurfaceType').value;
            const surfaceId = parseInt(document.getElementById('controlSurfaceId').value);
            const position = [
                parseFloat(document.getElementById('csPositionX').value),
                parseFloat(document.getElementById('csPositionY').value),
                parseFloat(document.getElementById('csPositionZ').value)
            ];
            const orientation = [
                THREE.MathUtils.degToRad(parseFloat(document.getElementById('csOrientationX').value)),
                THREE.MathUtils.degToRad(parseFloat(document.getElementById('csOrientationY').value)),
                THREE.MathUtils.degToRad(parseFloat(document.getElementById('csOrientationZ').value))
            ];
            const area = parseFloat(document.getElementById('csArea').value);
            const naca = document.getElementById('csNACA').value;
            const timeConstant = parseFloat(document.getElementById('csTimeConstant').value);
            const deltaMax = parseFloat(document.getElementById('csDeltaMax').value);
            const deltadMax = parseFloat(document.getElementById('csDeltadMax').value);
            
            console.log('Adding control surface with params:', { type, surfaceId, position, orientation, area });
            
            // Debug before adding
            console.log('Before adding control surface:');
            logComponentCounts();
            
            // Add control surface
            const id = window.currentVesselModel.addControlSurface(
                type, position, orientation, area, surfaceId, naca, timeConstant, deltaMax, deltadMax
            );
            
            // Debug after adding
            console.log('After adding control surface:');
            logComponentCounts();
            
            // Add visual representation if threeScene has a method for it
            let controlSurfaceObj = null;
            if (typeof threeScene.addControlSurface === 'function') {
                controlSurfaceObj = threeScene.addControlSurface(id, type, position, orientation, area);
                
                // Add component axes visualization
                if (controlSurfaceObj) {
                    threeScene.addComponentAxes(controlSurfaceObj);
                }
            }
            
            // Close modal
            bootstrap.Modal.getInstance(controlSurfaceModal).hide();
            
            // Generate YAML previews to verify control surfaces are included
            generateYAMLPreviews();
            
            // Show notification
            showNotification(`Added ${type} control surface (ID: ${id})`, 'success');
        });
        
        // Thruster Modal
        const thrusterModal = document.getElementById('thrusterModal');
        document.getElementById('btn-add-thruster')?.addEventListener('click', function() {
            const modal = new bootstrap.Modal(thrusterModal);
            modal.show();
        });
        
        document.getElementById('btnAddThruster')?.addEventListener('click', function() {
            // Get values from form
            const name = document.getElementById('thrusterName').value;
            const thrusterId = parseInt(document.getElementById('thrusterId').value);
            const position = [
                parseFloat(document.getElementById('thrusterPositionX').value),
                parseFloat(document.getElementById('thrusterPositionY').value),
                parseFloat(document.getElementById('thrusterPositionZ').value)
            ];
            const orientation = [
                THREE.MathUtils.degToRad(parseFloat(document.getElementById('thrusterOrientationX').value)),
                THREE.MathUtils.degToRad(parseFloat(document.getElementById('thrusterOrientationY').value)),
                THREE.MathUtils.degToRad(parseFloat(document.getElementById('thrusterOrientationZ').value))
            ];
            const tProp = parseFloat(document.getElementById('thrusterTProp').value);
            const diameter = parseFloat(document.getElementById('thrusterDiameter').value);
            const tp = parseFloat(document.getElementById('thrusterTp').value);
            const jvsKTFile = document.getElementById('thrusterJvsKTFile').value;
            
            // Add thruster
            const id = window.currentVesselModel.addThruster(
                name, position, orientation, thrusterId, diameter, tProp, tp, jvsKTFile
            );
            
            // Add visual representation if threeScene has a method for it
            let thrusterObj = null;
            if (typeof threeScene.addThruster === 'function') {
                thrusterObj = threeScene.addThruster(id, name, position, orientation, diameter);
                
                // Add component axes visualization
                if (thrusterObj) {
                    threeScene.addComponentAxes(thrusterObj);
                }
            }
            
            // Close modal
            bootstrap.Modal.getInstance(thrusterModal).hide();
            
            // Show notification
            showNotification(`Added ${name} thruster (ID: ${id})`, 'success');
        });
        
        // Sensor Modal
        const sensorModal = document.getElementById('sensorModal');
        document.getElementById('btn-add-sensor')?.addEventListener('click', function() {
            const modal = new bootstrap.Modal(sensorModal);
            modal.show();
        });
        
        document.getElementById('btnAddSensor')?.addEventListener('click', function() {
            // Get values from form
            const type = document.getElementById('sensorType').value;
            const useNoneLocation = document.getElementById('sensorNoneLocation').checked;
            const useNoneOrientation = document.getElementById('sensorNoneOrientation').checked;
            
            let position = [0, 0, 0];
            if (!useNoneLocation) {
                position = [
                    parseFloat(document.getElementById('sensorPositionX').value),
                    parseFloat(document.getElementById('sensorPositionY').value),
                    parseFloat(document.getElementById('sensorPositionZ').value)
                ];
            }
            
            let orientation = [1.0, 0.0, 0.0, 0.0]; // Quaternion [w, x, y, z]
            if (!useNoneOrientation) {
                orientation = [
                    parseFloat(document.getElementById('sensorOrientationW').value),
                    parseFloat(document.getElementById('sensorOrientationX').value),
                    parseFloat(document.getElementById('sensorOrientationY').value),
                    parseFloat(document.getElementById('sensorOrientationZ').value)
                ];
            }
            
            const publishRate = parseFloat(document.getElementById('sensorPublishRate').value);
            
            // Add sensor
            const id = window.currentVesselModel.addSensor(
                type, position, orientation, publishRate, useNoneLocation, useNoneOrientation
            );
            
            // Add visual representation if threeScene has a method for it
            let sensorObj = null;
            if (typeof threeScene.addSensor === 'function') {
                sensorObj = threeScene.addSensor(id, type, useNoneLocation ? null : position, useNoneOrientation ? null : orientation);
                
                // Add component axes visualization if location is not None
                if (sensorObj && !useNoneLocation) {
                    threeScene.addComponentAxes(sensorObj);
                }
            }
            
            // Close modal
            bootstrap.Modal.getInstance(sensorModal).hide();
            
            // Show notification
            showNotification(`Added ${type} sensor (ID: ${id})`, 'success');
        });
        
        // FBX Component Modal
        const fbxComponentModal = document.getElementById('fbxComponentModal');
        document.getElementById('btnSaveFbxComponent')?.addEventListener('click', function() {
            // Implementation to be added when handling FBX component selection
            bootstrap.Modal.getInstance(fbxComponentModal).hide();
        });
        
        // FBX Component modal event handler
        document.getElementById('btnApplyFbxComponent')?.addEventListener('click', () => {
            const modal = document.getElementById('fbxComponentModal');
            const objectUuid = modal?.dataset.objectUuid;
            
            if (!objectUuid) {
                console.error('No object UUID found in modal');
                return;
            }
            
            const object = threeScene.getObjectByProperty('uuid', objectUuid);
            if (!object) {
                console.error('Object not found with UUID:', objectUuid);
                return;
            }
            
            const componentType = document.getElementById('fbxComponentType').value;
            let componentData = {
                type: componentType,
                uuid: objectUuid
            };
            
            // Get component center position in world coordinates
            const center = new THREE.Vector3();
            // Calculate the center of the object's bounding box
            if (object.geometry) {
                // For objects with geometry, calculate from geometry bounds
                object.geometry.computeBoundingBox();
                object.geometry.boundingBox.getCenter(center);
                object.localToWorld(center);
            } else {
                // For groups or objects without geometry, use position
                center.copy(object.position);
            }
            
            // Use center as component's position
            const position = [
                center.x,
                center.y,
                center.z
            ];

            // Get specific data based on component type
            if (componentType === 'controlSurface') {
                const surfaceType = document.getElementById('fbxSurfaceType').value;
                const surfaceId = parseInt(document.getElementById('fbxSurfaceId').value);
                const surfaceName = document.getElementById('fbxSurfaceName').value || `${surfaceType} Surface`;
                const area = parseFloat(document.getElementById('fbxSurfaceArea').value);
                const naca = document.getElementById('fbxSurfaceNACA').value;
                const timeConstant = parseFloat(document.getElementById('fbxSurfaceTimeConstant').value);
                const deltaMax = parseFloat(document.getElementById('fbxSurfaceDeltaMax').value);
                const deltadMax = parseFloat(document.getElementById('fbxSurfaceDeltadMax').value);
                
                // Get orientation in radians
                const orientation = [0, 0, 0]; // Default orientation, should be updated based on object
                if (object.rotation) {
                    orientation[0] = object.rotation.x;
                    orientation[1] = object.rotation.y;
                    orientation[2] = object.rotation.z;
                }
                
                // Add to vessel model
                const id = window.currentVesselModel.addControlSurface(
                    surfaceType, 
                    position, 
                    orientation, 
                    area, 
                    surfaceId, 
                    naca, 
                    timeConstant, 
                    deltaMax, 
                    deltadMax
                );
                
                componentData = {
                    ...componentData,
                    surfaceType,
                    id,
                    name: surfaceName,
                    area,
                    naca,
                    timeConstant,
                    deltaMax,
                    deltadMax,
                    position
                };
                
                object.name = surfaceName;
                
                // Add axes to the component
                threeScene.addComponentAxes(object);
                
            } else if (componentType === 'thruster') {
                const thrusterName = document.getElementById('fbxThrusterName').value || 'Thruster';
                const thrusterId = parseInt(document.getElementById('fbxThrusterId').value);
                const tProp = parseFloat(document.getElementById('fbxThrusterTProp').value);
                const diameter = parseFloat(document.getElementById('fbxThrusterDiameter').value);
                const tp = parseFloat(document.getElementById('fbxThrusterTp').value);
                const jvsKTFile = document.getElementById('fbxThrusterJvsKTFile').value;
                
                // Get orientation in radians
                const orientation = [0, 0, 0]; // Default orientation, should be updated based on object
                if (object.rotation) {
                    orientation[0] = object.rotation.x;
                    orientation[1] = object.rotation.y;
                    orientation[2] = object.rotation.z;
                }
                
                // Add to vessel model
                const id = window.currentVesselModel.addThruster(
                    thrusterName, 
                    position, 
                    orientation, 
                    thrusterId, 
                    diameter, 
                    tProp, 
                    tp, 
                    jvsKTFile
                );
                
                componentData = {
                    ...componentData,
                    id,
                    name: thrusterName,
                    tProp,
                    diameter,
                    tp,
                    jvsKTFile,
                    position
                };
                
                object.name = thrusterName;
                
                // Add axes to the component
                threeScene.addComponentAxes(object);
                
            } else if (componentType === 'sensor') {
                const sensorType = document.getElementById('fbxSensorType').value;
                const sensorName = document.getElementById('fbxSensorName').value || `${sensorType} Sensor`;
                const publishRate = parseFloat(document.getElementById('fbxSensorRate').value);
                const useNoneLocation = document.getElementById('fbxSensorNoneLocation').checked;
                const useNoneOrientation = document.getElementById('fbxSensorNoneOrientation').checked;
                
                let orientation = [1.0, 0.0, 0.0, 0.0]; // Quaternion [w, x, y, z]
                if (!useNoneOrientation && object.quaternion) {
                    orientation = [
                        object.quaternion.w,
                        object.quaternion.x,
                        object.quaternion.y,
                        object.quaternion.z
                    ];
                } else if (!useNoneOrientation) {
                    orientation = [
                        parseFloat(document.getElementById('fbxSensorOrientationW').value),
                        parseFloat(document.getElementById('fbxSensorOrientationX').value),
                        parseFloat(document.getElementById('fbxSensorOrientationY').value),
                        parseFloat(document.getElementById('fbxSensorOrientationZ').value)
                    ];
                }
                
                // Add to vessel model
                const id = window.currentVesselModel.addSensor(
                    sensorType,
                    useNoneLocation ? null : position,
                    orientation,
                    publishRate,
                    useNoneLocation,
                    useNoneOrientation
                );
                
                componentData = {
                    ...componentData,
                    sensorType,
                    id,
                    name: sensorName,
                    publishRate,
                    useNoneLocation,
                    useNoneOrientation,
                    position: useNoneLocation ? null : position
                };
                
                object.name = sensorName;
                
                // Add axes to the component if location is not None
                if (!useNoneLocation) {
                    threeScene.addComponentAxes(object);
                }
            }
            
            // Map the component in the vessel model
            if (window.currentVesselModel) {
                window.currentVesselModel.mapModelComponent(objectUuid, componentData);
                console.log(`Component ${componentData.name || object.name} configured as ${componentType}`);
                
                // CRITICAL FIX: Set component ID and type directly on the object's userData
                // This ensures the object itself always has the correct identification
                object.userData.componentId = componentData.id;
                object.userData.componentType = componentType;
                console.log(`Set component ID ${componentData.id} and type ${componentType} directly on object:`, object.name);
                
                // If there are existing axes, make sure they have the correct component ID and type
                const axes = object.children.find(child => child.userData.isComponentAxes);
                if (axes) {
                    axes.userData.componentId = componentData.id;
                    axes.userData.componentType = componentType;
                    console.log(`Updated existing axes with component ID ${componentData.id} and type ${componentType}`);
                } else {
                    // Add axes if they don't exist yet
                    const newAxes = threeScene.addComponentAxes(object);
                    console.log(`Added new axes to component:`, newAxes);
                }
                
                // Update the scene hierarchy
                threeScene.updateSceneHierarchy();
                
                // Re-select the object to update UI
                threeScene.selectObject(object);
                
                // Hide the modal
                const bsModal = bootstrap.Modal.getInstance(modal);
                bsModal.hide();
                
                // Show success notification
                showNotification(`Component ${componentData.name || object.name} configured successfully`, 'success');
            } else {
                console.error('No vessel model available');
                showNotification('Failed to configure component: No vessel model available', 'error');
            }
        });
        
        // Handle showing the modal and prepopulating fields
        document.getElementById('fbxComponentModal')?.addEventListener('show.bs.modal', (event) => {
            const modal = event.target;
            const objectUuid = modal.dataset.objectUuid;
            
            if (!objectUuid) return;
            
            const object = threeScene.getObjectByProperty('uuid', objectUuid);
            if (!object) return;
            
            const componentData = window.currentVesselModel?.getModelComponentData(objectUuid);
            const componentType = document.getElementById('fbxComponentType').value;
            
            // Hide all settings panels first
            document.getElementById('fbxControlSurfaceSettings').style.display = 'none';
            document.getElementById('fbxThrusterSettings').style.display = 'none';
            document.getElementById('fbxSensorSettings').style.display = 'none';
            
            // Show the appropriate panel and set component name in the appropriate field based on type
            if (componentType === 'controlSurface') {
                document.getElementById('fbxControlSurfaceSettings').style.display = 'block';
                document.getElementById('fbxSurfaceName').value = object.name;
                
                // Set default values or restore saved values
                if (componentData) {
                    document.getElementById('fbxSurfaceType').value = componentData.surfaceType || 'Rudder';
                    document.getElementById('fbxSurfaceId').value = componentData.id || 1;
                    document.getElementById('fbxSurfaceArea').value = componentData.area || 0.1;
                    document.getElementById('fbxSurfaceNACA').value = componentData.naca || '0015';
                    document.getElementById('fbxSurfaceTimeConstant').value = componentData.timeConstant || 0.1;
                    document.getElementById('fbxSurfaceDeltaMax').value = componentData.deltaMax || 35.0;
                    document.getElementById('fbxSurfaceDeltadMax').value = componentData.deltadMax || 1.0;
                }
            } else if (componentType === 'thruster') {
                document.getElementById('fbxThrusterSettings').style.display = 'block';
                document.getElementById('fbxThrusterName').value = object.name;
                
                if (componentData) {
                    document.getElementById('fbxThrusterId').value = componentData.id || 1;
                    document.getElementById('fbxThrusterTProp').value = componentData.tProp || 1.0;
                    document.getElementById('fbxThrusterDiameter').value = componentData.diameter || 0.1;
                    document.getElementById('fbxThrusterTp').value = componentData.tp || 0.1;
                    document.getElementById('fbxThrusterJvsKTFile').value = componentData.jvsKTFile || 'J_vs_KT.csv';
                }
            } else if (componentType === 'sensor') {
                document.getElementById('fbxSensorSettings').style.display = 'block';
                document.getElementById('fbxSensorName').value = object.name;
                
                if (componentData) {
                    document.getElementById('fbxSensorType').value = componentData.sensorType || 'IMU';
                    document.getElementById('fbxSensorRate').value = componentData.publishRate || 10;
                    
                    // Handle None values for location and orientation
                    const useNoneLocation = componentData.useNoneLocation || false;
                    const useNoneOrientation = componentData.useNoneOrientation || false;
                    
                    document.getElementById('fbxSensorNoneLocation').checked = useNoneLocation;
                    document.getElementById('fbxSensorNoneOrientation').checked = useNoneOrientation;
                    
                    // Update UI visibility based on None settings
                    document.getElementById('fbxSensorOrientationGroup').style.display = 
                        useNoneOrientation ? 'none' : 'block';
                    
                    // Set orientation values if available
                    if (componentData.orientation && !useNoneOrientation) {
                        document.getElementById('fbxSensorOrientationW').value = componentData.orientation[0] || 1.0;
                        document.getElementById('fbxSensorOrientationX').value = componentData.orientation[1] || 0.0;
                        document.getElementById('fbxSensorOrientationY').value = componentData.orientation[2] || 0.0;
                        document.getElementById('fbxSensorOrientationZ').value = componentData.orientation[3] || 0.0;
                    }
                }
                
                // Trigger change event to handle visibility of location/orientation fields
                const event = new Event('change');
                document.getElementById('fbxSensorType').dispatchEvent(event);
            }
        });
        
        // Add handler for Edit Hydrodynamics button
        document.getElementById('btnEditHydrodynamics')?.addEventListener('click', function() {
            // Get current hydrodynamics values from vessel model
            const hydrodynamics = window.currentVesselModel.config.hydrodynamics;
            console.log("Opening hydrodynamics modal with current data:", hydrodynamics);
            
            // Populate the checkboxes
            document.getElementById('dimFlag').checked = hydrodynamics.dim_flag || false;
            document.getElementById('crossFlowDrag').checked = hydrodynamics.cross_flow_drag || false;
            
            // Generate text content for the textarea based on current hydrodynamics
            const textContent = Object.entries(hydrodynamics)
                .filter(([key]) => key !== 'dim_flag' && key !== 'cross_flow_drag' && key !== 'hydra_file')
                .map(([key, value]) => `${key}: ${value}`)
                .join('\n');
            
            document.getElementById('hydroCoefficients').value = textContent;
            
            // Show the modal
            const hydrodynamicsModal = new bootstrap.Modal(document.getElementById('hydrodynamicsModal'));
            hydrodynamicsModal.show();
        });
        
        // Add handler for file upload
        document.getElementById('hydroFileUpload')?.addEventListener('change', function(event) {
            const file = event.target.files[0];
            if (file) {
                const reader = new FileReader();
                reader.onload = function(e) {
                    document.getElementById('hydroCoefficients').value = e.target.result;
                };
                reader.readAsText(file);
            }
        });
        
        // Add handler for Save Hydrodynamics button
        document.getElementById('btnSaveHydrodynamics')?.addEventListener('click', function() {
            // Get values from the form
            const coefficientText = document.getElementById('hydroCoefficients').value;
            
            // Parse the text into key-value pairs
            const updatedHydrodynamics = {
                dim_flag: document.getElementById('dimFlag').checked,
                cross_flow_drag: document.getElementById('crossFlowDrag').checked,
                // Keep the existing hydra_file value
                hydra_file: window.currentVesselModel.config.hydrodynamics.hydra_file || ''
            };
            
            // Track parsing issues for user feedback
            const parseIssues = [];
            
            // Process each line of the text area
            const lines = coefficientText.split('\n');
            for (let i = 0; i < lines.length; i++) {
                const line = lines[i].trim();
                
                // Skip empty lines and comment lines
                if (!line || line.startsWith('#') || line.startsWith('//')) continue;
                
                // Parse key-value pairs (allow for different separators)
                const match = line.match(/([^:=,\s]+)\s*[:=,]\s*(-?\d+\.?\d*)/);
                if (match) {
                    const [, key, valueStr] = match;
                    const cleanKey = key.trim();
                    const numValue = parseFloat(valueStr);
                    
                    if (isNaN(numValue)) {
                        parseIssues.push(`Line ${i+1}: "${valueStr}" is not a valid number`);
                    } else {
                        updatedHydrodynamics[cleanKey] = numValue;
                    }
                } else if (line.includes(':') || line.includes('=')) {
                    // Line has a separator but didn't parse correctly
                    parseIssues.push(`Line ${i+1}: Format error in "${line}"`);
                }
            }
            
            console.log("Parsed hydrodynamics to save:", updatedHydrodynamics);
            
            // Show warning if there were parse issues
            if (parseIssues.length > 0) {
                const message = `Warning: Some lines couldn't be parsed:\n${parseIssues.slice(0, 3).join('\n')}${parseIssues.length > 3 ? `\n...and ${parseIssues.length - 3} more issues` : ''}`;
                showNotification(message, 'warning');
            }
            
            // Update the vessel model
            window.currentVesselModel.updateHydrodynamics(updatedHydrodynamics);
            
            // Verify the update worked
            console.log("Vessel hydrodynamics after update:", window.currentVesselModel.config.hydrodynamics);
            
            // Generate the hydrodynamics YAML directly and update the preview
            const yamlGenerator = new YAMLGenerator();
            const hydroYaml = yamlGenerator.generateHydrodynamicsYAML(
                window.currentVesselModel.config.hydrodynamics, 
                window.currentVesselModel.config.name
            );
            document.getElementById('hydrodynamics-yaml-content').textContent = hydroYaml;
            
            // Also update all other YAML previews
            generateYAMLPreviews();
            
            // Close the modal
            const hydrodynamicsModal = bootstrap.Modal.getInstance(document.getElementById('hydrodynamicsModal'));
            hydrodynamicsModal.hide();
            
            // Show success notification
            const coeffCount = Object.keys(updatedHydrodynamics).length - 3; // Subtract dim_flag, cross_flow_drag, hydra_file
            showNotification(`Hydrodynamics updated with ${coeffCount} coefficients`, 'success');
        });
        
        // Add handler for Edit Initial Conditions button
        document.getElementById('btnEditInitialConditions')?.addEventListener('click', function() {
            try {
                // Get current initial conditions from the vessel model
                const initialConditions = window.currentVesselModel.config.initial_conditions || {
                    start_location: [0, 0, 0],
                    start_orientation: [0, 0, 0],
                    start_velocity: [0, 0, 0, 0, 0, 0],
                    use_quaternion: false
                };
                
                // Populate the form fields with current values
                const startLocation = initialConditions.start_location || [0, 0, 0];
                document.getElementById('startLocationX').value = startLocation[0];
                document.getElementById('startLocationY').value = startLocation[1];
                document.getElementById('startLocationZ').value = startLocation[2];
                
                const startOrientation = initialConditions.start_orientation || [0, 0, 0];
                document.getElementById('startOrientationRoll').value = startOrientation[0];
                document.getElementById('startOrientationPitch').value = startOrientation[1];
                document.getElementById('startOrientationYaw').value = startOrientation[2];
                
                const startVelocity = initialConditions.start_velocity || [0, 0, 0, 0, 0, 0];
                document.getElementById('startVelocityU').value = startVelocity[0];
                document.getElementById('startVelocityV').value = startVelocity[1];
                document.getElementById('startVelocityW').value = startVelocity[2];
                document.getElementById('startVelocityP').value = startVelocity[3];
                document.getElementById('startVelocityQ').value = startVelocity[4];
                document.getElementById('startVelocityR').value = startVelocity[5];
                
                document.getElementById('useQuaternion').checked = initialConditions.use_quaternion || false;
                
                console.log("Opening initial conditions modal with:", initialConditions);
                
                // Show the modal
                const initialConditionsModal = new bootstrap.Modal(document.getElementById('initialConditionsModal'));
                initialConditionsModal.show();
            } catch (error) {
                console.error("Error opening initial conditions modal:", error);
                showNotification("Error loading initial conditions", "error");
            }
        });
        
        // Add handler for initial conditions file upload
        document.getElementById('initialConditionsFileUpload')?.addEventListener('change', function(event) {
            const file = event.target.files[0];
            if (file) {
                const reader = new FileReader();
                reader.onload = function(e) {
                    try {
                        // Parse the YAML and populate form fields
                        const initialConditions = jsyaml.load(e.target.result);
                        
                        if (initialConditions && initialConditions.start_location) {
                            document.getElementById('startLocationX').value = initialConditions.start_location[0];
                            document.getElementById('startLocationY').value = initialConditions.start_location[1];
                            document.getElementById('startLocationZ').value = initialConditions.start_location[2];
                        }
                        
                        if (initialConditions && initialConditions.start_orientation) {
                            document.getElementById('startOrientationRoll').value = initialConditions.start_orientation[0];
                            document.getElementById('startOrientationPitch').value = initialConditions.start_orientation[1];
                            document.getElementById('startOrientationYaw').value = initialConditions.start_orientation[2];
                        }
                        
                        if (initialConditions && initialConditions.start_velocity) {
                            document.getElementById('startVelocityU').value = initialConditions.start_velocity[0];
                            document.getElementById('startVelocityV').value = initialConditions.start_velocity[1];
                            document.getElementById('startVelocityW').value = initialConditions.start_velocity[2];
                            document.getElementById('startVelocityP').value = initialConditions.start_velocity[3];
                            document.getElementById('startVelocityQ').value = initialConditions.start_velocity[4];
                            document.getElementById('startVelocityR').value = initialConditions.start_velocity[5];
                        }
                        
                        if (initialConditions && initialConditions.hasOwnProperty('use_quaternion')) {
                            document.getElementById('useQuaternion').checked = initialConditions.use_quaternion;
                        }
                        
                        showNotification("Initial conditions file loaded successfully", "success");
                    } catch (error) {
                        console.error("Error parsing initial conditions file:", error);
                        showNotification("Error parsing file: " + error.message, "error");
                    }
                };
                reader.readAsText(file);
            }
        });
        
        // Add handler for Save Initial Conditions button
        document.getElementById('btnSaveInitialConditions')?.addEventListener('click', function() {
            try {
                // Get values from the form fields
                const startLocation = [
                    parseFloat(document.getElementById('startLocationX').value),
                    parseFloat(document.getElementById('startLocationY').value),
                    parseFloat(document.getElementById('startLocationZ').value)
                ];
                
                const startOrientation = [
                    parseFloat(document.getElementById('startOrientationRoll').value),
                    parseFloat(document.getElementById('startOrientationPitch').value),
                    parseFloat(document.getElementById('startOrientationYaw').value)
                ];
                
                const startVelocity = [
                    parseFloat(document.getElementById('startVelocityU').value),
                    parseFloat(document.getElementById('startVelocityV').value),
                    parseFloat(document.getElementById('startVelocityW').value),
                    parseFloat(document.getElementById('startVelocityP').value),
                    parseFloat(document.getElementById('startVelocityQ').value),
                    parseFloat(document.getElementById('startVelocityR').value)
                ];
                
                const useQuaternion = document.getElementById('useQuaternion').checked;
                
                // Check for invalid values
                const hasInvalidValues = [...startLocation, ...startOrientation, ...startVelocity].some(val => isNaN(val));
                if (hasInvalidValues) {
                    throw new Error("Some values are not valid numbers");
                }
                
                // Update the vessel model with the new initial conditions
                window.currentVesselModel.updateInitialConditions(
                    startVelocity,
                    startLocation,
                    startOrientation,
                    useQuaternion
                );
                
                // Generate the initial_conditions.yml directly and update the preview
                const yamlGenerator = new YAMLGenerator();
                const initialConditionsYaml = yamlGenerator.toYAML(
                    yamlGenerator.generateInitialConditionsYAML(window.currentVesselModel.config.initial_conditions)
                );
                document.getElementById('initial-conditions-yaml-content').textContent = initialConditionsYaml;
                
                // Also update all other YAML previews
                generateYAMLPreviews();
                
                // Close the modal
                const initialConditionsModal = bootstrap.Modal.getInstance(document.getElementById('initialConditionsModal'));
                initialConditionsModal.hide();
                
                // Show success notification
                showNotification("Initial conditions updated successfully", "success");
            } catch (error) {
                console.error("Error saving initial conditions:", error);
                showNotification("Error: " + error.message, "error");
            }
        });
        
        // Add handler for Edit Inertia Parameters button
        document.getElementById('btnEditInertia')?.addEventListener('click', function() {
            try {
                // Get current inertia parameters from the vessel model
                const inertia = window.currentVesselModel.config.inertia || {
                    mass: 1,
                    buoyancy_mass: 1,
                    inertia_matrix: null,
                    added_mass_matrix: null
                };
                
                // Populate form fields with current values
                document.getElementById('vesselMass').value = inertia.mass || 1;
                document.getElementById('vesselBuoyancyMass').value = inertia.buoyancy_mass || 1;
                
                // Handle inertia matrix
                const useDefaultInertia = !inertia.inertia_matrix;
                document.getElementById('useDefaultInertia').checked = useDefaultInertia;
                document.getElementById('inertiaMatrixSection').style.display = useDefaultInertia ? 'none' : 'block';
                
                // Set inertia matrix values if available
                if (!useDefaultInertia && inertia.inertia_matrix) {
                    document.getElementById('inertia11').value = inertia.inertia_matrix[0][0] || 5;
                    document.getElementById('inertia12').value = inertia.inertia_matrix[0][1] || 0;
                    document.getElementById('inertia13').value = inertia.inertia_matrix[0][2] || 0;
                    document.getElementById('inertia21').value = inertia.inertia_matrix[1][0] || 0;
                    document.getElementById('inertia22').value = inertia.inertia_matrix[1][1] || 10;
                    document.getElementById('inertia23').value = inertia.inertia_matrix[1][2] || 0;
                    document.getElementById('inertia31').value = inertia.inertia_matrix[2][0] || 0;
                    document.getElementById('inertia32').value = inertia.inertia_matrix[2][1] || 0;
                    document.getElementById('inertia33').value = inertia.inertia_matrix[2][2] || 10;
                }
                
                // Handle added mass matrix
                const useDefaultAddedMass = !inertia.added_mass_matrix;
                document.getElementById('useDefaultAddedMass').checked = useDefaultAddedMass;
                document.getElementById('addedMassSection').style.display = useDefaultAddedMass ? 'none' : 'block';
                
                if (!useDefaultAddedMass && inertia.added_mass_matrix) {
                    document.getElementById('addedMass11').value = inertia.added_mass_matrix[0][0] || 0;
                    document.getElementById('addedMass12').value = inertia.added_mass_matrix[0][1] || 0;
                    document.getElementById('addedMass13').value = inertia.added_mass_matrix[0][2] || 0;
                    document.getElementById('addedMass21').value = inertia.added_mass_matrix[1][0] || 0;
                    document.getElementById('addedMass22').value = inertia.added_mass_matrix[1][1] || 0;
                    document.getElementById('addedMass23').value = inertia.added_mass_matrix[1][2] || 0;
                    document.getElementById('addedMass31').value = inertia.added_mass_matrix[2][0] || 0;
                    document.getElementById('addedMass32').value = inertia.added_mass_matrix[2][1] || 0;
                    document.getElementById('addedMass33').value = inertia.added_mass_matrix[2][2] || 0;
                }
                
                console.log("Opening inertia modal with:", inertia);
                
                // Show the modal
                const inertiaModal = new bootstrap.Modal(document.getElementById('inertiaModal'));
                inertiaModal.show();
            } catch (error) {
                console.error("Error opening inertia modal:", error);
                showNotification("Error loading inertia parameters", "error");
            }
        });
        
        // Add handler to toggle inertia matrix section visibility
        document.getElementById('useDefaultInertia')?.addEventListener('change', function() {
            document.getElementById('inertiaMatrixSection').style.display = this.checked ? 'none' : 'block';
        });
        
        // Add handler to toggle added mass section visibility
        document.getElementById('useDefaultAddedMass')?.addEventListener('change', function() {
            document.getElementById('addedMassSection').style.display = this.checked ? 'none' : 'block';
        });
        
        // Add handler for inertia file upload
        document.getElementById('inertiaFileUpload')?.addEventListener('change', function(event) {
            const file = event.target.files[0];
            if (file) {
                const reader = new FileReader();
                reader.onload = function(e) {
                    try {
                        // Parse the YAML and populate form fields
                        const inertiaParams = jsyaml.load(e.target.result);
                        
                        if (inertiaParams) {
                            if (inertiaParams.mass) {
                                document.getElementById('vesselMass').value = inertiaParams.mass;
                            }
                            
                            if (inertiaParams.buoyancy_mass) {
                                document.getElementById('vesselBuoyancyMass').value = inertiaParams.buoyancy_mass;
                            }
                            
                            // Handle inertia matrix
                            const hasInertiaMatrix = inertiaParams.inertia_matrix && inertiaParams.inertia_matrix !== 'None';
                            document.getElementById('useDefaultInertia').checked = !hasInertiaMatrix;
                            document.getElementById('inertiaMatrixSection').style.display = hasInertiaMatrix ? 'block' : 'none';
                            
                            // Set inertia matrix values if available
                            if (hasInertiaMatrix) {
                                document.getElementById('inertia11').value = inertiaParams.inertia_matrix[0][0] || 5;
                                document.getElementById('inertia12').value = inertiaParams.inertia_matrix[0][1] || 0;
                                document.getElementById('inertia13').value = inertiaParams.inertia_matrix[0][2] || 0;
                                document.getElementById('inertia21').value = inertiaParams.inertia_matrix[1][0] || 0;
                                document.getElementById('inertia22').value = inertiaParams.inertia_matrix[1][1] || 10;
                                document.getElementById('inertia23').value = inertiaParams.inertia_matrix[1][2] || 0;
                                document.getElementById('inertia31').value = inertiaParams.inertia_matrix[2][0] || 0;
                                document.getElementById('inertia32').value = inertiaParams.inertia_matrix[2][1] || 0;
                                document.getElementById('inertia33').value = inertiaParams.inertia_matrix[2][2] || 10;
                            }
                            
                            // Handle added mass matrix
                            const hasAddedMassMatrix = inertiaParams.added_mass_matrix && inertiaParams.added_mass_matrix !== 'None';
                            document.getElementById('useDefaultAddedMass').checked = !hasAddedMassMatrix;
                            document.getElementById('addedMassSection').style.display = hasAddedMassMatrix ? 'block' : 'none';
                            
                            if (hasAddedMassMatrix) {
                                document.getElementById('addedMass11').value = inertiaParams.added_mass_matrix[0][0] || 0;
                                document.getElementById('addedMass12').value = inertiaParams.added_mass_matrix[0][1] || 0;
                                document.getElementById('addedMass13').value = inertiaParams.added_mass_matrix[0][2] || 0;
                                document.getElementById('addedMass21').value = inertiaParams.added_mass_matrix[1][0] || 0;
                                document.getElementById('addedMass22').value = inertiaParams.added_mass_matrix[1][1] || 0;
                                document.getElementById('addedMass23').value = inertiaParams.added_mass_matrix[1][2] || 0;
                                document.getElementById('addedMass31').value = inertiaParams.added_mass_matrix[2][0] || 0;
                                document.getElementById('addedMass32').value = inertiaParams.added_mass_matrix[2][1] || 0;
                                document.getElementById('addedMass33').value = inertiaParams.added_mass_matrix[2][2] || 0;
                            }
                        }
                        
                        showNotification("Inertia parameters file loaded successfully", "success");
                    } catch (error) {
                        console.error("Error parsing inertia file:", error);
                        showNotification("Error parsing file: " + error.message, "error");
                    }
                };
                reader.readAsText(file);
            }
        });
        
        // Add handler for Save Inertia Parameters button
        document.getElementById('btnSaveInertia')?.addEventListener('click', function() {
            try {
                // Get values from the form fields
                const mass = parseFloat(document.getElementById('vesselMass').value);
                const buoyancyMass = parseFloat(document.getElementById('vesselBuoyancyMass').value);
                
                // Check for invalid values
                if (isNaN(mass) || isNaN(buoyancyMass) || mass <= 0 || buoyancyMass <= 0) {
                    throw new Error("Mass and buoyancy mass must be positive numbers");
                }
                
                // Handle inertia matrix
                const useDefaultInertia = document.getElementById('useDefaultInertia').checked;
                let inertiaMatrix = null;
                
                if (!useDefaultInertia) {
                    // Build inertia matrix from form inputs
                    inertiaMatrix = [
                        [
                            parseFloat(document.getElementById('inertia11').value),
                            parseFloat(document.getElementById('inertia12').value),
                            parseFloat(document.getElementById('inertia13').value)
                        ],
                        [
                            parseFloat(document.getElementById('inertia21').value),
                            parseFloat(document.getElementById('inertia22').value),
                            parseFloat(document.getElementById('inertia23').value)
                        ],
                        [
                            parseFloat(document.getElementById('inertia31').value),
                            parseFloat(document.getElementById('inertia32').value),
                            parseFloat(document.getElementById('inertia33').value)
                        ]
                    ];
                    
                    // Check for invalid inertia matrix values
                    const hasInvalidInertia = inertiaMatrix.flat().some(val => isNaN(val));
                    if (hasInvalidInertia) {
                        throw new Error("Inertia matrix contains invalid values");
                    }
                }
                
                // Handle added mass matrix
                const useDefaultAddedMass = document.getElementById('useDefaultAddedMass').checked;
                let addedMassMatrix = null;
                
                if (!useDefaultAddedMass) {
                    // Build added mass matrix from form inputs
                    addedMassMatrix = [
                        [
                            parseFloat(document.getElementById('addedMass11').value),
                            parseFloat(document.getElementById('addedMass12').value),
                            parseFloat(document.getElementById('addedMass13').value)
                        ],
                        [
                            parseFloat(document.getElementById('addedMass21').value),
                            parseFloat(document.getElementById('addedMass22').value),
                            parseFloat(document.getElementById('addedMass23').value)
                        ],
                        [
                            parseFloat(document.getElementById('addedMass31').value),
                            parseFloat(document.getElementById('addedMass32').value),
                            parseFloat(document.getElementById('addedMass33').value)
                        ]
                    ];
                    
                    // Check for invalid added mass matrix values
                    const hasInvalidAddedMass = addedMassMatrix.flat().some(val => isNaN(val));
                    if (hasInvalidAddedMass) {
                        throw new Error("Added mass matrix contains invalid values");
                    }
                }
                
                // Update the vessel model with the new inertia parameters
                window.currentVesselModel.updatePhysicalProperties(
                    mass,
                    buoyancyMass,
                    window.currentVesselModel.config.inertia.CG || [0, 0, 0],
                    window.currentVesselModel.config.inertia.CB || [0, 0, 0]
                );
                
                // Also directly update the inertia matrices
                window.currentVesselModel.config.inertia.inertia_matrix = inertiaMatrix;
                window.currentVesselModel.config.inertia.added_mass_matrix = addedMassMatrix;
                
                // Generate the inertia.yml directly and update the preview
                const yamlGenerator = new YAMLGenerator();
                const inertiaYaml = yamlGenerator.toYAML(
                    yamlGenerator.generateInertiaYAML(window.currentVesselModel.config.inertia)
                );
                document.getElementById('inertia-yaml-content').textContent = inertiaYaml;
                
                // Also update all other YAML previews
                generateYAMLPreviews();
                
                // Close the modal
                const inertiaModal = bootstrap.Modal.getInstance(document.getElementById('inertiaModal'));
                inertiaModal.hide();
                
                // Show success notification
                showNotification("Inertia parameters updated successfully", "success");
            } catch (error) {
                console.error("Error saving inertia parameters:", error);
                showNotification("Error: " + error.message, "error");
            }
        });
    }
    
    // Initialize file handlers
    function initializeFileHandlers(threeScene) {
        // Basic vessel properties form
        const vesselNameInput = document.getElementById('vesselName');
        const vesselTypeSelect = document.getElementById('vesselType');
        
        if (vesselNameInput) {
            vesselNameInput.addEventListener('change', function() {
                window.currentVesselModel.setBasicInfo(this.value, vesselTypeSelect.value);
            });
        }
        
        if (vesselTypeSelect) {
            vesselTypeSelect.addEventListener('change', function() {
                window.currentVesselModel.setBasicInfo(vesselNameInput.value, this.value);
            });
        }
        
        // FBX file upload - with performance optimization (debounce loading events)
        const fbxFileInput = document.getElementById('fbxFile');
        if (fbxFileInput) {
            fbxFileInput.addEventListener('change', function() {
                if (this.files && this.files[0]) {
                    const file = this.files[0];
                    
                    // Show loading indicator
                    showNotification('Loading 3D model...', 'info');
                    
                    // Use setTimeout to allow UI to update before starting heavy operation
                    setTimeout(async () => {
                        try {
                            // Load the model if threeScene has the method
                            if (typeof threeScene.loadFBXModel === 'function') {
                                const model = await threeScene.loadFBXModel(file);
                                
                                // Store file reference
                                window.currentVesselModel.setModelFile('fbx', file);
                                
                                // Update scene hierarchy
                                threeScene.updateSceneHierarchy();
                                
                                // Show success message
                                showNotification('3D model loaded successfully', 'success');
                            } else {
                                console.error('threeScene.loadFBXModel is not a function');
                                showNotification('FBX loading not supported in this version', 'error');
                            }
                        } catch (error) {
                            console.error('Error loading 3D model:', error);
                            showNotification('Failed to load 3D model: ' + error.message, 'error');
                        }
                    }, 100);
                }
            });
        }
        
        // GDF file upload
        const gdfFileInput = document.getElementById('gdfFile');
        if (gdfFileInput) {
            gdfFileInput.addEventListener('change', function() {
                if (this.files && this.files[0]) {
                    const file = this.files[0];
                    
                    // Store file reference
                    window.currentVesselModel.setModelFile('gdf', file);
                    
                    // Update geometry file path in config
                    window.currentVesselModel.config.geometry.geometry_file = file.name;
                    
                    showNotification('GDF file loaded', 'success');
                }
            });
        }
        
        // Hydra output zip upload
        const hydraZipInput = document.getElementById('hydraZip');
        if (hydraZipInput) {
            hydraZipInput.addEventListener('change', function() {
                if (this.files && this.files[0]) {
                    const file = this.files[0];
                    
                    // Store file reference
                    window.currentVesselModel.setModelFile('hydra', file);
                    
                    // Update Hydra file path in config
                    const vesselName = window.currentVesselModel.config.name || 'vessel';
                    window.currentVesselModel.config.hydrodynamics.hydra_file = `/workspaces/mavlab/inputs/${vesselName}/HydRA/${vesselName.toUpperCase()}_hydra.json`;
                    
                    showNotification('Hydra output loaded', 'success');
                }
            });
        }
        
        // Simulation parameters - with debounced updates to reduce lag
        let simParamsDebounce;
        function updateSimParams() {
            clearTimeout(simParamsDebounce);
            simParamsDebounce = setTimeout(() => {
                const simParams = {
                    sim_time: parseFloat(document.getElementById('simTime')?.value || 100),
                    time_step: parseFloat(document.getElementById('timeStep')?.value || 0.01),
                    density: parseFloat(document.getElementById('density')?.value || 1025),
                    gravity: parseFloat(document.getElementById('gravity')?.value || 9.80665),
                    world_size: [
                        parseFloat(document.getElementById('worldSizeX')?.value || 1000),
                        parseFloat(document.getElementById('worldSizeY')?.value || 1000),
                        parseFloat(document.getElementById('worldSizeZ')?.value || 100)
                    ],
                    gps_datum: [
                        parseFloat(document.getElementById('gpsDatumLat')?.value || 12.99300),
                        parseFloat(document.getElementById('gpsDatumLon')?.value || 80.23913),
                        parseFloat(document.getElementById('gpsDatumAlt')?.value || -87.0)
                    ]
                };
                
                window.currentVesselModel.updateSimulationParams(simParams);
            }, 300); // 300ms debounce
        }
        
        document.querySelectorAll('#simTime, #timeStep, #density, #gravity, #worldSizeX, #worldSizeY, #worldSizeZ, #gpsDatumLat, #gpsDatumLon, #gpsDatumAlt').forEach(input => {
            input?.addEventListener('change', updateSimParams);
            input?.addEventListener('input', updateSimParams);
        });
    }
    
    // Initialize YAML generation
    function initializeYAMLGeneration() {
        document.getElementById('btnGenerateYaml')?.addEventListener('click', async function() {
            const vesselName = window.currentVesselModel.config.name;
            const outputPath = document.getElementById('outputPath').value;
            
            if (!vesselName) {
                window.currentVesselModel.config.name = 'vessel';
                document.getElementById('vesselName').value = 'vessel';
                showNotification('Using default vessel name "vessel"', 'warning');
            }
            
            try {
                // Generate YAML files
                const yamlFiles = window.currentVesselModel.generateYAMLFiles();
                
                if (!yamlFiles || Object.keys(yamlFiles).length === 0) {
                    throw new Error('No YAML files were generated');
                }
                
                // Create a notification with the result
                let message = `YAML files generated for vessel: ${vesselName}\n`;
                message += `Output path: ${outputPath}${vesselName}/`;
                
                // Show files that were generated
                message += `\nGenerated files: ${Object.keys(yamlFiles).join(', ')}`;
                
                showNotification(message, 'success');
                
                // Hide the modal
                bootstrap.Modal.getInstance(document.getElementById('yamlModal')).hide();
            } catch (error) {
                console.error('Error generating YAML files:', error);
                showNotification('Failed to generate YAML files: ' + error.message, 'error');
            }
        });
        
        document.getElementById('btnDownloadZip')?.addEventListener('click', async function() {
            const vesselName = window.currentVesselModel.config.name;
            
            if (!vesselName) {
                showNotification('Please enter a vessel name before downloading YAML files', 'error');
                return;
            }
            
            showNotification('Preparing ZIP file...', 'info');
            
            try {
                // Create a zip file with all YAML files - use setTimeout to allow UI to update
                setTimeout(async () => {
                    try {
                        const zipBlob = await window.currentVesselModel.createYAMLZip();
                        
                        // Create download link
                        const url = URL.createObjectURL(zipBlob);
                        const a = document.createElement('a');
                        a.href = url;
                        a.download = `${vesselName}_config.zip`;
                        a.click();
                        
                        URL.revokeObjectURL(url);
                        
                        // Show notification
                        showNotification(`YAML files downloaded as ${vesselName}_config.zip`, 'success');
                        
                        // Hide the modal
                        bootstrap.Modal.getInstance(document.getElementById('yamlModal')).hide();
                    } catch (error) {
                        console.error('Error creating ZIP file:', error);
                        showNotification('Failed to create ZIP file: ' + error.message, 'error');
                    }
                }, 100);
            } catch (error) {
                console.error('Error preparing ZIP file:', error);
                showNotification('Failed to prepare ZIP file: ' + error.message, 'error');
            }
        });
    }
    
    // Generate YAML previews in the modal
    function generateYAMLPreviews() {
        try {
            // Ensure vessel has a name
            if (!window.currentVesselModel.config.name) {
                window.currentVesselModel.config.name = 'vessel';
                document.getElementById('vesselName').value = 'vessel';
                showNotification('Using default vessel name "vessel"', 'warning');
            }
            
            // First, let's log the component counts to verify data
            logComponentCounts();
            
            // Generate YAML files
            const yamlFiles = window.currentVesselModel.generateYAMLFiles();
            
            // Check if YAML files were generated
            console.log('Generated YAML files:', Object.keys(yamlFiles));
            
            // Display previews for each YAML file
            document.getElementById('simulation-yaml-content').textContent = yamlFiles['simulation_input.yml'] || '';
            document.getElementById('geometry-yaml-content').textContent = yamlFiles['geometry.yml'] || '';
            document.getElementById('inertia-yaml-content').textContent = yamlFiles['inertia.yml'] || '';
            document.getElementById('hydrodynamics-yaml-content').textContent = yamlFiles['hydrodynamics.yml'] || '';
            
            // For control surfaces, add some extra debug information
            const controlSurfacesYaml = yamlFiles['control_surfaces.yml'] || '';
            document.getElementById('control-surfaces-yaml-content').textContent = 
                `# Number of control surfaces: ${window.currentVesselModel.config.control_surfaces?.control_surfaces?.length || 0}\n` +
                controlSurfacesYaml;
                
            document.getElementById('thrusters-yaml-content').textContent = yamlFiles['thrusters.yml'] || '';
            document.getElementById('sensors-yaml-content').textContent = yamlFiles['sensors.yml'] || '';
            document.getElementById('initial-conditions-yaml-content').textContent = yamlFiles['initial_conditions.yml'] || '';
            document.getElementById('guidance-yaml-content').textContent = yamlFiles['guidance.yml'] || '';
            document.getElementById('control-yaml-content').textContent = yamlFiles['control.yml'] || '';
            
            // Check if specific files are missing
            if (!yamlFiles['control_surfaces.yml'] && window.currentVesselModel.config.control_surfaces.control_surfaces.length > 0) {
                console.warn('Control surfaces YAML file is missing despite having control surfaces data');
                showNotification('Error generating control surfaces YAML', 'warning');
            }
            
            if (!yamlFiles['thrusters.yml'] && window.currentVesselModel.config.thrusters.thrusters.length > 0) {
                console.warn('Thrusters YAML file is missing despite having thrusters data');
                showNotification('Error generating thrusters YAML', 'warning');
            }
            
            if (!yamlFiles['sensors.yml'] && window.currentVesselModel.config.sensors.sensors.length > 0) {
                console.warn('Sensors YAML file is missing despite having sensors data');
                showNotification('Error generating sensors YAML', 'warning');
            }
            
        } catch (error) {
            console.error('Error generating YAML previews:', error);
            showNotification('Failed to generate YAML previews: ' + error.message, 'error');
        }
    }
    
    // --------- Utility Functions ---------
    
    // Show notification to the user
    function showNotification(message, type = 'info') {
        const colors = {
            success: '#28a745',
            error: '#dc3545',
            warning: '#ffc107',
            info: '#17a2b8'
        };
        
        // Create notification element
        const notification = document.createElement('div');
        notification.className = 'notification';
        notification.style.position = 'fixed';
        notification.style.top = '20px';
        notification.style.right = '20px';
        notification.style.padding = '12px 20px';
        notification.style.background = colors[type] || colors.info;
        notification.style.color = '#fff';
        notification.style.borderRadius = '4px';
        notification.style.boxShadow = '0 4px 8px rgba(0, 0, 0, 0.2)';
        notification.style.zIndex = '9999';
        notification.style.maxWidth = '400px';
        notification.style.wordWrap = 'break-word';
        
        // Format message for multiline if needed
        notification.innerHTML = message.replace(/\n/g, '<br>');
        
        // Add to DOM
        document.body.appendChild(notification);
        
        // Remove after 5 seconds
        setTimeout(() => {
            notification.style.opacity = '0';
            notification.style.transition = 'opacity 0.5s';
            setTimeout(() => {
                document.body.removeChild(notification);
            }, 500);
        }, 5000);
    }

    // Function to debug the vessel model structure
    function logComponentCounts() {
        if (window.currentVesselModel && window.currentVesselModel.config) {
            const config = window.currentVesselModel.config;
            
            console.log('------- Current Component Counts -------');
            // Control surfaces
            if (config.control_surfaces && Array.isArray(config.control_surfaces.control_surfaces)) {
                console.log(`Control Surfaces: ${config.control_surfaces.control_surfaces.length}`);
                console.log(config.control_surfaces.control_surfaces);
            } else {
                console.log('Control Surfaces: Not properly initialized');
                console.log(config.control_surfaces);
            }
            
            // Thrusters
            if (config.thrusters && Array.isArray(config.thrusters.thrusters)) {
                console.log(`Thrusters: ${config.thrusters.thrusters.length}`);
            } else {
                console.log('Thrusters: Not properly initialized');
            }
            
            // Sensors
            if (config.sensors && Array.isArray(config.sensors.sensors)) {
                console.log(`Sensors: ${config.sensors.sensors.length}`);
            } else {
                console.log('Sensors: Not properly initialized');
            }
            console.log('---------------------------------------');
        } else {
            console.log('Vessel model not initialized');
        }
    }

    // Add handler for Advanced Simulation Parameters button
    document.getElementById('btnAdvancedSimParams')?.addEventListener('click', function() {
        try {
            const yamlGenerator = new YAMLGenerator();
            
            // Get current simulation parameters from the vessel model
            // Either use from config or get them from the UI if not yet saved
            let simParams = window.currentVesselModel.config.simulation || {};
            if (Object.keys(simParams).length === 0) {
                simParams = {
                    sim_time: parseFloat(document.getElementById('simTime')?.value || 100),
                    time_step: parseFloat(document.getElementById('timeStep')?.value || 0.01),
                    density: parseFloat(document.getElementById('density')?.value || 1025),
                    gravity: parseFloat(document.getElementById('gravity')?.value || 9.80665),
                    world_size: [
                        parseFloat(document.getElementById('worldSizeX')?.value || 1000),
                        parseFloat(document.getElementById('worldSizeY')?.value || 1000),
                        parseFloat(document.getElementById('worldSizeZ')?.value || 100)
                    ],
                    gps_datum: [
                        parseFloat(document.getElementById('gpsDatumLat')?.value || 12.99300),
                        parseFloat(document.getElementById('gpsDatumLon')?.value || 80.23913),
                        parseFloat(document.getElementById('gpsDatumAlt')?.value || -87.0)
                    ],
                    geofence: yamlGenerator.defaultSimParams.geofence || [],
                    nagents: 1,
                    agents: [{
                        name: window.currentVesselModel.config.name || 'vessel',
                        type: window.currentVesselModel.config.type || 'auv'
                    }]
                };
            }
            
            // Generate YAML string from current simulation parameters
            const yamlString = jsyaml.dump(simParams, { lineWidth: -1 });
            document.getElementById('simParamsYaml').value = yamlString;
            
            console.log("Opening simulation params modal with:", simParams);
            
            // Show the modal
            const simParamsModal = new bootstrap.Modal(document.getElementById('simParamsModal'));
            simParamsModal.show();
        } catch (error) {
            console.error("Error opening simulation parameters modal:", error);
            showNotification("Error loading simulation parameters", "error");
        }
    });
    
    // Add handler for simulation parameters file upload
    document.getElementById('simParamsFileUpload')?.addEventListener('change', function(event) {
        const file = event.target.files[0];
        if (file) {
            const reader = new FileReader();
            reader.onload = function(e) {
                document.getElementById('simParamsYaml').value = e.target.result;
            };
            reader.readAsText(file);
        }
    });
    
    // Add handler for Save Simulation Parameters button
    document.getElementById('btnSaveSimParams')?.addEventListener('click', function() {
        try {
            // Get the YAML text from the textarea
            const yamlText = document.getElementById('simParamsYaml').value;
            
            // Parse the YAML into an object
            const simParams = jsyaml.load(yamlText);
            console.log("Parsed simulation parameters:", simParams);
            
            if (!simParams || typeof simParams !== 'object') {
                throw new Error("Invalid YAML format");
            }
            
            // Add the fullReplace flag to replace all parameters
            simParams.fullReplace = true;
            
            // Update the vessel model with the new parameters
            window.currentVesselModel.updateSimulationParams(simParams);
            
            // Update the UI inputs to reflect the new values
            if (simParams.sim_time) document.getElementById('simTime').value = simParams.sim_time;
            if (simParams.time_step) document.getElementById('timeStep').value = simParams.time_step;
            if (simParams.density) document.getElementById('density').value = simParams.density;
            if (simParams.gravity) document.getElementById('gravity').value = simParams.gravity;
            
            if (simParams.world_size && Array.isArray(simParams.world_size)) {
                document.getElementById('worldSizeX').value = simParams.world_size[0] || 1000;
                document.getElementById('worldSizeY').value = simParams.world_size[1] || 1000;
                document.getElementById('worldSizeZ').value = simParams.world_size[2] === 'Inf' ? 100 : simParams.world_size[2] || 100;
            }
            
            if (simParams.gps_datum && Array.isArray(simParams.gps_datum)) {
                document.getElementById('gpsDatumLat').value = simParams.gps_datum[0] || 12.99300;
                document.getElementById('gpsDatumLon').value = simParams.gps_datum[1] || 80.23913;
                document.getElementById('gpsDatumAlt').value = simParams.gps_datum[2] || -87.0;
            }
            
            // Generate the simulation_input.yml directly and update the preview
            const yamlGenerator = new YAMLGenerator();
            const simInputYaml = yamlGenerator.generateSimulationInputYAML(window.currentVesselModel.config);
            document.getElementById('simulation-yaml-content').textContent = simInputYaml;
            
            // Also update all other YAML previews
            generateYAMLPreviews();
            
            // Close the modal
            const simParamsModal = bootstrap.Modal.getInstance(document.getElementById('simParamsModal'));
            simParamsModal.hide();
            
            // Show success notification
            showNotification("Simulation parameters updated successfully", "success");
        } catch (error) {
            console.error("Error saving simulation parameters:", error);
            showNotification("Error: " + error.message, "error");
        }
    });

    // Add handler for theme toggle
    document.getElementById('btn-toggle-theme')?.addEventListener('click', function() {
        // Check if dark mode is currently enabled
        const isDarkMode = document.documentElement.getAttribute('data-bs-theme') === 'dark';
        
        // Toggle the theme
        document.documentElement.setAttribute('data-bs-theme', isDarkMode ? 'light' : 'dark');
        
        // Save preference in localStorage
        localStorage.setItem('theme', isDarkMode ? 'light' : 'dark');
        
        // Show notification
        showNotification(`Switched to ${isDarkMode ? 'light' : 'dark'} mode`, 'info');
    });
    
    // Initialize theme based on saved preference or system preference
    function initializeTheme() {
        // Check if theme is saved in localStorage
        const savedTheme = localStorage.getItem('theme');
        
        if (savedTheme) {
            // Use saved preference
            document.documentElement.setAttribute('data-bs-theme', savedTheme);
        } else {
            // Default to dark mode instead of system preference
            document.documentElement.setAttribute('data-bs-theme', 'dark');
            
            // Listen for system preference changes
            window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', e => {
                if (!localStorage.getItem('theme')) {
                    // Only apply system preference if user hasn't manually set a preference
                    const newMode = e.matches ? 'dark' : 'light';
                    document.documentElement.setAttribute('data-bs-theme', newMode);
                    showNotification(`Matched system theme: ${newMode} mode`, 'info');
                }
            });
        }
        
        // Apply theme-specific adjustments for components that need special handling
        applyThemeSpecificAdjustments();
    }
    
    // Apply additional theme-specific adjustments to components that need special handling
    function applyThemeSpecificAdjustments() {
        const isDarkMode = document.documentElement.getAttribute('data-bs-theme') === 'dark';
        
        // Adjust the Three.js scene background
        if (window.threeScene && window.threeScene.scene) {
            // Keep the 3D scene background dark in both themes, but slightly lighter in light mode
            window.threeScene.scene.background = new THREE.Color(isDarkMode ? '#111111' : '#1e1e1e');
        }
        
        // Fix code preview areas for YAML
        document.querySelectorAll('.yaml-preview, .code-example').forEach(el => {
            el.style.border = `1px solid var(--border-color)`;
            el.style.backgroundColor = `var(--code-bg)`;
            el.style.color = `var(--text-color)`;
        });
        
        // Apply simple syntax highlighting to code examples
        document.querySelectorAll('.code-example').forEach(el => {
            // Skip if already processed
            if (el.dataset.highlighted === 'true') return;
            
            let content = el.innerHTML;
            
            // Highlight comments
            content = content.replace(/(#.+)$/gm, '<span class="comment">$1</span>');
            
            // Highlight keys and values in YAML
            content = content.replace(/^(\s*)([A-Za-z0-9_-]+)(:)/gm, '$1<span class="key">$2</span>$3');
            
            // Highlight numbers
            content = content.replace(/: (-?\d+(\.\d+)?)/g, ': <span class="number">$1</span>');
            
            // Highlight arrays/lists
            content = content.replace(/(\[.+?\])/g, '<span class="string">$1</span>');
            
            el.innerHTML = content;
            el.dataset.highlighted = 'true';
        });
        
        // Fix inline code elements in alerts
        document.querySelectorAll('.alert code').forEach(el => {
            el.style.backgroundColor = isDarkMode ? 'rgba(0, 0, 0, 0.2)' : 'rgba(255, 255, 255, 0.5)';
            el.style.color = `var(--text-color)`;
            el.style.borderColor = isDarkMode ? 'rgba(0, 0, 0, 0.3)' : 'rgba(0, 0, 0, 0.1)';
        });
        
        // Ensure alert text is visible in dark mode
        if (isDarkMode) {
            document.querySelectorAll('.alert').forEach(el => {
                el.style.setProperty('--alert-text-opacity', '1');
            });
        }
        
        // Fix placeholder text color
        document.querySelectorAll('input, textarea').forEach(el => {
            el.style.setProperty('--placeholder-opacity', isDarkMode ? '0.7' : '0.5');
        });
        
        // Apply theme to any SVG icons
        document.querySelectorAll('svg').forEach(svg => {
            if (!svg.hasAttribute('data-no-theme')) {
                svg.style.fill = `var(--text-color)`;
            }
        });
    }
    
    // Call the initializeTheme function when the page loads
    document.addEventListener('DOMContentLoaded', initializeTheme);
    
    // Also listen for theme changes to apply specific adjustments
    document.addEventListener('themeChange', applyThemeSpecificAdjustments);
    
    // Observe theme attribute changes and dispatch custom event
    const observer = new MutationObserver(mutations => {
        mutations.forEach(mutation => {
            if (mutation.attributeName === 'data-bs-theme') {
                document.dispatchEvent(new CustomEvent('themeChange', {
                    detail: { theme: document.documentElement.getAttribute('data-bs-theme') }
                }));
                applyThemeSpecificAdjustments();
            }
        });
    });
    
    // Start observing data-bs-theme attribute changes
    observer.observe(document.documentElement, { attributes: true });
}); 