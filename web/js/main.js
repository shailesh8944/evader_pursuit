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
}); 