/**
 * Vessel Simulator - Marine Vessel Simulation Environment
 * 
 * This file provides a realistic 3D ocean environment for simulating 
 * marine vessels. It loads the vessel FBX model from the configurator
 * and places it in an interactive water scene for simulation.
 * 
 * Features:
 * - Realistic ocean water simulation with waves and movement
 * - Dynamic lighting with day/night cycle
 * - Vessel physics integration (prepared for ROS backend)
 * - Camera controls for viewing the simulation
 * - Environment controls (wave height, wind speed, etc.)
 * 
 * The simulator is designed to be ready for integration with ROS
 * for controlling vessel movement and physics.
 */

class VesselSimulator {
    constructor() {
        // Core Three.js components
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;
        this.clock = new THREE.Clock();
        this.stats = null;
        
        // Water and environment
        this.water = null;
        this.sky = null;
        this.sun = null;
        
        // Vessel
        this.vessel = null;
        this.vesselConfig = null;
        
        // Simulation parameters
        this.params = {
            waterLevel: 0,
            waveHeight: 0.2,
            waveSpeed: 1.0,
            windDirection: 0,
            windSpeed: 5.0,
            sunPosition: new THREE.Vector3(10, 10, -10),
            timeOfDay: 0.3, // Normalized time (0-1), 0.3 = morning
            waterColor: 0x001e0f,
            waterDistortion: 0.1,
            fogDensity: 0.01
        };
        
        // Animation/simulation state
        this.running = false;
        this.animationFrameId = null;
        
        // Container element
        this.container = null;
        
        // Controls
        this.gui = null;
    }
    
    /**
     * Initialize the simulator with a vessel FBX file and configuration
     * @param {string|File} vesselFbxPath - Path to the FBX file or File object
     * @param {Object} vesselConfig - Vessel configuration from the configurator
     * @param {HTMLElement} container - DOM element to render into
     */
    async init(vesselFbxPath, vesselConfig, container) {
        if (!container) {
            throw new Error('Container element is required');
        }
        
        this.container = container;
        this.vesselConfig = vesselConfig;
        
        // Setup renderer
        this.renderer = new THREE.WebGLRenderer({ 
            antialias: true,
            logarithmicDepthBuffer: true 
        });
        this.renderer.setPixelRatio(window.devicePixelRatio);
        this.renderer.setSize(container.clientWidth, container.clientHeight);
        this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
        this.renderer.toneMappingExposure = 0.5;
        this.renderer.shadowMap.enabled = true;
        container.appendChild(this.renderer.domElement);
        
        // Setup scene
        this.scene = new THREE.Scene();
        this.scene.fog = new THREE.FogExp2(0x87ceeb, this.params.fogDensity);
        this.scene.background = new THREE.Color(0x87ceeb);
        
        // Setup camera
        this.camera = new THREE.PerspectiveCamera(
            60, container.clientWidth / container.clientHeight, 0.1, 5000
        );
        this.camera.position.set(10, 10, 10);
        this.camera.lookAt(0, 0, 0);
        
        // Setup controls
        this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;
        this.controls.screenSpacePanning = false;
        this.controls.minDistance = 5;
        this.controls.maxDistance = 500;
        this.controls.maxPolarAngle = Math.PI * 0.95;
        this.controls.update();
        
        // Add stats if in development mode
        if (window.location.search.includes('stats=1')) {
            this.stats = new Stats();
            container.appendChild(this.stats.dom);
        }
        
        // Add window resize handler
        window.addEventListener('resize', this.onWindowResize.bind(this));
        
        // Create environment
        await this.createEnvironment();
        
        // Load vessel model
        await this.loadVesselModel(vesselFbxPath);
        
        // Start animation loop
        this.running = true;
        this.animate();
        
        // Add GUI controls if available
        if (window.dat && window.dat.GUI) {
            this.setupGUI();
        }
        
        return this;
    }
    
    /**
     * Create the ocean environment with water, sky, and lighting
     */
    async createEnvironment() {
        // Create sun light
        this.sun = new THREE.DirectionalLight(0xffffff, 1.0);
        this.sun.position.copy(this.params.sunPosition);
        this.sun.castShadow = true;
        this.sun.shadow.mapSize.width = 2048;
        this.sun.shadow.mapSize.height = 2048;
        this.sun.shadow.camera.near = 0.5;
        this.sun.shadow.camera.far = 500;
        this.sun.shadow.camera.left = -100;
        this.sun.shadow.camera.right = 100;
        this.sun.shadow.camera.top = 100;
        this.sun.shadow.camera.bottom = -100;
        this.scene.add(this.sun);
        
        // Add ambient light
        const ambientLight = new THREE.AmbientLight(0x404040, 0.3);
        this.scene.add(ambientLight);
        
        // Add hemisphere light
        const hemiLight = new THREE.HemisphereLight(0xffffff, 0x444444, 0.6);
        hemiLight.position.set(0, 50, 0);
        this.scene.add(hemiLight);
        
        // Load Ocean shader
        const waterGeometry = new THREE.PlaneGeometry(10000, 10000);
        this.water = new THREE.Water(
            waterGeometry,
            {
                textureWidth: 512,
                textureHeight: 512,
                waterNormals: new THREE.TextureLoader().load('https://raw.githubusercontent.com/mrdoob/three.js/master/examples/textures/waternormals.jpg', function(texture) {
                    texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
                }),
                sunDirection: new THREE.Vector3(this.params.sunPosition.x, this.params.sunPosition.y, this.params.sunPosition.z).normalize(),
                sunColor: 0xffffff,
                waterColor: this.params.waterColor,
                distortionScale: this.params.waterDistortion,
                fog: this.scene.fog !== undefined
            }
        );
        this.water.rotation.x = -Math.PI / 2;
        this.water.position.y = this.params.waterLevel;
        this.scene.add(this.water);
        
        // Create sky
        this.sky = new THREE.Sky();
        this.sky.scale.setScalar(10000);
        this.scene.add(this.sky);
        
        // Update sky based on parameters
        this.updateSky();
    }
    
    /**
     * Update the sky appearance based on time of day
     */
    updateSky() {
        // Convert normalized time to sun elevation
        const phi = 2 * Math.PI * (this.params.timeOfDay - 0.25);
        const theta = Math.PI * (0.25 + 0.2 * Math.sin(phi));
        
        const sunPosition = new THREE.Vector3();
        sunPosition.x = Math.cos(phi) * Math.cos(theta);
        sunPosition.y = Math.sin(theta);
        sunPosition.z = Math.sin(phi) * Math.cos(theta);
        sunPosition.multiplyScalar(100);
        
        this.sun.position.copy(sunPosition);
        
        // Update water reflection
        this.water.material.uniforms['sunDirection'].value.copy(this.sun.position).normalize();
        
        // Update sky
        const skyUniforms = this.sky.material.uniforms;
        skyUniforms['turbidity'].value = 10 - 5 * Math.sin(theta);
        skyUniforms['rayleigh'].value = 2 - Math.sin(theta);
        skyUniforms['mieCoefficient'].value = 0.005;
        skyUniforms['mieDirectionalG'].value = 0.8;
        skyUniforms['sunPosition'].value.copy(sunPosition);
        
        // Update fog based on time of day
        const fogColor = new THREE.Color(0x87ceeb);
        if (this.params.timeOfDay < 0.25 || this.params.timeOfDay > 0.75) {
            // Night
            fogColor.setRGB(0.1, 0.1, 0.2);
            this.scene.background = new THREE.Color(0x0a1a2a);
        } else if (this.params.timeOfDay < 0.3 || this.params.timeOfDay > 0.7) {
            // Dawn/Dusk
            fogColor.setRGB(0.7, 0.5, 0.3);
            this.scene.background = new THREE.Color(0x6a8fc5);
        } else {
            // Day
            fogColor.setRGB(0.5, 0.7, 0.9);
            this.scene.background = new THREE.Color(0x87ceeb);
        }
        
        this.scene.fog.color.copy(fogColor);
    }
    
    /**
     * Load the vessel FBX model into the scene
     * @param {string|File} vesselFbxPath - Path to the FBX file or File object
     */
    async loadVesselModel(vesselFbxPath) {
        if (!vesselFbxPath) {
            console.warn('No vessel model provided');
            return;
        }
        
        // Create loading indicator
        const loadingElement = document.createElement('div');
        loadingElement.style.position = 'absolute';
        loadingElement.style.top = '50%';
        loadingElement.style.left = '50%';
        loadingElement.style.transform = 'translate(-50%, -50%)';
        loadingElement.style.padding = '10px 20px';
        loadingElement.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
        loadingElement.style.color = 'white';
        loadingElement.style.borderRadius = '5px';
        loadingElement.textContent = 'Loading vessel model...';
        this.container.appendChild(loadingElement);
        
        try {
            const fbxLoader = new THREE.FBXLoader();
            
            // Handle both file objects and paths
            let model;
            if (typeof vesselFbxPath === 'string') {
                model = await new Promise((resolve, reject) => {
                    fbxLoader.load(
                        vesselFbxPath,
                        resolve,
                        (xhr) => {
                            const percent = Math.round((xhr.loaded / xhr.total) * 100);
                            loadingElement.textContent = `Loading vessel model... ${percent}%`;
                        },
                        reject
                    );
                });
            } else if (vesselFbxPath instanceof File) {
                const fileUrl = URL.createObjectURL(vesselFbxPath);
                model = await new Promise((resolve, reject) => {
                    fbxLoader.load(
                        fileUrl,
                        resolve,
                        (xhr) => {
                            const percent = xhr.loaded > 0 ? Math.round((xhr.loaded / xhr.total) * 100) : 0;
                            loadingElement.textContent = `Loading vessel model... ${percent}%`;
                        },
                        reject
                    );
                });
                URL.revokeObjectURL(fileUrl);
            }
            
            if (model) {
                this.vessel = model;
                
                // Calculate model center and size
                const bbox = new THREE.Box3().setFromObject(model);
                const center = bbox.getCenter(new THREE.Vector3());
                const size = bbox.getSize(new THREE.Vector3());
                
                // Calculate scale to normalize size
                const scale = 1 / Math.max(size.x, size.y, size.z);
                
                // Position model at water level
                model.position.set(0, this.params.waterLevel + (size.y * scale / 2), 0);
                
                // Apply scale if needed
                //model.scale.set(scale, scale, scale);
                
                // Add model to scene
                this.scene.add(model);
                
                // Position camera to see the vessel
                this.camera.position.set(
                    model.position.x + 5,
                    model.position.y + 5,
                    model.position.z + 5
                );
                this.controls.target.copy(model.position);
                this.controls.update();
                
                // Enable shadows
                model.traverse(child => {
                    if (child.isMesh) {
                        child.castShadow = true;
                        child.receiveShadow = true;
                    }
                });
                
                console.log('Vessel model loaded successfully');
            }
        } catch (error) {
            console.error('Error loading vessel model:', error);
        } finally {
            // Remove loading indicator
            loadingElement.remove();
        }
    }
    
    /**
     * Set up GUI controls for the simulator
     */
    setupGUI() {
        this.gui = new dat.GUI({ autoPlace: false });
        this.gui.domElement.style.position = 'absolute';
        this.gui.domElement.style.top = '0';
        this.gui.domElement.style.right = '0';
        this.container.appendChild(this.gui.domElement);
        
        // Environment controls
        const envFolder = this.gui.addFolder('Environment');
        envFolder.add(this.params, 'waveHeight', 0, 1).name('Wave Height').onChange(value => {
            this.water.material.uniforms['distortionScale'].value = value;
        });
        envFolder.add(this.params, 'waveSpeed', 0, 2).name('Wave Speed');
        envFolder.add(this.params, 'timeOfDay', 0, 1).name('Time of Day').onChange(() => {
            this.updateSky();
        });
        envFolder.add(this.params, 'windSpeed', 0, 20).name('Wind Speed (m/s)');
        envFolder.add(this.params, 'windDirection', 0, 360).name('Wind Direction (deg)');
        envFolder.addColor({ waterColor: '#001e0f' }, 'waterColor').onChange(value => {
            this.water.material.uniforms['waterColor'].value.setHex(parseInt(value.replace('#', '0x')));
        });
        envFolder.open();
        
        // Camera controls
        const cameraFolder = this.gui.addFolder('Camera');
        cameraFolder.add({ resetCamera: () => {
            if (this.vessel) {
                const bbox = new THREE.Box3().setFromObject(this.vessel);
                const center = bbox.getCenter(new THREE.Vector3());
                const size = bbox.getSize(new THREE.Vector3());
                
                // Position camera relative to vessel
                this.camera.position.set(
                    center.x + size.x * 2,
                    center.y + size.y * 2,
                    center.z + size.z * 2
                );
                this.controls.target.copy(center);
                this.controls.update();
            }
        }}, 'resetCamera').name('Reset Camera');
        cameraFolder.open();
        
        // Simulation controls
        const simFolder = this.gui.addFolder('Simulation');
        simFolder.add({ 
            toggleSimulation: () => {
                this.running = !this.running;
                if (this.running) {
                    this.animate();
                } else if (this.animationFrameId) {
                    cancelAnimationFrame(this.animationFrameId);
                    this.animationFrameId = null;
                }
            }
        }, 'toggleSimulation').name('Toggle Simulation');
        simFolder.open();
    }
    
    /**
     * Update the water surface
     * @param {number} delta - Time delta for animation
     */
    updateWater(delta) {
        if (this.water) {
            // Update water time value
            this.water.material.uniforms['time'].value += delta * this.params.waveSpeed;
        }
    }
    
    /**
     * Update vessel position and orientation (will be controlled by ROS)
     * @param {number} delta - Time delta for animation
     */
    updateVessel(delta) {
        if (this.vessel) {
            // This will be replaced with ROS integration
            // For now, just apply some gentle motion to simulate floating
            const time = this.clock.getElapsedTime();
            
            // Gentle bobbing and rocking
            this.vessel.position.y = this.params.waterLevel + 
                Math.sin(time * 0.5) * 0.05 * this.params.waveHeight;
            
            this.vessel.rotation.z = Math.sin(time * 0.3) * 0.02 * this.params.waveHeight;
            this.vessel.rotation.x = Math.sin(time * 0.4) * 0.01 * this.params.waveHeight;
        }
    }
    
    /**
     * Window resize handler
     */
    onWindowResize() {
        if (!this.container) return;
        
        const width = this.container.clientWidth;
        const height = this.container.clientHeight;
        
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(width, height);
    }
    
    /**
     * Animation loop
     */
    animate() {
        if (!this.running) return;
        
        this.animationFrameId = requestAnimationFrame(this.animate.bind(this));
        
        // Calculate time delta
        const delta = this.clock.getDelta();
        
        // Update controls
        this.controls.update();
        
        // Update water
        this.updateWater(delta);
        
        // Update vessel
        this.updateVessel(delta);
        
        // Render scene
        this.renderer.render(this.scene, this.camera);
        
        // Update stats if available
        if (this.stats) {
            this.stats.update();
        }
    }
    
    /**
     * Clean up resources
     */
    dispose() {
        // Stop animation loop
        this.running = false;
        if (this.animationFrameId) {
            cancelAnimationFrame(this.animationFrameId);
            this.animationFrameId = null;
        }
        
        // Remove event listeners
        window.removeEventListener('resize', this.onWindowResize);
        
        // Dispose GUI if exists
        if (this.gui) {
            this.gui.destroy();
            this.gui = null;
        }
        
        // Clean up Three.js resources
        if (this.vessel) {
            this.scene.remove(this.vessel);
            this.vessel.traverse(child => {
                if (child.geometry) child.geometry.dispose();
                if (child.material) {
                    if (Array.isArray(child.material)) {
                        child.material.forEach(mat => mat.dispose());
                    } else {
                        child.material.dispose();
                    }
                }
            });
            this.vessel = null;
        }
        
        if (this.water) {
            this.scene.remove(this.water);
            this.water.geometry.dispose();
            this.water.material.dispose();
            this.water = null;
        }
        
        if (this.sky) {
            this.scene.remove(this.sky);
            this.sky.geometry.dispose();
            this.sky.material.dispose();
            this.sky = null;
        }
        
        // Clear container
        if (this.container && this.renderer) {
            this.container.removeChild(this.renderer.domElement);
        }
        
        this.renderer = null;
        this.scene = null;
        this.camera = null;
        this.controls = null;
    }
} 