class Controls {
    constructor(threeScene) {
        this.threeScene = threeScene;
        this.setupKeyboardShortcuts();
    }

    setupKeyboardShortcuts() {
        document.addEventListener('keydown', (event) => {
            switch(event.key) {
                case 'g':
                    this.threeScene.toggleGrid();
                    break;
                case 'w':
                    this.threeScene.toggleWireframe();
                    break;
                case 'r':
                    this.threeScene.resetCamera();
                    break;
                case 'Escape':
                    if (this.threeScene.transformControls.object) {
                        this.threeScene.transformControls.detach();
                    }
                    break;
            }
        });
    }
}

// Export the class
window.Controls = Controls; 