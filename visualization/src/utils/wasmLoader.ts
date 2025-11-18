import type { UrdfxModule, UrdfxModuleFactory } from '../../public/urdfx';

let modulePromise: Promise<UrdfxModule> | null = null;
let moduleInstance: UrdfxModule | null = null;

/**
 * Loads the WASM factory function from /public/urdfx.js
 */
async function loadWasmFactory(): Promise<UrdfxModuleFactory> {
  // Check if already loaded globally
  if ((window as any).createUrdfxModule) {
    return (window as any).createUrdfxModule;
  }

  // Fetch and evaluate the script to define createUrdfxModule globally
  const response = await fetch('/urdfx.js');
  if (!response.ok) {
    throw new Error(`Failed to fetch urdfx.js: ${response.statusText}`);
  }
  
  const scriptContent = await response.text();
  
  // Use indirect eval to execute in global scope
  (0, eval)(scriptContent);
  
  // Check if it's now available
  const factory = (window as any).createUrdfxModule;
  if (!factory) {
    throw new Error('urdfx.js loaded but createUrdfxModule not found in global scope');
  }
  
  return factory;
}

/**
 * Loads the urdfx WASM module. Returns cached instance if already loaded.
 */
export async function loadUrdfxModule(): Promise<UrdfxModule> {
  if (moduleInstance) {
    return moduleInstance;
  }

  if (modulePromise) {
    return modulePromise;
  }

  modulePromise = (async () => {
    try {
      // Load the WASM factory script dynamically using script tag
      // since Vite doesn't allow importing from /public
      const factory = await loadWasmFactory();
      
      // Initialize the module with proper configuration
      const module = await factory({
        locateFile: (path: string) => {
          // Ensure WASM file is loaded from public directory
          if (path.endsWith('.wasm')) {
            return '/urdfx.wasm';
          }
          return path;
        },
        print: (text: string) => console.log('[WASM]', text),
        printErr: (text: string) => console.error('[WASM Error]', text),
      });

      moduleInstance = module;
      return module;
    } catch (error) {
      modulePromise = null;
      throw new Error(`Failed to load urdfx WASM module: ${error}`);
    }
  })();

  return modulePromise;
}

/**
 * Gets the loaded module instance. Throws if not loaded.
 */
export function getUrdfxModule(): UrdfxModule {
  if (!moduleInstance) {
    throw new Error('urdfx module not loaded. Call loadUrdfxModule() first.');
  }
  return moduleInstance;
}

/**
 * Checks if the module is loaded
 */
export function isModuleLoaded(): boolean {
  return moduleInstance !== null;
}
