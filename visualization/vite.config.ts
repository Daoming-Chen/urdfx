import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// https://vite.dev/config/
export default defineConfig({
  plugins: [react()],
  optimizeDeps: {
    exclude: ['three/examples/jsm/loaders/OBJLoader.js'],
  },
  assetsInclude: ['**/*.wasm'],
})
