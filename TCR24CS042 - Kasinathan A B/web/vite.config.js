import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// https://vite.dev/config/
export default defineConfig({
  // GitHub Pages serves project sites at /repo-name/
  // This must match exactly so asset imports and the WASM locateFile resolve correctly.
  base: '/Safe-Semantic-Planner/',

  plugins: [react()],

  server: {
    port: 5173,
    headers: {
      'Cross-Origin-Opener-Policy': 'same-origin',
      'Cross-Origin-Embedder-Policy': 'require-corp',
    },
  },

  optimizeDeps: {
    exclude: ['@emscripten/wasm'],
  },
})
