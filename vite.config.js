import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import tailwindcss from '@tailwindcss/vite'

// https://vite.dev/config/
export default defineConfig({
  server: {
    host: '0.0.0.0',
    allowedHosts:[
      "macbook-pro.tail432847.ts.net",
    ],
    port: 5173,
  },
  plugins: [
    tailwindcss(),
    react()
  ],
})
