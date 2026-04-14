import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
export default defineConfig({
    plugins: [react()],
    server: {
        host: "0.0.0.0",
        port: 5173,
        proxy: {
            "/__playback_control": {
                target: "http://127.0.0.1:8765",
                changeOrigin: true,
                rewrite: function (path) { return path.replace(/^\/__playback_control/, ""); }
            },
            "/__hifi_bridge": {
                target: "http://127.0.0.1:8876",
                changeOrigin: true,
                rewrite: function (path) { return path.replace(/^\/__hifi_bridge/, ""); }
            },
            "/__hifi_window": {
                target: "http://127.0.0.1:8877",
                changeOrigin: true,
                rewrite: function (path) { return path.replace(/^\/__hifi_window/, ""); }
            },
            "/__rosbridge": {
                target: "ws://127.0.0.1:9090",
                ws: true,
                changeOrigin: true,
                rewrite: function (path) { return path.replace(/^\/__rosbridge/, ""); }
            }
        }
    },
    preview: {
        host: "0.0.0.0",
        port: 4173
    }
});
