import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

const nodeEnv =
  (globalThis as typeof globalThis & { process?: { env?: Record<string, string | undefined> } }).process?.env ?? {};
const playbackControlTarget =
  nodeEnv.VITE_PLAYBACK_CONTROL_TARGET ??
  `http://127.0.0.1:${nodeEnv.WEB_PLAYBACK_CONTROL_PORT ?? nodeEnv.CONTROL_PORT ?? "8765"}`;
const rosbridgeTarget =
  nodeEnv.VITE_ROSBRIDGE_TARGET ??
  `ws://127.0.0.1:${nodeEnv.WEB_ROSBRIDGE_PORT ?? nodeEnv.ROSBRIDGE_PORT ?? "9090"}`;
const hifiBridgeTarget =
  nodeEnv.VITE_HIFI_BRIDGE_TARGET ??
  `http://127.0.0.1:${nodeEnv.WEB_HIFI_BRIDGE_PORT ?? nodeEnv.HIFI_PORT ?? "8876"}`;
const hifiWindowTarget =
  nodeEnv.VITE_HIFI_WINDOW_TARGET ??
  `http://127.0.0.1:${nodeEnv.WEB_HIFI_WINDOW_PORT ?? nodeEnv.HIFI_WINDOW_PORT ?? "8877"}`;
const isaacGaussianOnlineTarget =
  nodeEnv.VITE_ISAAC_GAUSSIAN_ONLINE_TARGET ??
  `http://127.0.0.1:${nodeEnv.WEB_ISAAC_GAUSSIAN_ONLINE_PORT ?? nodeEnv.ISAAC_GAUSSIAN_ONLINE_PORT ?? "8890"}`;
const isaacGaussianMapperTarget =
  nodeEnv.VITE_ISAAC_GAUSSIAN_MAPPER_TARGET ??
  `http://127.0.0.1:${nodeEnv.WEB_ISAAC_GAUSSIAN_MAPPER_PORT ?? nodeEnv.ISAAC_GAUSSIAN_MAPPER_PORT ?? "8891"}`;
const worldNavTarget =
  nodeEnv.VITE_WORLD_NAV_TARGET ??
  `http://127.0.0.1:${nodeEnv.WEB_WORLD_NAV_PORT ?? nodeEnv.WORLD_NAV_PORT ?? "8892"}`;

export default defineConfig({
  plugins: [react()],
  server: {
    host: "0.0.0.0",
    port: 5173,
    proxy: {
      "/__playback_control": {
        target: playbackControlTarget,
        changeOrigin: true,
        rewrite: (path) => path.replace(/^\/__playback_control/, "")
      },
      "/__hifi_bridge": {
        target: hifiBridgeTarget,
        changeOrigin: true,
        rewrite: (path) => path.replace(/^\/__hifi_bridge/, "")
      },
      "/__hifi_window": {
        target: hifiWindowTarget,
        changeOrigin: true,
        rewrite: (path) => path.replace(/^\/__hifi_window/, "")
      },
      "/__rosbridge": {
        target: rosbridgeTarget,
        ws: true,
        changeOrigin: true,
        rewrite: (path) => path.replace(/^\/__rosbridge/, "")
      },
      "/__isaac_gaussian_online": {
        target: isaacGaussianOnlineTarget,
        changeOrigin: true,
        rewrite: (path) => path.replace(/^\/__isaac_gaussian_online/, "")
      },
      "/__isaac_gaussian_mapper": {
        target: isaacGaussianMapperTarget,
        changeOrigin: true,
        rewrite: (path) => path.replace(/^\/__isaac_gaussian_mapper/, "")
      },
      "/__world_nav": {
        target: worldNavTarget,
        changeOrigin: true,
        rewrite: (path) => path.replace(/^\/__world_nav/, "")
      }
    }
  },
  preview: {
    host: "0.0.0.0",
    port: 4173
  }
});
