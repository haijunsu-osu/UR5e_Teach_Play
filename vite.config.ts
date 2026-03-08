import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

export default defineConfig(({ mode }) => ({
  base: mode === "production" ? "/UR5e_Teach_Play/" : "/",
  plugins: [react()],
}));
