import "./storageShim.js";
import React from "react";
import { createRoot } from "react-dom/client";
import App from "./phodar.jsx";

createRoot(document.getElementById("root")).render(<App />);
