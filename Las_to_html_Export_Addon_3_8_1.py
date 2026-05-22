bl_info = {
    "name": "LiDAR WebGL HTML Exporter - Chunked Quantized",
    "author": "AI Assistant, based on user addon",
    "version": (3, 7, 0),
    "blender": (5, 1, 0),
    "location": "View3D > N-Panel > LiDAR",
    "description": "Export multiple LiDAR point cloud objects to a single HTML viewer, with chunked loading, quantization, delta/Morton compression, global point limiter/cutoff, preview LOD, parallel chunk workers, auto external chunks for >100M pts, and viewer defaults",
    "category": "Import-Export",
}

import bpy
import numpy as np
import zlib
import base64
import os
import json
import math
import html
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from mathutils import Matrix, Vector
from bpy_extras.io_utils import ExportHelper


HTML_TEMPLATE = r'''<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>__TITLE_HTML__ - LiDAR Viewer</title>
    <style>
        body { margin: 0; overflow: hidden; background-color: #111; font-family: sans-serif; color: white; }
        .drag-zone { position: absolute; z-index: 50; }
        #rollZone { top: 0; left: 0; width: 7.5%; height: 100%; cursor: ns-resize; }
        #pitchZone { top: 0; right: 245px; width: 7.5%; height: 100%; cursor: ns-resize; }
        #yawZone { bottom: 54px; left: 7.5%; width: calc(100% - 245px - 7.5%); height: 7.5%; cursor: ew-resize; }
        #loading {
            display: none; position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%);
            color: #fff; font-size: 18px; background: rgba(0,0,0,0.92); padding: 18px 26px;
            border-radius: 8px; border: 1px solid #555; z-index: 999; box-shadow: 0 0 20px rgba(0,0,0,0.5);
            text-align: center; min-width: 300px; line-height: 1.45;
        }
        #loading small { display: block; color: #bbb; margin-top: 6px; font-size: 12px; }
        #footer {
            position: absolute; bottom: 0; width: 100%; background: rgba(0,0,0,0.85);
            padding: 10px 0; display: flex; justify-content: center; align-items: center; gap: 15px; z-index: 100;
            flex-wrap: wrap;
        }
        .btn-group { display: flex; gap: 2px; align-items: center; background: #222; padding: 4px; border-radius: 6px; }
        #footer button {
            cursor: pointer; padding: 8px 15px; font-weight: bold; font-size: 14px;
            background: #444; color: white; border: none; border-radius: 4px; transition: background 0.2s;
            position: relative;
        }
        #footer button:hover { background: #666; }
        .view-btn { padding: 8px 20px !important; }
        .store-btn { background: #b35900 !important; }
        .store-btn:hover { background: #e67300 !important; }
        #camSelect {
            background: #222; color: white; border: none; padding: 8px 10px;
            border-radius: 4px; font-size: 14px; font-weight: bold; cursor: pointer; outline: none;
            max-width: 360px;
        }
        #scaleBarContainer {
            display: none; position: absolute; bottom: 70px; left: 50%; transform: translateX(-50%);
            text-align: center; pointer-events: none; z-index: 90;
        }
        #scaleBarText { font-weight: bold; margin-bottom: 4px; font-size: 14px; }
        #scaleBarLine { border-bottom: 2px solid; border-left: 2px solid; border-right: 2px solid; height: 8px; width: 100%; box-sizing: border-box; }
        #statsBadge {
            position: absolute; left: 10px; top: 10px; z-index: 80; background: rgba(0,0,0,0.65);
            border: 1px solid #444; border-radius: 6px; padding: 6px 9px; font-size: 12px; color: #ddd;
            pointer-events: none; white-space: pre;
        }
        #customTooltip {
            position: absolute;
            pointer-events: none;
            z-index: 2000;
            background: rgba(20, 20, 20, 0.92);
            color: #eee;
            padding: 6px 10px;
            border-radius: 4px;
            font-size: 13px;
            font-family: sans-serif;
            white-space: nowrap;
            display: none;
            border: 1px solid #666;
            box-shadow: 0 2px 8px rgba(0,0,0,0.6);
        }
    </style>
    <script type="importmap">
        {
            "imports": {
                "three": "https://unpkg.com/three@0.160.0/build/three.module.js",
                "three/addons/": "https://unpkg.com/three@0.160.0/examples/jsm/",
                "fflate": "https://unpkg.com/fflate@0.8.2/esm/browser.js"
            }
        }
    </script>
</head>
<body>
    <div id="rollZone" class="drag-zone" data-tooltip="Level horizon (roll) – drag up/down"></div>
    <div id="pitchZone" class="drag-zone" data-tooltip="Vertical tilt (pitch) – drag up/down"></div>
    <div id="yawZone" class="drag-zone" data-tooltip="Horizontal rotation (yaw) – drag left/right"></div>

    <div id="loading">Loading point cloud...<small>Starting</small></div>
    <div id="statsBadge">Loading...</div>

    <div id="scaleBarContainer">
        <div id="scaleBarText">10 m</div>
        <div id="scaleBarLine"></div>
    </div>

    <div id="customTooltip"></div>

    <div id="footer">
        <div class="btn-group">
            <button id="storeView" class="view-btn store-btn" data-tooltip="Store current view as default orientation for this session. Works when cloud is fully loaded.">Store</button>
            <button id="rotXPlus" data-tooltip="Rotate view +90° around local X axis (right vector)">+90x</button>
            <button id="viewTop" class="view-btn" data-tooltip="Switch to orthographic top-down view">Top View</button>
            <button id="rotXMinus" data-tooltip="Rotate view -90° around local X axis">-90x</button>
        </div>

        <div class="btn-group">
            <button id="prevCam" data-tooltip="Previous camera (Left Arrow)">&#9664;</button>
            <select id="camSelect" data-tooltip="Select a stored camera or special view"></select>
            <button id="nextCam" data-tooltip="Next camera (Right Arrow)">&#9654;</button>
        </div>

        <div class="btn-group">
            <button id="rotZPlus" data-tooltip="Rotate view +90° around global Z axis (yaw)">+90z</button>
            <button id="viewFront" class="view-btn" data-tooltip="Switch to orthographic front view">Front View</button>
            <button id="rotZMinus" data-tooltip="Rotate view -90° around global Z axis">-90z</button>
        </div>

        <button id="viewModeBtn" class="view-btn" data-tooltip="Cycle through Orthographic / Perspective / Fly modes">Perspective</button>
    </div>

    <script>
        window.PC_META = __PC_META_JSON__;
        window.CAMERAS_JSON = __CAMERAS_JSON__;
    </script>

    <script type="module">
        import * as THREE from 'three';
        import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
        import { GUI } from 'three/addons/libs/lil-gui.module.min.js';
        import * as fflate from 'fflate';

        const FILE_NAME = __FILE_NAME_JSON__;
        const META = window.PC_META;

        let renderer, scene, cameraPersp, cameraOrtho, currentCamera, controls, material;
        let pointsGroup = null;
        let previewGroup = null;
        let pointChunks = [];
        let previewPointChunks = [];
        let guiControllers = {};
        let cameraList = [];
        let currentCamIndex = 0;
        let totalPointCount = 0;
        let loadedPointCount = 0;
        let previewPointCount = 0;
        let previewLoadedPointCount = 0;
        let previewLoadingComplete = false;
        let loadingComplete = false;

        const cloudBox = new THREE.Box3();
        const cloudSphere = new THREE.Sphere();
        let cloudCenter = new THREE.Vector3();
        let cloudRadius = 1.0;

        // Default viewer settings values derived from exported meta defaults or system fallbacks
        const defaultOpacity = (META.viewerDefaults && META.viewerDefaults.opacity !== undefined) ? META.viewerDefaults.opacity : 1.0;
        const defaultBrightness = (META.viewerDefaults && META.viewerDefaults.brightness !== undefined) ? META.viewerDefaults.brightness : 0.0;
        const defaultGlobalIntensity = (META.viewerDefaults && META.viewerDefaults.globalIntensity !== undefined) ? META.viewerDefaults.globalIntensity : 0.0;
        const defaultLocalIntensity = (META.viewerDefaults && META.viewerDefaults.localIntensity !== undefined) ? META.viewerDefaults.localIntensity : 0.0;
        const defaultLimitMinPct = (META.viewerDefaults && META.viewerDefaults.pointLimitMinPct !== undefined) ? META.viewerDefaults.pointLimitMinPct : 0;
        const defaultLimitMaxPct = (META.viewerDefaults && META.viewerDefaults.pointLimitMaxPct !== undefined) ? META.viewerDefaults.pointLimitMaxPct : 100;

        const naturalValues = {
            pointLimitMinPct: defaultLimitMinPct,
            pointLimitMaxPct: defaultLimitMaxPct,
            clipGlobalMin: -1.0,
            clipGlobalMax: 3.0,
            clipFront: 0.1,
            clipBack: 100.0
        };

        const measurements = [];
        let isMeasuring = false;
        let measureState = 0;
        let currentMeasure = null;
        const raycaster = new THREE.Raycaster();
        const mouse = new THREE.Vector2();

        const params = {
            pointSize: 0.6,
            opacity: defaultOpacity,
            brightness: defaultBrightness,
            aoStrength: 0.8,
            blendMode: 'Normal',
            depthCull: true,
            camera: 'Perspective',
            navMode: 'Orbit',
            flySpeed: 10.0,
            displayMode: 'RGB',
            clipGlobalMin: -1.0,
            clipGlobalMax: 3.0,
            cutGlobal: false,
            globalIntensity: defaultGlobalIntensity,
            pointLimitPct: 100,
            pointLimitMinPct: defaultLimitMinPct,
            pointLimitMaxPct: defaultLimitMaxPct,
            pointLimitWindowPct: 3,
            clipFront: 0.1,
            clipBack: 100.0,
            cutLocal: false,
            localIntensity: defaultLocalIntensity,
            measureMode: false,
            measureColor: '#ffcc00',
            clearMeasurements: () => clearMeasurements(),
            bgColor: '#111111',
            captureRes: 1,
            takeScreenshot: () => takeScreenshot(),
            jitterEnabled: true,
            jitterAmount: 0.0035,

            // Lighting & shadow params (floor shadows are optional and disabled by default)
            lightType: '3-Point Rig',
            showFloorShadows: false,
            shadowOpacity: 0.6
        };

        const keys = { w: false, a: false, s: false, d: false, q: false, e: false, shift: false };
        const clock = new THREE.Clock();
        let isFlyDragging = false;
        const prevMouse = { x: 0, y: 0 };
        const modeMap = { 'RGB': 0.0, 'Intensity': 1.0, 'Monochrome': 2.0, 'NormalCol': 3.0 };
        const viewModes = ['Orthographic', 'Perspective', 'FlyMode'];
        let currentViewModeIdx = 1;

        let freezeRender = false;

        // -------- Tooltip handling --------
        function initTooltips() {
            const tooltip = document.getElementById('customTooltip');
            let timeoutId;
            let activeElement = null;

            function show(e) {
                tooltip.style.display = 'block';
                const offsetX = 15;
                const offsetY = 15;
                tooltip.style.left = (e.clientX + offsetX) + 'px';
                tooltip.style.top = (e.clientY + offsetY) + 'px';
            }

            function hide() {
                tooltip.style.display = 'none';
                if (timeoutId) {
                    clearTimeout(timeoutId);
                    timeoutId = null;
                }
                activeElement = null;
            }

            document.addEventListener('mouseover', (e) => {
                const target = e.target.closest('[data-tooltip]');
                if (!target || target === activeElement) return;
                hide();
                activeElement = target;
                const text = target.getAttribute('data-tooltip');
                if (!text) return;
                tooltip.textContent = text;
                timeoutId = setTimeout(() => {
                    show(e);
                    timeoutId = null;
                }, 2000);
            });

            document.addEventListener('mouseout', (e) => {
                const target = e.target.closest('[data-tooltip]');
                if (target === activeElement) {
                    hide();
                }
            });

            document.addEventListener('mousemove', (e) => {
                if (tooltip.style.display === 'block' && activeElement) {
                    const offsetX = 15;
                    const offsetY = 15;
                    tooltip.style.left = (e.clientX + offsetX) + 'px';
                    tooltip.style.top = (e.clientY + offsetY) + 'px';
                }
            });
        }

        function showLoading(main, small) {
            const el = document.getElementById('loading');
            el.style.display = 'block';
            el.innerHTML = `${main}<small>${small || ''}</small>`;
        }

        function hideLoading() {
            document.getElementById('loading').style.display = 'none';
        }

        function yieldToBrowser() {
            return new Promise(resolve => setTimeout(resolve, 0));
        }

        function formatInt(n) {
            return Number(n || 0).toLocaleString('en-US').replace(/,/g, ' ');
        }

        function updateStatsBadge() {
            const badge = document.getElementById('statsBadge');
            const chunksLoaded = pointChunks.length;
            const chunksTotal = (META.chunks || []).length;
            const mode = META.storage === 'external' ? 'external chunks' : 'embedded chunks';
            const q = META.quantized ? `quantized, precision ${META.precision}` : 'float32 positions';
            const previewLine = previewPointCount > 0
                ? `Preview LOD: ${formatInt(previewLoadedPointCount)} / ${formatInt(previewPointCount)} pts, ${previewPointChunks.length} chunks\n`
                : '';
            badge.textContent = `${previewLine}Full points: ${formatInt(loadedPointCount)} / ${formatInt(totalPointCount)}\nFull chunks: ${chunksLoaded} / ${chunksTotal}\n${q}, ${mode}`;
        }

        function sumChunkCounts(chunks) {
            return (chunks || []).reduce((acc, chunk) => acc + Number(chunk.count || 0), 0);
        }

        function ensureChunkGlobalRanges(chunks) {
            let cursor = 0;
            (chunks || []).forEach((chunk, sequenceIndex) => {
                const count = Number(chunk.count || 0);
                if (chunk.globalStart === undefined || chunk.globalEnd === undefined) {
                    chunk.globalStart = cursor;
                    chunk.globalEnd = cursor + count;
                }
                chunk.sequenceIndex = sequenceIndex;
                cursor = Number(chunk.globalEnd || (cursor + count));
            });
            return cursor;
        }

        function getPreviewLOD() {
            if (!META.lods || !Array.isArray(META.lods)) return null;
            return META.lods.find(lod => lod && lod.name === 'preview_1pct_q2' && lod.chunks && lod.chunks.length) || null;
        }

        function hasChunkPayload(chunk, key) {
            return !!(chunk[key + 'B64'] || chunk[key + 'File']);
        }

        async function compressedBytesFromChunk(chunk, key) {
            const b64Key = key + 'B64';
            const fileKey = key + 'File';

            if (chunk[b64Key]) {
                const binaryString = atob(chunk[b64Key]);
                const len = binaryString.length;
                const bytes = new Uint8Array(len);
                for (let i = 0; i < len; i++) bytes[i] = binaryString.charCodeAt(i);
                return bytes;
            }

            if (chunk[fileKey]) {
                const response = await fetch(chunk[fileKey]);
                if (!response.ok) throw new Error(`Cannot fetch ${chunk[fileKey]} (${response.status})`);
                return new Uint8Array(await response.arrayBuffer());
            }

            return null;
        }

        async function decodePayload(chunk, key) {
            const compressed = await compressedBytesFromChunk(chunk, key);
            if (!compressed) return null;
            return fflate.unzlibSync(compressed);
        }

        function decodePositions(bytes, chunk) {
            if (chunk.posType === 'float32') {
                const src = new Float32Array(bytes.buffer, bytes.byteOffset, bytes.byteLength / 4);
                return new Float32Array(src);
            }

            const invScale = 1.0 / (chunk.scale || META.scale || 1);
            const ox = chunk.origin[0], oy = chunk.origin[1], oz = chunk.origin[2];
            const positions = new Float32Array(chunk.count * 3);
            const encoding = chunk.posEncoding || 'absolute';

            if (encoding === 'delta_morton') {
                if (bytes.byteLength < 12) throw new Error('Invalid delta position payload.');
                const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
                let qx = view.getUint32(0, true);
                let qy = view.getUint32(4, true);
                let qz = view.getUint32(8, true);

                positions[0] = ox + qx * invScale;
                positions[1] = oy + qy * invScale;
                positions[2] = oz + qz * invScale;

                const deltaOffset = 12;
                const deltaType = chunk.deltaType || 'int16';
                const deltaCount = Math.max(0, (chunk.count - 1) * 3);

                if (deltaCount > 0) {
                    if (deltaType === 'int16') {
                        const deltas = new Int16Array(bytes.buffer, bytes.byteOffset + deltaOffset, deltaCount);
                        for (let p = 1, j = 3, d = 0; p < chunk.count; p++, j += 3) {
                            qx += deltas[d++];
                            qy += deltas[d++];
                            qz += deltas[d++];
                            positions[j] = ox + qx * invScale;
                            positions[j + 1] = oy + qy * invScale;
                            positions[j + 2] = oz + qz * invScale;
                        }
                    } else {
                        const deltas = new Int32Array(bytes.buffer, bytes.byteOffset + deltaOffset, deltaCount);
                        for (let p = 1, j = 3, d = 0; p < chunk.count; p++, j += 3) {
                            qx += deltas[d++];
                            qy += deltas[d++];
                            qz += deltas[d++];
                            positions[j] = ox + qx * invScale;
                            positions[j + 1] = oy + qy * invScale;
                            positions[j + 2] = oz + qz * invScale;
                        }
                    }
                }
                return positions;
            }

            const Typed = chunk.posType === 'uint16' ? Uint16Array : Uint32Array;
            const q = new Typed(bytes.buffer, bytes.byteOffset, bytes.byteLength / Typed.BYTES_PER_ELEMENT);

            for (let i = 0, j = 0; i < q.length; i += 3, j += 3) {
                positions[j] = ox + q[i] * invScale;
                positions[j + 1] = oy + q[i + 1] * invScale;
                positions[j + 2] = oz + q[i + 2] * invScale;
            }
            return positions;
        }

        function createMaterial() {
            material = new THREE.ShaderMaterial({
                defines: {
                    HAS_COLOR: META.hasColor ? 1 : 0,
                    HAS_INTENSITY: META.hasIntensity ? 1 : 0
                },
                uniforms: {
                    uPointSize: { value: params.pointSize },
                    uPixelRatio: { value: window.devicePixelRatio },
                    uOpacity: { value: params.opacity },
                    uBrightness: { value: params.brightness },
                    uAOStrength: { value: params.aoStrength },
                    uDisplayMode: { value: modeMap[params.displayMode] },
                    uGlobalZMin: { value: params.clipGlobalMin },
                    uGlobalZMax: { value: params.clipGlobalMax },
                    uCutGlobal: { value: params.cutGlobal ? 1.0 : 0.0 },
                    uGlobalIntensity: { value: params.globalIntensity },
                    uClipStart: { value: params.clipFront },
                    uClipEnd: { value: params.clipBack },
                    uCutLocal: { value: params.cutLocal ? 1.0 : 0.0 },
                    uLocalIntensity: { value: params.localIntensity },
                    uEnableJitter: { value: params.jitterEnabled ? 1.0 : 0.0 },
                    uJitterAmount: { value: params.jitterAmount },

                    // Full depth range for Depth Tint (independent from slicing)
                    uDepthFullStart: { value: params.clipFront },
                    uDepthFullEnd:   { value: params.clipBack },

                    // Lighting uniforms
                    // 0.0 = 3-Point Rig, 1.0 = Camera Omni
                    uLightType:    { value: params.lightType === '3-Point Rig' ? 0.0 : 1.0 },
                    uLightPosView: { value: new THREE.Vector3(0.5, 1.3, 1.0) },

                    uKeyLightDir:   { value: new THREE.Vector3(50, -50, 100).normalize() },
                    uKeyLightColor: { value: new THREE.Color('#fbf7e4') },
                    uFillLightDir:  { value: new THREE.Vector3(-50, 0, -25).normalize() },
                    uFillLightColor:{ value: new THREE.Color('#aca7d3') },
                    uRimLightDir:   { value: new THREE.Vector3(-10, -100, 33).normalize() },
                    uRimLightColor: { value: new THREE.Color('#ffffff') }
                },
                transparent: true,
                depthWrite: params.blendMode === 'Normal',
                depthTest: params.depthCull,
                blending: params.blendMode === 'Additive'
                    ? THREE.AdditiveBlending
                    : (params.blendMode === 'Subtractive'
                        ? THREE.SubtractiveBlending
                        : THREE.NormalBlending),
                vertexShader: `
                    uniform float uPointSize;
                    uniform float uPixelRatio;
                    uniform float uEnableJitter;
                    uniform float uJitterAmount;

                    #if HAS_COLOR == 1
                        attribute vec4 customColor;
                        varying vec4 vColor;
                    #endif

                    #if HAS_INTENSITY == 1
                        attribute float intensity;
                        varying float vIntensity;
                    #endif

                    varying float vDepth;
                    varying float vWorldZ;

                    float hash(vec3 p) {
                        return fract(sin(dot(p, vec3(12.9898, 78.233, 45.164))) * 43758.5453);
                    }

                    void main() {
                        #if HAS_COLOR == 1
                            vColor = customColor;
                        #endif
                        #if HAS_INTENSITY == 1
                            vIntensity = intensity;
                        #endif

                        vec3 pos = position;
                        if (uEnableJitter > 0.5) {
                            float offsetX = hash(pos + vec3(0.1)) * 2.0 - 1.0;
                            float offsetY = hash(pos + vec3(0.2)) * 2.0 - 1.0;
                            float offsetZ = hash(pos + vec3(0.3)) * 2.0 - 1.0;
                            pos += vec3(offsetX, offsetY, offsetZ) * uJitterAmount;
                        }

                        vec4 worldPosition = modelMatrix * vec4(pos, 1.0);
                        vWorldZ = worldPosition.z;
                        vec4 mvPosition = viewMatrix * worldPosition;
                        gl_Position = projectionMatrix * mvPosition;

                        if (projectionMatrix[2][3] == -1.0) {
                            gl_PointSize = uPointSize * uPixelRatio * (100.0 / max(-mvPosition.z, 0.0001));
                        } else {
                            gl_PointSize = uPointSize * uPixelRatio;
                        }
                        vDepth = -mvPosition.z;
                    }
                `,
                fragmentShader: `
                    uniform float uOpacity;
                    uniform float uBrightness;
                    uniform float uAOStrength;
                    uniform float uDisplayMode;
                    uniform float uGlobalZMin;
                    uniform float uGlobalZMax;
                    uniform float uCutGlobal;
                    uniform float uGlobalIntensity;
                    uniform float uClipStart;
                    uniform float uClipEnd;
                    uniform float uCutLocal;
                    uniform float uLocalIntensity;

                    // Full depth range for Depth Tint
                    uniform float uDepthFullStart;
                    uniform float uDepthFullEnd;

                    // Lighting
                    uniform float uLightType;    // 0 = 3-Point Rig, 1 = Camera Omni
                    uniform vec3  uLightPosView;

                    uniform vec3 uKeyLightDir;
                    uniform vec3 uKeyLightColor;
                    uniform vec3 uFillLightDir;
                    uniform vec3 uFillLightColor;
                    uniform vec3 uRimLightDir;
                    uniform vec3 uRimLightColor;

                    #if HAS_COLOR == 1
                        varying vec4 vColor;
                    #endif
                    #if HAS_INTENSITY == 1
                        varying float vIntensity;
                    #endif

                    varying float vDepth;
                    varying float vWorldZ;

                    void main() {
                        vec2 p = gl_PointCoord - vec2(0.5);
                        float dist = length(p);
                        if (dist > 0.5) discard;

                        float edgeAlpha = 1.0;
                        // Soft edge only when AO simulation is enabled
                        if (uAOStrength > 0.0) {
                            float normRadius = dist / 0.5;
                            if (normRadius > 0.7) {
                                edgeAlpha = 1.0 - smoothstep(0.7, 1.0, normRadius);
                            }
                        }

                        if (uCutLocal > 0.5 && (vDepth < uClipStart || vDepth > uClipEnd)) discard;
                        if (uCutGlobal > 0.5 && (vWorldZ < uGlobalZMin || vWorldZ > uGlobalZMax)) discard;

                        // Base color + NormalCol mode
                        vec3 baseColor;
                        if (uDisplayMode < 0.5) {
                            #if HAS_COLOR == 1
                                baseColor = vColor.rgb;
                            #else
                                baseColor = vec3(0.85);
                            #endif
                        } else if (uDisplayMode < 1.5) {
                            #if HAS_INTENSITY == 1
                                baseColor = vec3(vIntensity);
                            #else
                                baseColor = vec3(0.85);
                            #endif
                        } else if (uDisplayMode < 2.5) {
                            baseColor = vec3(0.85);
                        } else {
                            // NormalCol: hemisphere normal in view space encoded as RGB
                            vec2 normXY = p * 2.0;
                            float normZ = sqrt(max(0.0, 1.0 - dot(normXY, normXY)));
                            vec3 normal = normalize(vec3(normXY.x, -normXY.y, normZ));
                            baseColor = normal * 0.5 + 0.5;
                        }

                        vec3 finalColor = baseColor;

                        // Elevation tint from global Z
                        float normGlobal = clamp((vWorldZ - uGlobalZMin) / max(uGlobalZMax - uGlobalZMin, 0.001), 0.0, 1.0);
                        vec3 zTintColor = mix(vec3(1.0, 0.0, 0.0), vec3(0.0, 1.0, 1.0), normGlobal);
                        float zTintStrength = (uGlobalIntensity > 0.0) ? pow(uGlobalIntensity, 0.4) : 0.0;
                        if (zTintStrength > 0.0) {
                            finalColor = mix(finalColor, zTintColor, zTintStrength);
                        }

                        // Depth tint using full depth range (independent from local slicing)
                        float normLocal = clamp((vDepth - uDepthFullStart) / max(uDepthFullEnd - uDepthFullStart, 0.001), 0.0, 1.0);
                        float depthStrength = (uLocalIntensity > 0.0) ? clamp(uLocalIntensity * normLocal, 0.0, 1.0) : 0.0;
                        if (depthStrength > 0.0) {
                            vec3 fogTarget = vec3(0.25, 0.30, 0.80);
                            float baseLum = dot(finalColor, vec3(0.299, 0.587, 0.114));
                            float fogLum  = dot(fogTarget, vec3(0.299, 0.587, 0.114));
                            if (fogLum > baseLum) {
                                fogTarget *= baseLum / max(fogLum, 1e-3);
                            }
                            finalColor = mix(finalColor, fogTarget, depthStrength);
                        }

                        // AO / lighting – disabled if uAOStrength == 0
                        if (uAOStrength > 0.0) {
                            vec2 normXY = p * 2.0;
                            float normZ = sqrt(max(0.0, 1.0 - dot(normXY, normXY)));
                            vec3 normal = vec3(normXY.x, -normXY.y, normZ);
                            vec3 viewDir = vec3(0.0, 0.0, 1.0);
                            vec3 shading;

                            if (uLightType < 0.5) {
                                // 3-Point Rig in view space
                                vec3 keyDir = normalize(uKeyLightDir);
                                float keyDiff = max(dot(normal, keyDir), 0.0);
                                vec3 halfKey = normalize(keyDir + viewDir);
                                float keySpec = pow(max(dot(normal, halfKey), 0.0), 32.0);
                                vec3 keyLightEffect = uKeyLightColor * (keyDiff + keySpec * 0.4);

                                vec3 fillDir = normalize(uFillLightDir);
                                float fillDiff = max(dot(normal, fillDir), 0.0);
                                vec3 fillLightEffect = uFillLightColor * (fillDiff * 0.5);

                                vec3 rimDir = normalize(uRimLightDir);
                                float rimDiff = max(dot(normal, rimDir), 0.0);
                                float rimGlow = pow(1.0 - max(dot(normal, viewDir), 0.0), 4.0);
                                vec3 rimLightEffect = uRimLightColor * (rimDiff * 0.2 + rimGlow * 0.65);

                                shading = vec3(0.15) + keyLightEffect + fillLightEffect + rimLightEffect;
                            } else {
                                // Camera Omni – light position in view space
                                vec3 posView = vec3(0.0, 0.0, -vDepth);
                                vec3 lightDir = normalize(uLightPosView - posView);
                                float diffuse = max(dot(normal, lightDir), 0.0);
                                vec3 halfDir = normalize(lightDir + viewDir);
                                float specular = pow(max(dot(normal, halfDir), 0.0), 32.0);
                                shading = vec3(0.2) + vec3(diffuse * 0.8 + specular * 0.45);
                            }

                            shading = max(shading, vec3(0.2));
                            shading = mix(vec3(1.0), shading, clamp(uAOStrength, 0.0, 1.0));
                            finalColor *= shading;
                        }

                        // Brightness with attenuation for Depth Tint – far points are slightly dimmed
                        float effBrightness = uBrightness;
                        if (uLocalIntensity > 0.0) {
                            float normLocalB = clamp((vDepth - uDepthFullStart) / max(uDepthFullEnd - uDepthFullStart, 0.001), 0.0, 1.0);
                            float dim = 0.4 * uLocalIntensity * normLocalB;
                            effBrightness = max(uBrightness - dim, -1.0);
                        }

                        if (effBrightness > 0.0) finalColor = mix(finalColor, vec3(1.0), effBrightness);
                        else if (effBrightness < 0.0) finalColor = mix(finalColor, vec3(0.0), -effBrightness);

                        gl_FragColor = vec4(finalColor, uOpacity * edgeAlpha);
                    }
                `
            });
            applyMaterialState();
        }

        function applyMaterialState() {
            if (!material) return;
            material.uniforms.uDisplayMode.value = modeMap[params.displayMode];
            material.uniforms.uOpacity.value = params.opacity;
            material.uniforms.uBrightness.value = params.brightness;
            material.uniforms.uGlobalIntensity.value = params.globalIntensity;
            material.uniforms.uLocalIntensity.value = params.localIntensity;
            material.uniforms.uPointSize.value = params.pointSize;
            material.uniforms.uAOStrength.value = params.aoStrength;
            material.uniforms.uEnableJitter.value = params.jitterEnabled ? 1.0 : 0.0;
            material.uniforms.uJitterAmount.value = params.jitterAmount;
            material.uniforms.uCutGlobal.value = params.cutGlobal ? 1.0 : 0.0;
            material.uniforms.uCutLocal.value = params.cutLocal ? 1.0 : 0.0;
            material.uniforms.uClipStart.value = params.clipFront;
            material.uniforms.uClipEnd.value = params.clipBack;
            material.uniforms.uGlobalZMin.value = params.clipGlobalMin;
            material.uniforms.uGlobalZMax.value = params.clipGlobalMax;
            material.uniforms.uDepthFullStart.value = params.clipFront;
            material.uniforms.uDepthFullEnd.value = params.clipBack;
            material.uniforms.uLightType.value = params.lightType === '3-Point Rig' ? 0.0 : 1.0;

            if (params.blendMode === 'Additive') {
                material.blending = THREE.AdditiveBlending;
                material.depthWrite = false;
            } else if (params.blendMode === 'Subtractive') {
                material.blending = THREE.SubtractiveBlending;
                material.depthWrite = false;
            } else {
                material.blending = THREE.NormalBlending;
                material.depthWrite = true;
            }
            material.depthTest = params.depthCull;
            material.needsUpdate = true;
        }

        function initViewer() {
            THREE.Object3D.DEFAULT_UP.set(0, 0, 1);
            totalPointCount = META.pointCount || 0;
            ensureChunkGlobalRanges(META.chunks || []);
            const previewLOD = getPreviewLOD();
            if (previewLOD) {
                ensureChunkGlobalRanges(previewLOD.chunks || []);
                previewPointCount = previewLOD.pointCount || sumChunkCounts(previewLOD.chunks || []);
            }

            renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
            renderer.setSize(window.innerWidth, window.innerHeight);
            renderer.setPixelRatio(window.devicePixelRatio);
            renderer.setClearColor(params.bgColor);
            // Enable shadow map for optional shadow floor
            renderer.shadowMap.enabled = true;
            renderer.shadowMap.type = THREE.PCFSoftShadowMap;
            document.body.appendChild(renderer.domElement);

            scene = new THREE.Scene();
            previewGroup = new THREE.Group();
            previewGroup.name = 'Preview LOD';
            scene.add(previewGroup);
            pointsGroup = new THREE.Group();
            pointsGroup.name = 'Full Point Cloud';
            scene.add(pointsGroup);

            cameraPersp = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.1, 1000);
            cameraOrtho = new THREE.OrthographicCamera(-10, 10, 10, -10, 0.1, 1000);
            currentCamera = cameraPersp;
            controls = new OrbitControls(currentCamera, renderer.domElement);
            controls.addEventListener('change', () => {
                freezeRender = false;
                updateScaleBar();
            });

            setCloudBoundsFromMeta();
            setDefaultDisplayMode();
            buildGUI();
            setupMeasurementEvents();
            setupDragZones();
            setupViewButtons();
            setupFlyModeEvents();
            createMaterial();
            updateCloudDependentControls();
            setupCameras(cloudCenter, cloudRadius);
            initTooltips();

            window.addEventListener('resize', onWindowResize);
            animate();
            loadChunksProgressively();
        }

        function setCloudBoundsFromMeta() {
            const mn = META.bounds.min, mx = META.bounds.max;
            cloudBox.min.set(mn[0], mn[1], mn[2]);
            cloudBox.max.set(mx[0], mx[1], mx[2]);
            cloudCenter = cloudBox.getCenter(new THREE.Vector3());
            cloudRadius = Math.max(1.0, cloudBox.getBoundingSphere(cloudSphere).radius);
        }

        function setDefaultDisplayMode() {
            if (META.hasColor) applyDisplayModeSettings('RGB', false);
            else if (META.hasIntensity) applyDisplayModeSettings('Intensity', false);
            else applyDisplayModeSettings('Monochrome', false);
        }

        async function loadChunksConcurrently(chunks, options = {}) {
            const isPreview = !!options.preview;
            const total = chunks.length;
            let loadedCount = 0;

            const label = isPreview ? 'Loading preview LOD...' : 'Loading full point cloud...';
            showLoading(label, `0 / ${total} chunks`);
            updateStatsBadge();

            // Limit HTTP concurrent connections to 4 to prevent network congestion
            const limit = 4;
            let index = 0;

            async function worker() {
                while (index < total) {
                    const i = index++;
                    const chunk = chunks[i];

                    try {
                        const posBytes = await decodePayload(chunk, 'pos');
                        const colBytes = META.hasColor && hasChunkPayload(chunk, 'color') ? await decodePayload(chunk, 'color') : null;
                        const intBytes = META.hasIntensity && hasChunkPayload(chunk, 'intensity') ? await decodePayload(chunk, 'intensity') : null;

                        const positions = decodePositions(posBytes, chunk);
                        const geometry = new THREE.BufferGeometry();
                        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
                        geometry.userData.fullCount = chunk.count;
                        geometry.setDrawRange(0, chunk.count);

                        if (colBytes) {
                            const src = new Uint8Array(colBytes.buffer, colBytes.byteOffset, colBytes.byteLength);

                            let minR = 255, minG = 255, minB = 255, minA = 255;
                            let maxR = 0,   maxG = 0,   maxB = 0,   maxA = 0;

                            const count = Math.floor(src.length / 4);
                            const colors = new Float32Array(count * 4);

                            for (let i = 0, j = 0; i < src.length; i += 4, j += 4) {
                                const r = src[i];
                                const g = src[i + 1];
                                const b = src[i + 2];
                                const a = src[i + 3];

                                if (r < minR) minR = r; if (r > maxR) maxR = r;
                                if (g < minG) minG = g; if (g > maxG) maxG = g;
                                if (b < minB) minB = b; if (b > maxB) maxB = b;
                                if (a < minA) minA = a; if (a > maxA) maxA = a;

                                colors[j]     = r / 255.0;
                                colors[j + 1] = g / 255.0;
                                colors[j + 2] = b / 255.0;
                                colors[j + 3] = a / 255.0;
                            }

                            console.log('CHUNK COLOR RANGE', {
                                points: count,
                                bytes: src.length,
                                minR, maxR, minG, maxG, minB, maxB, minA, maxA,
                                firstRGBA: Array.from(src.slice(0, 16))
                            });

                            geometry.setAttribute('customColor', new THREE.BufferAttribute(colors, 4));
                        }

                        if (intBytes) {
                            let intensities;
                            let normalized = false;
                            if (META.intensityType === 'uint8_norm') {
                                intensities = new Uint8Array(intBytes.buffer, intBytes.byteOffset, intBytes.byteLength);
                                normalized = true;
                            } else if (META.intensityType === 'uint16_norm') {
                                intensities = new Uint16Array(intBytes.buffer, intBytes.byteOffset, intBytes.byteLength / 2);
                                normalized = true;
                            } else {
                                intensities = new Float32Array(intBytes.buffer, intBytes.byteOffset, intBytes.byteLength / 4);
                            }
                            geometry.setAttribute('intensity', new THREE.BufferAttribute(intensities, 1, normalized));
                        }

                        geometry.boundingBox = new THREE.Box3(
                            new THREE.Vector3(chunk.boundsMin[0], chunk.boundsMin[1], chunk.boundsMin[2]),
                            new THREE.Vector3(chunk.boundsMax[0], chunk.boundsMax[1], chunk.boundsMax[2])
                        );
                        geometry.computeBoundingSphere();

                        const pts = new THREE.Points(geometry, material);
                        pts.frustumCulled = true;
                        pts.userData.chunkIndex = i;
                        pts.userData.count = chunk.count;
                        pts.userData.globalStart = Number(chunk.globalStart || 0);
                        pts.userData.globalEnd = Number(chunk.globalEnd || (pts.userData.globalStart + chunk.count));
                        pts.userData.isPreview = isPreview;
                        pts.castShadow = true;

                        if (isPreview) {
                            previewGroup.add(pts);
                            previewPointChunks.push(pts);
                            previewPointChunks.sort((a, b) => a.userData.chunkIndex - b.userData.chunkIndex);
                            previewLoadedPointCount += chunk.count;
                        } else {
                            pointsGroup.add(pts);
                            pointChunks.push(pts);
                            pointChunks.sort((a, b) => a.userData.chunkIndex - b.userData.chunkIndex);
                            loadedPointCount += chunk.count;
                        }

                        loadedCount++;
                        showLoading(label, `Chunk ${loadedCount} / ${total}`);
                        applyDrawRange();
                        updateStatsBadge();
                    } catch (err) {
                        console.error(`Error loading chunk ${i}:`, err);
                        throw err;
                    }
                    await yieldToBrowser();
                }
            }

            const promises = [];
            const activeWorkers = Math.min(limit, total);
            for (let w = 0; w < activeWorkers; w++) {
                promises.push(worker());
            }
            await Promise.all(promises);
        }

        async function loadChunksProgressively() {
            const previewLOD = getPreviewLOD();
            if (previewLOD) {
                try {
                    await loadChunksConcurrently(previewLOD.chunks, { preview: true });
                    previewLoadingComplete = true;
                    updateScaleBar();
                } catch (e) {
                    console.warn('Preview LOD loading failed:', e);
                    previewLoadingComplete = false;
                }
            }

            try {
                await loadChunksConcurrently(META.chunks || [], { preview: false });
                loadingComplete = true;
                clearPreviewLOD();
                hideLoading();
                updateScaleBar();
                updateStatsBadge();
            } catch (e) {
                console.error(e);
                const extra = META.storage === 'external'
                    ? 'External chunks need the HTML file to be opened through a local web server, not directly as file://. Use the generated start_server file.'
                    : 'Check browser console for details.';
                const previewExtra = previewLoadingComplete ? '<br>Preview LOD remains visible.' : '';
                showLoading('Error loading full point cloud.', `${e.message}<br>${extra}${previewExtra}`);
            }
        }

        function clearPreviewLOD() {
            if (!previewGroup || previewPointChunks.length === 0) return;
            previewPointChunks.forEach(pts => {
                previewGroup.remove(pts);
                if (pts.geometry) pts.geometry.dispose();
            });
            previewPointChunks.length = 0;
            previewLoadedPointCount = 0;
            previewPointCount = 0;
            previewLoadingComplete = false;
        }

        function applyCameraAndNavMode() {
            const oldCam = currentCamera;
            currentCamera = params.camera === 'Perspective' ? cameraPersp : cameraOrtho;
            currentCamera.position.copy(oldCam.position);
            currentCamera.quaternion.copy(oldCam.quaternion);
            currentCamera.up.copy(oldCam.up);
            controls.object = currentCamera;

            if (params.navMode === 'Fly') {
                controls.enabled = false;
            } else {
                const forward = new THREE.Vector3(0, 0, -1).applyQuaternion(currentCamera.quaternion);
                controls.target.copy(currentCamera.position).add(forward.multiplyScalar(Math.max(20, cloudRadius)));
                if (params.measureMode) {
                    controls.enableRotate = false;
                    controls.enableZoom = false;
                } else {
                    controls.enableRotate = true;
                    controls.enableZoom = true;
                }
                controls.enabled = true;
            }
            controls.update();
            updateScaleBar();
        }

        // Keyboard and Mouse Event handling for Fly Mode
        function setupFlyModeEvents() {
            window.addEventListener('keydown', (e) => {
                if (params.navMode !== 'Fly') return;
                const key = e.key.toLowerCase();
                if (keys.hasOwnProperty(key)) keys[key] = true;
                if (e.key === 'Shift') keys.shift = true;
            });
            window.addEventListener('keyup', (e) => {
                const key = e.key.toLowerCase();
                if (keys.hasOwnProperty(key)) keys[key] = false;
                if (e.key === 'Shift') keys.shift = false;
            });
            renderer.domElement.addEventListener('pointerdown', (e) => {
                if (params.navMode === 'Fly' && !isMeasuring && e.button === 0 && !e.target.closest('#footer') && !e.target.closest('.lil-gui')) {
                    isFlyDragging = true;
                    prevMouse.x = e.clientX;
                    prevMouse.y = e.clientY;
                    renderer.domElement.setPointerCapture(e.pointerId);
                }
            });
            renderer.domElement.addEventListener('pointermove', (e) => {
                if (params.navMode === 'Fly' && isFlyDragging) {
                    const deltaX = e.clientX - prevMouse.x;
                    const deltaY = e.clientY - prevMouse.y;
                    prevMouse.x = e.clientX;
                    prevMouse.y = e.clientY;
                    const yawQ = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), -deltaX * 0.002);
                    currentCamera.quaternion.premultiply(yawQ);
                    const pitchQ = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -deltaY * 0.002);
                    currentCamera.quaternion.multiply(pitchQ);
                }
            });
            renderer.domElement.addEventListener('pointerup', (e) => {
                isFlyDragging = false;
                if (e.pointerId) renderer.domElement.releasePointerCapture(e.pointerId);
            });
            renderer.domElement.addEventListener('pointercancel', () => { isFlyDragging = false; });
        }

        function rotateCameraAxis(axis, angle) {
            const offset = new THREE.Vector3().subVectors(currentCamera.position, controls.target);
            offset.applyAxisAngle(axis, angle);
            currentCamera.position.copy(controls.target).add(offset);
            cameraPersp.up.applyAxisAngle(axis, angle);
            cameraOrtho.up.applyAxisAngle(axis, angle);
            controls.update();
            updateScaleBar();
        }

        // Level, tilt, and yaw drag zones setup
        function setupDragZones() {
            const createZone = (id, axisType, callback) => {
                const el = document.getElementById(id);
                let isDragging = false;
                let lastVal = 0;
                el.addEventListener('pointerdown', (e) => {
                    if (isMeasuring || e.button !== 0) return;
                    isDragging = true;
                    lastVal = axisType === 'x' ? e.clientX : e.clientY;
                    el.setPointerCapture(e.pointerId);
                    e.preventDefault();
                });
                el.addEventListener('pointermove', (e) => {
                    if (!isDragging) return;
                    const currentVal = axisType === 'x' ? e.clientX : e.clientY;
                    const delta = currentVal - lastVal;
                    lastVal = currentVal;
                    callback(delta);
                });
                el.addEventListener('pointerup', (e) => { isDragging = false; el.releasePointerCapture(e.pointerId); });
                el.addEventListener('pointercancel', (e) => { isDragging = false; el.releasePointerCapture(e.pointerId); });
            };
            createZone('rollZone', 'y', (delta) => {
                const angle = delta * 0.005;
                const viewDir = new THREE.Vector3().subVectors(controls.target, currentCamera.position).normalize();
                cameraPersp.up.applyAxisAngle(viewDir, angle);
                cameraOrtho.up.applyAxisAngle(viewDir, angle);
                controls.update();
                updateScaleBar();
            });
            createZone('pitchZone', 'y', (delta) => {
                const angle = delta * 0.005;
                const rightAxis = new THREE.Vector3().setFromMatrixColumn(currentCamera.matrixWorld, 0).normalize();
                rotateCameraAxis(rightAxis, angle);
            });
            createZone('yawZone', 'x', (delta) => {
                const angle = delta * 0.005;
                rotateCameraAxis(new THREE.Vector3(0, 0, 1), angle);
            });
        }

        function updateGUIFromParams() {
            for (const key in guiControllers) {
                if (guiControllers[key] && typeof guiControllers[key].updateDisplay === 'function') guiControllers[key].updateDisplay();
            }
        }

        function applyDisplayModeSettings(mode, updateGui = true) {
            if (mode === 'RGB' && !META.hasColor) mode = META.hasIntensity ? 'Intensity' : 'Monochrome';
            if (mode === 'Intensity' && !META.hasIntensity) mode = META.hasColor ? 'RGB' : 'Monochrome';
            params.displayMode = mode;

            if (mode === 'Monochrome') {
                params.opacity = defaultOpacity;
                params.brightness = defaultBrightness;
                params.globalIntensity = defaultGlobalIntensity;
                params.localIntensity = defaultLocalIntensity;
                params.blendMode = 'Normal';
                params.depthCull = true;
            } else if (mode === 'Intensity') {
                params.opacity = defaultOpacity;
                params.brightness = defaultBrightness;
                params.globalIntensity = defaultGlobalIntensity;
                params.localIntensity = defaultLocalIntensity;
                params.blendMode = 'Normal';
                params.depthCull = true;
            } else if (mode === 'RGB') {
                params.opacity = defaultOpacity;
                params.brightness = defaultBrightness;
                params.globalIntensity = defaultGlobalIntensity;
                params.localIntensity = defaultLocalIntensity;
                params.blendMode = 'Normal';
                params.depthCull = true;
            } else if (mode === 'NormalCol') {
                // NormalCol: zostawiamy aktualne wartości opacity/brightness/tint, tylko tryb koloru zmienia się na normal map
                params.blendMode = 'Normal';
                params.depthCull = true;
            }

            applyMaterialState();
            if (updateGui) updateGUIFromParams();
        }

        // Shadow floor based on global bounds, reused from standalone LasViewer
        let shadowFloorPlane = null;
        let shadowLightSource = null;

        function updateShadowFloor(geometry) {
            if (shadowFloorPlane) {
                scene.remove(shadowFloorPlane);
                shadowFloorPlane.geometry.dispose();
                shadowFloorPlane.material.dispose();
                shadowFloorPlane = null;
            }
            if (shadowLightSource) {
                scene.remove(shadowLightSource);
                shadowLightSource = null;
            }

            if (!params.showFloorShadows || !geometry) return;

            const box = geometry.boundingBox;
            const center = box.getCenter(new THREE.Vector3());
            const size = box.getSize(new THREE.Vector3());
            const radius = geometry.boundingSphere.radius;

            const floorZ = box.min.z - (radius * 0.005 + 0.02);
            const floorSize = Math.max(size.x, size.y) * 12.0;

            const planeGeo = new THREE.PlaneGeometry(floorSize, floorSize);
            const planeMat = new THREE.ShadowMaterial({ opacity: params.shadowOpacity });

            shadowFloorPlane = new THREE.Mesh(planeGeo, planeMat);
            shadowFloorPlane.position.set(center.x, center.y, floorZ);
            shadowFloorPlane.receiveShadow = true;
            scene.add(shadowFloorPlane);

            shadowLightSource = new THREE.DirectionalLight(0xffffff, 1.0);
            shadowLightSource.castShadow = true;

            const keyDirWorld = new THREE.Vector3(50, -50, 100).normalize();
            const lightDistance = radius * 5.0;
            shadowLightSource.position.copy(center).add(keyDirWorld.clone().multiplyScalar(lightDistance));
            shadowLightSource.target.position.copy(center);

            shadowLightSource.shadow.mapSize.width = 2048;
            shadowLightSource.shadow.mapSize.height = 2048;
            shadowLightSource.shadow.camera.near = radius * 0.1;
            shadowLightSource.shadow.camera.far = lightDistance * 2.5;

            const shadowFrustum = radius * 1.6;
            shadowLightSource.shadow.camera.left = -shadowFrustum;
            shadowLightSource.shadow.camera.right = shadowFrustum;
            shadowLightSource.shadow.camera.top = shadowFrustum;
            shadowLightSource.shadow.camera.bottom = -shadowFrustum;
            shadowLightSource.shadow.bias = -0.0002;

            scene.add(shadowLightSource);
            scene.add(shadowLightSource.target);
        }

        function updateCloudDependentControls() {
            const maxZ = cloudBox.max.z + 0.1;
            const minZ = cloudBox.min.z - 0.1;
            const sliderMin = Math.min(minZ, -10);
            const sliderMax = Math.max(maxZ, 10);
            params.clipGlobalMin = sliderMin;
            params.clipGlobalMax = sliderMax;
            naturalValues.clipGlobalMin = params.clipGlobalMin;
            naturalValues.clipGlobalMax = params.clipGlobalMax;

            if (guiControllers.clipGlobalMin) guiControllers.clipGlobalMin.min(sliderMin).max(sliderMax).step(0.01).setValue(params.clipGlobalMin);
            if (guiControllers.clipGlobalMax) guiControllers.clipGlobalMax.min(sliderMin).max(sliderMax).step(0.01).setValue(params.clipGlobalMax);
            if (guiControllers.clipFront) guiControllers.clipFront.max(cloudRadius * 2).step(0.01).setValue(0.1);
            if (guiControllers.clipBack) guiControllers.clipBack.max(cloudRadius * 4).step(0.01).setValue(cloudRadius * 2);

            const aspect = window.innerWidth / window.innerHeight;
            const maxRadius = Math.max(cloudRadius, 1.0);
            cameraPersp.near = maxRadius * 0.001;
            cameraPersp.far = maxRadius * 100;
            cameraPersp.updateProjectionMatrix();
            cameraOrtho.left = -cloudRadius * aspect;
            cameraOrtho.right = cloudRadius * aspect;
            cameraOrtho.top = cloudRadius;
            cameraOrtho.bottom = -cloudRadius;
            cameraOrtho.near = maxRadius * 0.001;
            cameraOrtho.far = maxRadius * 100;
            cameraOrtho.updateProjectionMatrix();
            applyMaterialState();
        }

        function gcdInt(a, b) {
            a = Math.abs(a);
            b = Math.abs(b);
            while (b) {
                const t = b;
                b = a % b;
                a = t;
            }
            return a || 1;
        }

        function limiterPattern(pointLimitPct) {
            const pct = Math.max(1, Math.min(100, Math.round(pointLimitPct)));
            if (pct >= 100) return { pct, period: 1, keep: [0] };
            const divisor = gcdInt(100, pct);
            const period = 100 / divisor;
            const keep = [];
            for (let r = 0; r < period; r++) {
                if (((r * pct) % 100) < pct) keep.push(r);
            }
            return { pct, period, keep };
        }

        function countPatternHits(startGlobal, endGlobal, pattern) {
            const length = Math.max(0, endGlobal - startGlobal);
            if (length <= 0) return 0;
            const fullCycles = Math.floor(length / pattern.period);
            let count = fullCycles * pattern.keep.length;
            const remainder = length % pattern.period;
            const base = ((startGlobal % pattern.period) + pattern.period) % pattern.period;
            for (let i = 0; i < remainder; i++) {
                const r = (base + i) % pattern.period;
                if (pattern.keep.includes(r)) count++;
            }
            return count;
        }

        function buildLimiterIndex(pts, localStart, localEnd, pattern) {
            const geometry = pts.geometry;
            const chunkStart = Number(pts.userData.globalStart || 0);
            const startGlobal = chunkStart + localStart;
            const endGlobal = chunkStart + localEnd;
            const cacheKey = `${localStart}:${localEnd}:${pattern.pct}`;

            if (pts.userData.limitIndexKey === cacheKey && geometry.index) {
                return geometry.index.count;
            }

            const indexCount = countPatternHits(startGlobal, endGlobal, pattern);
            const vertexCount = geometry.attributes.position.count;
            const IndexType = vertexCount > 65535 ? Uint32Array : Uint16Array;
            const indices = new IndexType(indexCount);
            let out = 0;
            const firstCycle = Math.floor(startGlobal / pattern.period) * pattern.period;

            for (let cycle = firstCycle; cycle < endGlobal; cycle += pattern.period) {
                for (let k = 0; k < pattern.keep.length; k++) {
                    const g = cycle + pattern.keep[k];
                    if (g >= startGlobal && g < endGlobal) {
                        indices[out++] = g - chunkStart;
                    }
                }
            }

            geometry.setIndex(new THREE.BufferAttribute(indices, 1));
            pts.userData.limitIndexKey = cacheKey;
            return indexCount;
        }

        function applyDrawRangeToChunks(chunks, totalCount) {
            if (!chunks || !chunks.length) return;
            totalCount = Math.max(0, Number(totalCount || sumChunkCountsFromObjects(chunks)));
            const pct = Math.max(1, Math.min(100, Math.round(params.pointLimitPct)));
            const minPct = Math.max(0, Math.min(100, params.pointLimitMinPct)) / 100.0;
            const maxPct = Math.max(0, Math.min(100, params.pointLimitMaxPct)) / 100.0;
            const cutStart = Math.floor(totalCount * minPct);
            const cutEnd = Math.floor(totalCount * maxPct);
            const pattern = limiterPattern(pct);

            chunks.forEach(pts => {
                const geometry = pts.geometry;
                const count = Number(pts.userData.count || geometry.userData.fullCount || 0);
                const chunkStart = Number(pts.userData.globalStart || 0);
                const chunkEnd = Number(pts.userData.globalEnd || (chunkStart + count));
                const visibleStartGlobal = Math.max(chunkStart, cutStart);
                const visibleEndGlobal = Math.min(chunkEnd, cutEnd);

                if (visibleEndGlobal <= visibleStartGlobal) {
                    pts.visible = false;
                    geometry.setDrawRange(0, 0);
                    return;
                }

                pts.visible = true;
                const localStart = visibleStartGlobal - chunkStart;
                const localEnd = visibleEndGlobal - chunkStart;

                if (pct >= 100) {
                    if (geometry.index) geometry.setIndex(null);
                    pts.userData.limitIndexKey = null;
                    geometry.setDrawRange(localStart, localEnd - localStart);
                } else {
                    const indexCount = buildLimiterIndex(pts, localStart, localEnd, pattern);
                    geometry.setDrawRange(0, indexCount);
                    if (indexCount <= 0) pts.visible = false;
                }
            });
        }

        function sumChunkCountsFromObjects(chunks) {
            return (chunks || []).reduce((acc, pts) => acc + Number(pts.userData.count || pts.geometry.userData.fullCount || 0), 0);
        }

        function applyDrawRange() {
            applyDrawRangeToChunks(pointChunks, totalPointCount);
            applyDrawRangeToChunks(previewPointChunks, previewPointCount);
        }

        // Slider constraints sync functions
        function linkedSliderMinChanged(minKey, maxKey, minWindow, lo, hi) {
            const diff = params[maxKey] - params[minKey];
            if (diff < minWindow) {
                const candidate = params[minKey] + minWindow;
                if (candidate <= hi) params[maxKey] = candidate;
                else params[minKey] = hi - minWindow;
            } else if (params[maxKey] > naturalValues[maxKey]) {
                const restored = Math.max(naturalValues[maxKey], params[minKey] + minWindow);
                if (restored < params[maxKey] && restored <= hi) params[maxKey] = restored;
            }
            if (params[minKey] < lo) params[minKey] = lo;
            if (params[maxKey] > hi) params[maxKey] = hi;
            updateGUIFromParams();
        }

        function linkedSliderMaxChanged(minKey, maxKey, minWindow, lo, hi) {
            const diff = params[maxKey] - params[minKey];
            if (diff < minWindow) {
                const candidate = params[maxKey] - minWindow;
                if (candidate >= lo) params[minKey] = candidate;
                else params[maxKey] = lo + minWindow;
            } else if (params[minKey] < naturalValues[minKey]) {
                const restored = Math.min(naturalValues[minKey], params[maxKey] - minWindow);
                if (restored > params[minKey] && restored >= lo) params[minKey] = restored;
            }
            if (params[minKey] < lo) params[minKey] = lo;
            if (params[maxKey] > hi) params[maxKey] = hi;
            updateGUIFromParams();
        }

        // Setup Blender cameras in the dropdown select list
        function setupCameras(center, radius) {
            cameraList = [];
            cameraList.push({
                name: 'Isometric (Default)', isSpecial: true,
                setup: () => {
                    params.camera = 'Orthographic';
                    params.navMode = 'Orbit';
                    currentViewModeIdx = 0;
                    document.getElementById('viewModeBtn').innerText = viewModes[currentViewModeIdx];
                    applyCameraAndNavMode();
                    currentCamera.position.copy(center).add(new THREE.Vector3(0.7 * radius * 1.5, -0.7 * radius * 1.5, 0.7 * radius * 1.5));
                    currentCamera.up.set(0, 0, 1);
                    currentCamera.lookAt(center);
                    controls.target.copy(center);
                    applyLocalClipValues(0.1, radius * 4, false);
                }
            });

            if (window.CAMERAS_JSON) {
                window.CAMERAS_JSON.forEach(cam => cameraList.push({ name: cam.name, isSpecial: false, data: cam }));
            }

            const select = document.getElementById('camSelect');
            select.innerHTML = '';
            cameraList.forEach((c, i) => {
                const opt = document.createElement('option');
                opt.value = i;
                opt.innerText = c.name;
                select.appendChild(opt);
            });
            select.addEventListener('change', (e) => applyCamera(parseInt(e.target.value)));
            document.getElementById('prevCam').addEventListener('click', () => switchCamera(-1));
            document.getElementById('nextCam').addEventListener('click', () => switchCamera(1));
            window.addEventListener('keydown', (e) => {
                if (e.key === 'ArrowLeft') { e.preventDefault(); switchCamera(-1); }
                else if (e.key === 'ArrowRight') { e.preventDefault(); switchCamera(1); }
            });
            applyCamera(0);
        }

        function switchCamera(dir) {
            if (cameraList.length === 0) return;
            currentCamIndex = (currentCamIndex + dir + cameraList.length) % cameraList.length;
            document.getElementById('camSelect').value = currentCamIndex;
            applyCamera(currentCamIndex);
        }

        function applyCamera(index) {
            currentCamIndex = index;
            const cam = cameraList[index];
            if (!cam) return;

            if (cam.isSpecial) {
                cam.setup();
            } else {
                const data = cam.data;
                const mat = new THREE.Matrix4().fromArray(data.matrix);
                const pos = new THREE.Vector3();
                const quat = new THREE.Quaternion();
                const scale = new THREE.Vector3();
                mat.decompose(pos, quat, scale);

                const isOrtho = data.type === 'ORTHO';
                params.camera = isOrtho ? 'Orthographic' : 'Perspective';
                params.navMode = 'Orbit';
                currentViewModeIdx = isOrtho ? 0 : 1;
                document.getElementById('viewModeBtn').innerText = viewModes[currentViewModeIdx];
                applyCameraAndNavMode();

                currentCamera.position.copy(pos);
                currentCamera.quaternion.copy(quat);
                currentCamera.up.set(0, 0, 1);

                if (isOrtho) {
                    const aspect = window.innerWidth / window.innerHeight;
                    const s = data.ortho_scale / 2;
                    cameraOrtho.left = -s * aspect;
                    cameraOrtho.right = s * aspect;
                    cameraOrtho.top = s;
                    cameraOrtho.bottom = -s;
                    cameraOrtho.updateProjectionMatrix();
                } else {
                    cameraPersp.fov = THREE.MathUtils.radToDeg(data.fov);
                    cameraPersp.updateProjectionMatrix();
                }

                applyLocalClipValues(data.clip_start, data.clip_end, true);
                const forward = new THREE.Vector3(0, 0, -1).applyQuaternion(quat);
                const dist = (params.clipFront + params.clipBack) / 2.0;
                controls.target.copy(pos).add(forward.multiplyScalar(dist));
            }
            if (params.measureMode) {
                controls.enableRotate = false;
                controls.enableZoom = false;
            }
            controls.enabled = params.navMode !== 'Fly' && !params.measureMode;
            controls.update();
            updateScaleBar();
        }

        function applyLocalClipValues(front, back, enabled = true) {
            let safeFront = Number(front);
            let safeBack = Number(back);

            if (!Number.isFinite(safeFront)) safeFront = 0.1;
            if (!Number.isFinite(safeBack)) safeBack = Math.max(safeFront + 0.3, cloudRadius * 2);

            safeFront = Math.max(0.0001, safeFront);
            safeBack = Math.max(safeFront + 0.001, safeBack);

            params.clipFront = safeFront;
            params.clipBack = safeBack;
            params.cutLocal = !!enabled;

            naturalValues.clipFront = params.clipFront;
            naturalValues.clipBack = params.clipBack;

            if (guiControllers.clipFront) {
                if (params.clipFront < guiControllers.clipFront._min) {
                    guiControllers.clipFront.min(Math.max(0.0001, params.clipFront * 0.5));
                }
                if (params.clipFront > guiControllers.clipFront._max) {
                    guiControllers.clipFront.max(params.clipFront * 2);
                }
                guiControllers.clipFront.updateDisplay();
            }

            if (guiControllers.clipBack) {
                if (params.clipBack < guiControllers.clipBack._min) {
                    guiControllers.clipBack.min(Math.max(params.clipFront + 0.001, params.clipBack * 0.5));
                }
                if (params.clipBack > guiControllers.clipBack._max) {
                    guiControllers.clipBack.max(params.clipBack * 2);
                }
                guiControllers.clipBack.updateDisplay();
            }

            if (guiControllers.cutLocal) {
                guiControllers.cutLocal.updateDisplay();
            }

            if (currentCamera) {
                currentCamera.near = params.clipFront;
                currentCamera.far = params.clipBack;
                currentCamera.updateProjectionMatrix();
            }

            if (material) {
                material.uniforms.uClipStart.value = params.clipFront;
                material.uniforms.uClipEnd.value = params.clipBack;
                material.uniforms.uCutLocal.value = params.cutLocal ? 1.0 : 0.0;
                material.uniforms.uDepthFullStart.value = params.clipFront;
                material.uniforms.uDepthFullEnd.value = params.clipBack;
            }
        }

        // Jednorazowe przerysowanie warstwowe – QRedraw
        // Renderuje kolejne plasterki głębokości od najdalszego do najbliższego,
        // przy WYŁĄCZONYM depthTest/depthWrite (warstwy mieszają się przez blending zamiast klasycznego Z-bufora).
        function qRedraw() {
            if (!material || !currentCamera) return;

            freezeRender = true;

            const prevAutoClear = renderer.autoClear;
            const prevDepthTest = material.depthTest;
            const prevDepthWrite = material.depthWrite;
            const prevTransparent = material.transparent;
            const prevBlending = material.blending;

            const prevClipStart = params.clipFront;
            const prevClipEnd = params.clipBack;
            const prevCutLocal = params.cutLocal;

            material.transparent = true;
            material.depthTest = false;
            material.depthWrite = false;
            material.blending = THREE.NormalBlending;
            material.needsUpdate = true;

            renderer.autoClear = false;

            const range = prevClipEnd - prevClipStart;
            if (range <= 0.0001) {
                renderer.autoClear = prevAutoClear;
                material.depthTest = prevDepthTest;
                material.depthWrite = prevDepthWrite;
                material.transparent = prevTransparent;
                material.blending = prevBlending;
                material.needsUpdate = true;
                return;
            }

            const sliceStep = 0.2;
            const maxSlices = 512;
            const slices = Math.min(maxSlices, Math.ceil(range / sliceStep));

            material.uniforms.uDepthFullStart.value = prevClipStart;
            material.uniforms.uDepthFullEnd.value = prevClipEnd;

            // Render from far to near without depth test, only blending
            for (let i = slices - 1; i >= 0; i--) {
                const sliceStart = prevClipStart + i * sliceStep;
                const sliceEnd = Math.min(prevClipStart + (i + 1) * sliceStep, prevClipEnd);

                material.uniforms.uClipStart.value = sliceStart;
                material.uniforms.uClipEnd.value = sliceEnd;
                material.uniforms.uCutLocal.value = 1.0;

                renderer.render(scene, currentCamera);
            }

            params.clipFront = prevClipStart;
            params.clipBack = prevClipEnd;
            params.cutLocal = prevCutLocal;

            material.uniforms.uClipStart.value = params.clipFront;
            material.uniforms.uClipEnd.value = params.clipBack;
            material.uniforms.uCutLocal.value = params.cutLocal ? 1.0 : 0.0;

            material.depthTest = prevDepthTest;
            material.depthWrite = prevDepthWrite;
            material.transparent = prevTransparent;
            material.blending = prevBlending;
            material.needsUpdate = true;

            renderer.autoClear = prevAutoClear;
        }

        // GUI Control layout definition using lil-gui
        function buildGUI() {
            const gui = new GUI({ title: 'LiDAR Options' });
            const ptFolder = gui.addFolder('Point Style');
            ptFolder.add(params, 'pointSize', 0.01, 10.0).step(0.01).name('Point Size').onChange(v => {
                if (material) material.uniforms.uPointSize.value = v;
            });
            guiControllers.opacity = ptFolder.add(params, 'opacity', 0.0, 1.0).step(0.01).name('Opacity').onChange(v => { if (material) material.uniforms.uOpacity.value = v; });
            guiControllers.brightness = ptFolder.add(params, 'brightness', -1.0, 1.0).step(0.01).name('Brightness').onChange(v => { if (material) material.uniforms.uBrightness.value = v; });
            ptFolder.add(params, 'aoStrength', 0.0, 1.0).step(0.01).name('AO Simulation').onChange(v => { if (material) material.uniforms.uAOStrength.value = v; });
            ptFolder.addColor(params, 'bgColor').name('Background Color').onChange(v => { renderer.setClearColor(v); updateScaleBar(); });
            guiControllers.blendMode = ptFolder.add(params, 'blendMode', ['Normal', 'Additive', 'Subtractive']).name('Blend Mode').onChange(() => applyMaterialState());
            guiControllers.depthCull = ptFolder.add(params, 'depthCull').name('Depth Culling').onChange(() => applyMaterialState());
            ptFolder.add(params, 'jitterEnabled').name('Anti-Moire (Jitter)').onChange(() => applyMaterialState());
            ptFolder.add(params, 'jitterAmount', 0.0, 0.1).step(0.001).name('Jitter Strength').onChange(() => applyMaterialState());

            // Lighting & Shadows
            const lightingFolder = gui.addFolder('💡 Lighting & Shadows');
            lightingFolder.add(params, 'lightType', ['3-Point Rig', 'Camera Omni']).name('Light Source').onChange(v => {
                if (material) material.uniforms.uLightType.value = v === '3-Point Rig' ? 0.0 : 1.0;
            });
            // Floor shadow controls left in params & updateShadowFloor, but GUI toggles are optional
            // to avoid accidental heavy shadow rendering on huge point clouds.

            const navFolder = gui.addFolder('Camera Navigation');
            navFolder.add(params, 'flySpeed', 1.0, 100.0).step(1.0).name('Fly Speed');

            const displayModes = [];
            if (META.hasColor) displayModes.push('RGB');
            if (META.hasIntensity) displayModes.push('Intensity');
            displayModes.push('Monochrome');
            displayModes.push('NormalCol');
            guiControllers.displayMode = gui.add(params, 'displayMode', displayModes).name('Display Mode').onChange(v => applyDisplayModeSettings(v));

            const measFolder = gui.addFolder('Measurement (on camera plane)');
            measFolder.add(params, 'measureMode').name('Enable Measuring').onChange(v => {
                isMeasuring = v;
                renderer.domElement.style.cursor = v ? 'crosshair' : 'default';
                if (v) {
                    controls.enableRotate = false;
                    controls.enableZoom = false;
                } else {
                    controls.enableRotate = true;
                    controls.enableZoom = true;
                    if (measureState === 1 && currentMeasure) {
                        scene.remove(currentMeasure.line);
                        currentMeasure.div.remove();
                        measurements.pop();
                        measureState = 0;
                        currentMeasure = null;
                    }
                }
                controls.enabled = params.navMode !== 'Fly';
            });
            measFolder.addColor(params, 'measureColor').name('Line Color').onChange(v => {
                measurements.forEach(m => { m.line.material.color.set(v); m.div.style.color = v; });
            });
            measFolder.add(params, 'clearMeasurements').name('Clear All');

            const plFolder = gui.addFolder('Point Limiter (%)');
            guiControllers.pointLimitPct = plFolder.add(params, 'pointLimitPct', 1, 100).step(1).name('% Point Limiter').onChange(() => applyDrawRange());
            guiControllers.pointLimitMinPct = plFolder.add(params, 'pointLimitMinPct', 0, 100).step(1).name('Start Cutoff %').onChange(v => {
                naturalValues.pointLimitMinPct = v;
                linkedSliderMinChanged('pointLimitMinPct', 'pointLimitMaxPct', params.pointLimitWindowPct, 0, 100);
                applyDrawRange();
            });
            guiControllers.pointLimitMaxPct = plFolder.add(params, 'pointLimitMaxPct', 0, 100).step(1).name('End Cutoff %').onChange(v => {
                naturalValues.pointLimitMaxPct = v;
                linkedSliderMaxChanged('pointLimitMinPct', 'pointLimitMaxPct', params.pointLimitWindowPct, 0, 100);
                applyDrawRange();
            });

            const glFolder = gui.addFolder('Global Clipping (Z Elevation)');
            guiControllers.clipGlobalMax = glFolder.add(params, 'clipGlobalMax', -10, 10).name('Z Top').onChange(v => {
                naturalValues.clipGlobalMax = v;
                linkedSliderMaxChanged('clipGlobalMin', 'clipGlobalMax', 0.3, guiControllers.clipGlobalMax._min, guiControllers.clipGlobalMax._max);
                applyMaterialState();
            });
            guiControllers.clipGlobalMin = glFolder.add(params, 'clipGlobalMin', -10, 10).name('Z Bottom').onChange(v => {
                naturalValues.clipGlobalMin = v;
                linkedSliderMinChanged('clipGlobalMin', 'clipGlobalMax', 0.3, guiControllers.clipGlobalMin._min, guiControllers.clipGlobalMin._max);
                applyMaterialState();
            });
            glFolder.add(params, 'cutGlobal').name('Apply Global Clip').onChange(() => applyMaterialState());
            guiControllers.globalIntensity = glFolder.add(params, 'globalIntensity', 0.0, 1.0).name('Elevation Tint').onChange(() => applyMaterialState());

            const locFolder = gui.addFolder('Local Clipping (Depth)');
            guiControllers.clipFront = locFolder.add(params, 'clipFront', 0.01, 100).name('Clip Front').onChange(v => {
                naturalValues.clipFront = v;
                linkedSliderMinChanged('clipFront', 'clipBack', 0.3, guiControllers.clipFront._min, guiControllers.clipFront._max);
                if (material) {
                    material.uniforms.uClipStart.value = params.clipFront;
                    material.uniforms.uClipEnd.value = params.clipBack;
                    material.uniforms.uDepthFullStart.value = params.clipFront;
                    material.uniforms.uDepthFullEnd.value = params.clipBack;
                }
            });
            guiControllers.clipBack = locFolder.add(params, 'clipBack', 0.1, 100).name('Clip Back').onChange(v => {
                naturalValues.clipBack = v;
                linkedSliderMaxChanged('clipFront', 'clipBack', 0.3, guiControllers.clipBack._min, guiControllers.clipBack._max);
                if (material) {
                    material.uniforms.uClipEnd.value = params.clipBack;
                    material.uniforms.uClipStart.value = params.clipFront;
                    material.uniforms.uDepthFullStart.value = params.clipFront;
                    material.uniforms.uDepthFullEnd.value = params.clipBack;
                }
            });
            guiControllers.cutLocal = locFolder.add(params, 'cutLocal').name('Apply Local Clip').onChange(() => applyMaterialState());
            guiControllers.localIntensity = locFolder.add(params, 'localIntensity', 0.0, 1.0).name('Depth Tint').onChange(() => applyMaterialState());

            const exportFolder = gui.addFolder('Image Export');
            exportFolder.add(params, 'captureRes', 1, 10, 1).name('Resolution Multiplier');
            exportFolder.add(params, 'takeScreenshot').name('Save JPG Screenshot');
            exportFolder.add({ qredraw: () => { qRedraw(); } }, 'qredraw').name('QRedraw');
        }

        function clearMeasurements() {
            measurements.forEach(m => {
                scene.remove(m.line);
                if (m.div && m.div.parentNode) m.div.parentNode.removeChild(m.div);
            });
            measurements.length = 0;
            measureState = 0;
            currentMeasure = null;
        }

        function getFocalPlaneIntersection() {
            const planeNormal = new THREE.Vector3(0, 0, 1).applyQuaternion(currentCamera.quaternion);
            const plane = new THREE.Plane().setFromNormalAndCoplanarPoint(planeNormal, controls.target);
            raycaster.setFromCamera(mouse, currentCamera);
            const target = new THREE.Vector3();
            if (raycaster.ray.intersectPlane(plane, target)) return target;
            return null;
        }

        // Raycasting and drawing measuring lines
        function setupMeasurementEvents() {
            window.addEventListener('pointerdown', (e) => {
                if (!isMeasuring || e.button !== 0 || e.target.closest('#footer') || e.target.closest('.lil-gui') || e.target.classList.contains('drag-zone')) return;
                mouse.x = (e.clientX / window.innerWidth) * 2 - 1;
                mouse.y = -(e.clientY / window.innerHeight) * 2 + 1;
                const p = getFocalPlaneIntersection();
                if (!p) return;

                if (measureState === 0) {
                    const geo = new THREE.BufferGeometry().setFromPoints([p, p.clone()]);
                    const mat = new THREE.LineBasicMaterial({ color: params.measureColor, depthTest: false, transparent: true });
                    const line = new THREE.Line(geo, mat);
                    line.renderOrder = 999;
                    scene.add(line);
                    const div = document.createElement('div');
                    Object.assign(div.style, {
                        position: 'absolute', color: params.measureColor, background: 'rgba(0,0,0,0.7)', padding: '2px 6px', borderRadius: '4px', fontFamily: 'sans-serif', fontSize: '14px',
                        fontWeight: 'bold', pointerEvents: 'none', transform: 'translate(-50%, -50%)', zIndex: '1000'
                    });
                    document.body.appendChild(div);
                    currentMeasure = { p1: p, p2: p.clone(), line, div };
                    measurements.push(currentMeasure);
                    measureState = 1;
                } else if (measureState === 1) {
                    currentMeasure.p2.copy(p);
                    updateMeasureGeometry(currentMeasure);
                    measureState = 0;
                    currentMeasure = null;
                }
            });
            window.addEventListener('pointermove', (e) => {
                if (!isMeasuring || measureState === 0) return;
                mouse.x = (e.clientX / window.innerWidth) * 2 - 1;
                mouse.y = -(e.clientY / window.innerHeight) * 2 + 1;
                const p = getFocalPlaneIntersection();
                if (p && currentMeasure) {
                    currentMeasure.p2.copy(p);
                    updateMeasureGeometry(currentMeasure);
                }
            });
        }

        function updateMeasureGeometry(m) {
            m.line.geometry.attributes.position.setXYZ(1, m.p2.x, m.p2.y, m.p2.z);
            m.line.geometry.attributes.position.needsUpdate = true;
            m.div.innerText = m.p1.distanceTo(m.p2).toFixed(3) + ' m';
        }

        function recomputeCloudBoundsFromChunks() {
            cloudBox.makeEmpty();
            pointChunks.forEach(pts => {
                pts.geometry.computeBoundingBox();
                pts.geometry.computeBoundingSphere();
                cloudBox.union(pts.geometry.boundingBox);
            });
            cloudCenter = cloudBox.getCenter(new THREE.Vector3());
            cloudRadius = Math.max(1.0, cloudBox.getBoundingSphere(cloudSphere).radius);
            updateCloudDependentControls();
        }

        // Store view and direct camera control action buttons
        function setupViewButtons() {
            const setView = (ox, oy, oz) => {
                if (!pointsGroup) return;
                if (params.camera !== 'Orthographic') {
                    currentViewModeIdx = 0;
                    document.getElementById('viewModeBtn').innerText = viewModes[currentViewModeIdx];
                    params.camera = 'Orthographic';
                    params.navMode = 'Orbit';
                    applyCameraAndNavMode();
                }
                const center = cloudCenter;
                const r = cloudRadius;
                currentCamera.position.copy(center).add(new THREE.Vector3(ox * r * 1.5, oy * r * 1.5, oz * r * 1.5));
                if (ox === 0 && oy === 0 && oz === 1) currentCamera.up.set(0, 1, 0);
                else currentCamera.up.set(0, 0, 1);
                currentCamera.lookAt(center);
                controls.target.copy(center);
                controls.update();
                updateScaleBar();
            };

            document.getElementById('viewTop').onclick = () => setView(0, 0, 1);
            document.getElementById('viewFront').onclick = () => setView(0, -1, 0);

            document.getElementById('storeView').onclick = () => {
                if (!pointsGroup || !pointChunks.length || !loadingComplete) return;
                const transMat = new THREE.Matrix4().makeTranslation(-controls.target.x, -controls.target.y, -controls.target.z);
                const rotMat = new THREE.Matrix4().makeRotationFromQuaternion(currentCamera.quaternion.clone().invert());
                pointChunks.forEach(pts => {
                    pts.geometry.applyMatrix4(transMat);
                    pts.geometry.applyMatrix4(rotMat);
                    pts.geometry.attributes.position.needsUpdate = true;
                    pts.geometry.computeBoundingBox();
                    pts.geometry.computeBoundingSphere();
                });
                clearMeasurements();
                recomputeCloudBoundsFromChunks();
                document.getElementById('viewTop').onclick();
            };

            document.getElementById('viewModeBtn').onclick = (e) => {
                currentViewModeIdx = (currentViewModeIdx + 1) % 3;
                const mode = viewModes[currentViewModeIdx];
                e.target.innerText = mode;
                if (mode === 'Orthographic') { params.camera = 'Orthographic'; params.navMode = 'Orbit'; }
                else if (mode === 'Perspective') { params.camera = 'Perspective'; params.navMode = 'Orbit'; }
                else if (mode === 'FlyMode') { params.camera = 'Perspective'; params.navMode = 'Fly'; }
                applyCameraAndNavMode();
            };

            document.getElementById('rotXPlus').onclick = () => {
                const rightAxis = new THREE.Vector3().setFromMatrixColumn(currentCamera.matrixWorld, 0).normalize();
                rotateCameraAxis(rightAxis, Math.PI / 2);
            };
            document.getElementById('rotXMinus').onclick = () => {
                const rightAxis = new THREE.Vector3().setFromMatrixColumn(currentCamera.matrixWorld, 0).normalize();
                rotateCameraAxis(rightAxis, -Math.PI / 2);
            };
            document.getElementById('rotZPlus').onclick = () => rotateCameraAxis(new THREE.Vector3(0, 0, 1), Math.PI / 2);
            document.getElementById('rotZMinus').onclick = () => rotateCameraAxis(new THREE.Vector3(0, 0, 1), -Math.PI / 2);
        }

        function getNegativeColor(hex) {
            if (hex.indexOf('#') === 0) hex = hex.slice(1);
            if (hex.length === 3) hex = hex[0]+hex[0]+hex[1]+hex[1]+hex[2]+hex[2];
            const r = (255 - parseInt(hex.slice(0, 2), 16)).toString(16).padStart(2, '0');
            const g = (255 - parseInt(hex.slice(2, 4), 16)).toString(16).padStart(2, '0');
            const b = (255 - parseInt(hex.slice(4, 6), 16)).toString(16).padStart(2, '0');
            return '#' + r + g + b;
        }

        function updateScaleBar() {
            const container = document.getElementById('scaleBarContainer');
            if (params.camera !== 'Orthographic' || !pointsGroup) { container.style.display = 'none'; return; }
            container.style.display = 'block';
            const visibleWorldWidth = (cameraOrtho.right - cameraOrtho.left) / cameraOrtho.zoom;
            const targetWidth = visibleWorldWidth * 0.50;
            const magnitude = Math.pow(10, Math.floor(Math.log10(Math.max(targetWidth, 0.0001))));
            let niceNumber = magnitude;
            if (targetWidth >= 5 * magnitude) niceNumber = 5 * magnitude;
            else if (targetWidth >= 2 * magnitude) niceNumber = 2 * magnitude;
            container.style.width = ((niceNumber / visibleWorldWidth) * 100) + '%';
            document.getElementById('scaleBarText').innerText = `${niceNumber} m`;
            const negColor = getNegativeColor(params.bgColor);
            document.getElementById('scaleBarText').style.color = negColor;
            document.getElementById('scaleBarLine').style.borderColor = negColor;
        }

        // Tiled high resolution screenshot render method
        function takeScreenshot() {
            if (!material) return;
            const mult = parseInt(params.captureRes);
            const w = window.innerWidth;
            const h = window.innerHeight;
            let targetW = Math.floor(w * mult);
            let targetH = Math.floor(h * mult);
            const MAX_OUTPUT = 20000;
            if (targetW > MAX_OUTPUT || targetH > MAX_OUTPUT) {
                const s = MAX_OUTPUT / Math.max(targetW, targetH);
                targetW = Math.floor(targetW * s);
                targetH = Math.floor(targetH * s);
            }

            const isOrtho = params.camera === 'Orthographic';
            const needsTiling = mult >= 3;
            const origPixelRatio = renderer.getPixelRatio();
            const savedAspect = currentCamera.aspect || (w / h);
            const orthoSaved = { left: cameraOrtho.left, right: cameraOrtho.right, top: cameraOrtho.top, bottom: cameraOrtho.bottom };

            renderer.setPixelRatio(1.0);
            material.uniforms.uPixelRatio.value = 1.0;
            const outputCanvas = document.createElement('canvas');
            outputCanvas.width = targetW;
            outputCanvas.height = targetH;
            const outCtx = outputCanvas.getContext('2d');
            outCtx.fillStyle = params.bgColor;
            outCtx.fillRect(0, 0, targetW, targetH);

            if (!needsTiling) {
                if (isOrtho) {
                    const orthoSize = (orthoSaved.top - orthoSaved.bottom) / 2;
                    const aspect = targetW / targetH;
                    cameraOrtho.left = -orthoSize * aspect;
                    cameraOrtho.right = orthoSize * aspect;
                    cameraOrtho.updateProjectionMatrix();
                } else {
                    cameraPersp.aspect = targetW / targetH;
                    cameraPersp.updateProjectionMatrix();
                }
                renderer.setSize(targetW, targetH, false);
                renderer.render(scene, currentCamera);
                outCtx.drawImage(renderer.domElement, 0, 0);
            } else {
                const TILE_SIZE = 2048;
                const cols = Math.ceil(targetW / TILE_SIZE);
                const rows = Math.ceil(targetH / TILE_SIZE);
                const gl = renderer.getContext();
                for (let row = 0; row < rows; row++) {
                    for (let col = 0; col < cols; col++) {
                        const x0 = (col / cols) * targetW;
                        const x1 = ((col + 1) / cols) * targetW;
                        const y0 = (row / rows) * targetH;
                        const y1 = ((row + 1) / rows) * targetH;
                        const tw = Math.round(Math.min(x1 - x0, TILE_SIZE));
                        const th = Math.round(Math.min(y1 - y0, TILE_SIZE));
                        const destX = Math.round(x0);
                        const destY = Math.round(y0);
                        currentCamera.setViewOffset(targetW, targetH, destX, destY, tw, th);
                        currentCamera.updateProjectionMatrix();
                        const rt = new THREE.WebGLRenderTarget(tw, th, { format: THREE.RGBAFormat, type: THREE.UnsignedByteType, depthBuffer: true, stencilBuffer: false, samples: 0 });
                        renderer.setRenderTarget(rt);
                        renderer.render(scene, currentCamera);
                        const pixels = new Uint8Array(tw * th * 4);
                        gl.readPixels(0, 0, tw, th, gl.RGBA, gl.UNSIGNED_BYTE, pixels);
                        renderer.setRenderTarget(null);
                        rt.dispose();
                        const imageData = outCtx.createImageData(tw, th);
                        const rowBytes = tw * 4;
                        for (let y = 0; y < th; y++) {
                            const srcStart = (th - 1 - y) * rowBytes;
                            imageData.data.set(pixels.subarray(srcStart, srcStart + rowBytes), y * rowBytes);
                        }
                        outCtx.putImageData(imageData, destX, destY);
                    }
                }
                currentCamera.clearViewOffset();
                currentCamera.updateProjectionMatrix();
            }

            renderer.setPixelRatio(origPixelRatio);
            renderer.setSize(w, h, false);
            material.uniforms.uPixelRatio.value = window.devicePixelRatio;
            if (isOrtho) {
                cameraOrtho.left = orthoSaved.left;
                cameraOrtho.right = orthoSaved.right;
                cameraOrtho.top = orthoSaved.top;
                cameraOrtho.bottom = orthoSaved.bottom;
                cameraOrtho.updateProjectionMatrix();
            } else {
                cameraPersp.aspect = savedAspect;
                cameraPersp.updateProjectionMatrix();
            }

            // Draw measurements on the screenshot canvas
            measurements.forEach(m => {
                const p1 = m.p1.clone().project(currentCamera);
                const p2 = m.p2.clone().project(currentCamera);
                if (p1.z > 1.0 || p2.z > 1.0) return;
                const x1 = (p1.x * 0.5 + 0.5) * targetW;
                const y1 = (-p1.y * 0.5 + 0.5) * targetH;
                const x2 = (p2.x * 0.5 + 0.5) * targetW;
                const y2 = (-p2.y * 0.5 + 0.5) * targetH;
                outCtx.strokeStyle = params.measureColor;
                outCtx.lineWidth = 2;
                outCtx.beginPath(); outCtx.moveTo(x1, y1); outCtx.lineTo(x2, y2); outCtx.stroke();
                outCtx.fillStyle = params.measureColor;
                outCtx.beginPath(); outCtx.arc(x1, y1, 4, 0, Math.PI * 2); outCtx.fill();
                outCtx.beginPath(); outCtx.arc(x2, y2, 4, 0, Math.PI * 2); outCtx.fill();
                const dist = m.p1.distanceTo(m.p2).toFixed(3) + ' m';
                const midX = (x1 + x2) / 2, midY = (y1 + y2) / 2;
                outCtx.font = 'bold 16px sans-serif';
                outCtx.textAlign = 'center';
                outCtx.textBaseline = 'middle';
                const mText = outCtx.measureText(dist);
                outCtx.fillStyle = 'rgba(0,0,0,0.7)';
                outCtx.fillRect(midX - mText.width / 2 - 6, midY - 14, mText.width + 12, 28);
                outCtx.fillStyle = params.measureColor;
                outCtx.fillText(dist, midX, midY);
            });

            // Draw ortho scale bar on screenshot canvas
            if (isOrtho) {
                const visibleWorldWidth = (cameraOrtho.right - cameraOrtho.left) / cameraOrtho.zoom;
                const targetSW = visibleWorldWidth * 0.50;
                const mag = Math.pow(10, Math.floor(Math.log10(Math.max(targetSW, 0.0001))));
                const niceNumber = targetSW >= 5 * mag ? 5 * mag : (targetSW >= 2 * mag ? 2 * mag : mag);
                const barWidthPixels = (niceNumber / visibleWorldWidth) * targetW;
                const barX = targetW / 2 - barWidthPixels / 2;
                const barY = targetH - 10;
                const negColor = getNegativeColor(params.bgColor);
                outCtx.strokeStyle = negColor;
                outCtx.lineWidth = 2;
                outCtx.beginPath(); outCtx.moveTo(barX, barY - 10); outCtx.lineTo(barX, barY); outCtx.lineTo(barX + barWidthPixels, barY); outCtx.lineTo(barX + barWidthPixels, barY - 10); outCtx.stroke();
                outCtx.fillStyle = negColor;
                outCtx.font = 'bold 16px sans-serif';
                outCtx.textAlign = 'center';
                outCtx.fillText(`${niceNumber} m`, barX + barWidthPixels / 2, barY - 15);
            }

            let finalName = FILE_NAME + (isOrtho ? '_Ortho' : '_Persp');
            finalName += `_x${mult}`;
            if (isOrtho) {
                const vw = (cameraOrtho.right - cameraOrtho.left) / cameraOrtho.zoom;
                finalName += `_${vw.toFixed(2).replace('.', ',')}m`;
            }
            const link = document.createElement('a');
            link.download = `${finalName}.jpg`;
            link.href = outputCanvas.toDataURL('image/jpeg', 0.95);
            link.click();
        }

        function onWindowResize() {
            const aspect = window.innerWidth / window.innerHeight;
            cameraPersp.aspect = aspect;
            cameraPersp.updateProjectionMatrix();
            const orthoSize = (cameraOrtho.top - cameraOrtho.bottom) / 2;
            cameraOrtho.left = -orthoSize * aspect;
            cameraOrtho.right = orthoSize * aspect;
            cameraOrtho.updateProjectionMatrix();
            renderer.setSize(window.innerWidth, window.innerHeight);
            updateScaleBar();
        }

        function animate() {
            requestAnimationFrame(animate);
            if (params.navMode === 'Fly') {
                const delta = clock.getDelta();
                const speed = params.flySpeed * (keys.shift ? 3.0 : 1.0) * delta;
                const forward = new THREE.Vector3(0, 0, -1).applyQuaternion(currentCamera.quaternion);
                const right = new THREE.Vector3(1, 0, 0).applyQuaternion(currentCamera.quaternion);
                const up = new THREE.Vector3(0, 0, 1);
                if (keys.w) currentCamera.position.addScaledVector(forward, speed);
                if (keys.s) currentCamera.position.addScaledVector(forward, -speed);
                if (keys.a) currentCamera.position.addScaledVector(right, -speed);
                if (keys.d) currentCamera.position.addScaledVector(right, speed);
                if (keys.e) currentCamera.position.addScaledVector(up, speed);
                if (keys.q) currentCamera.position.addScaledVector(up, -speed);
            } else {
                clock.getDelta();
                controls.update();
            }

            // Update Camera Omni light position & mode
            if (material) {
                const omniView = new THREE.Vector3(0.5, 1.3, 1.0);
                material.uniforms.uLightPosView.value.copy(omniView);
                material.uniforms.uLightType.value = params.lightType === '3-Point Rig' ? 0.0 : 1.0;
            }

            if (!freezeRender) {
                renderer.render(scene, currentCamera);
            }

            measurements.forEach(m => {
                const mid = m.p1.clone().lerp(m.p2, 0.5);
                mid.project(currentCamera);
                if (mid.z > 1.0) m.div.style.display = 'none';
                else {
                    m.div.style.display = 'block';
                    m.div.style.left = ((mid.x * 0.5 + 0.5) * window.innerWidth) + 'px';
                    m.div.style.top = ((-mid.y * 0.5 + 0.5) * window.innerHeight) + 'px';
                }
            });
        }

        initViewer();
    </script>
</body>
</html>
'''


# ----------------------------------------------------------------------
# Parallel-safe worker helpers
# Operating only on NumPy arrays, primitive values, and file operations.
# ----------------------------------------------------------------------
def _lidar_float_list(values):
    return [float(v) for v in values]


def _lidar_store_payload_worker(chunk, key, compressed_bytes, storage_external, data_dir_name, data_dir_abs, chunk_index, file_prefix="chunk"):
    if storage_external:
        suffix = {"pos": "pos", "color": "col", "intensity": "int"}[key]
        safe_prefix = file_prefix or "chunk"
        filename = f"{safe_prefix}_{chunk_index:05d}.{suffix}.z"
        abs_path = os.path.join(data_dir_abs, filename)
        with open(abs_path, 'wb') as f:
            f.write(compressed_bytes)
        chunk[f"{key}File"] = f"{data_dir_name}/{filename}"
    else:
        chunk[f"{key}B64"] = base64.b64encode(compressed_bytes).decode('ascii')


def _lidar_deduplicate_points_worker(positions3, colors_u8, intensity_array, threshold):
    threshold = max(float(threshold), 1e-9)
    # Scale coordinates to grid cell integers
    grid = np.rint(positions3 / threshold).astype(np.int64)
    grid = np.ascontiguousarray(grid)
    
    # High-Performance deduplication: View rows of 2D grid as a 1D void structure.
    # This avoids sorting individual dimensions, speeding up deduplication significantly.
    void_view = grid.view(np.dtype((np.void, grid.dtype.itemsize * grid.shape[1])))
    _, unique_indices = np.unique(void_view, return_index=True)
    unique_indices.sort() # Keep the original order of the points
    
    new_positions = positions3[unique_indices]
    new_colors = colors_u8[unique_indices] if colors_u8 is not None else None
    new_intensity = intensity_array[unique_indices] if intensity_array is not None else None
    return new_positions, new_colors, new_intensity


def _lidar_morton3_10_worker(chunk_pos):
    c_min = chunk_pos.min(axis=0)
    c_max = chunk_pos.max(axis=0)
    span = np.maximum(c_max - c_min, 1e-9)
    
    # Direct normalization to 10-bit integer space (0 to 1023)
    norm = np.floor(((chunk_pos - c_min) / span) * 1023.0).astype(np.uint32)
    x = norm[:, 0]
    y = norm[:, 1]
    z = norm[:, 2]

    def part1by2(n):
        n = n & np.uint32(0x000003ff)
        n = (n | (n << np.uint32(16))) & np.uint32(0x030000ff)
        n = (n | (n << np.uint32(8))) & np.uint32(0x0300f00f)
        n = (n | (n << np.uint32(4))) & np.uint32(0x030c30c3)
        n = (n | (n << np.uint32(2))) & np.uint32(0x09249249)
        return n.astype(np.uint64)

    return part1by2(x) | (part1by2(y) << np.uint64(1)) | (part1by2(z) << np.uint64(2))


def _lidar_encode_positions_worker(chunk_pos, quantize, precision, optimize_positions):
    count = int(chunk_pos.shape[0])

    if not quantize:
        payload = np.ascontiguousarray(chunk_pos.astype(np.float32)).tobytes()
        return {
            "payload": payload,
            "origin": [0.0, 0.0, 0.0],
            "pos_type": "float32",
            "pos_encoding": "absolute",
            "delta_type": None,
            "order": None,
            "uncompressed_bytes": len(payload),
        }

    scale = int(10 ** precision)
    origin = chunk_pos.min(axis=0).astype(np.float64)
    q_float = np.rint((chunk_pos - origin) * float(scale))
    q_float = np.maximum(q_float, 0.0)
    q_max = float(np.max(q_float)) if q_float.size else 0.0

    if q_max <= np.iinfo(np.uint16).max:
        abs_dtype = np.uint16
        pos_type = "uint16"
    elif q_max <= np.iinfo(np.uint32).max:
        abs_dtype = np.uint32
        pos_type = "uint32"
    else:
        raise RuntimeError("Chunk exceeds uint32 range. Reduce precision or chunk size.")

    q_int64 = q_float.astype(np.int64)

    if optimize_positions and count > 1:
        try:
            morton = _lidar_morton3_10_worker(chunk_pos)
            order = np.argsort(morton, kind='stable')
            q_sorted = np.ascontiguousarray(q_int64[order])
            deltas = np.diff(q_sorted, axis=0)
            dmin = int(deltas.min()) if deltas.size else 0
            dmax = int(deltas.max()) if deltas.size else 0

            if dmin >= np.iinfo(np.int16).min and dmax <= np.iinfo(np.int16).max:
                delta_dtype = np.int16
                delta_type = "int16"
            elif dmin >= np.iinfo(np.int32).min and dmax <= np.iinfo(np.int32).max:
                delta_dtype = np.int32
                delta_type = "int32"
            else:
                raise OverflowError("Delta range exceeds int32.")

            first = np.ascontiguousarray(q_sorted[0].astype(np.uint32))
            delta_payload = np.ascontiguousarray(deltas.astype(delta_dtype)).tobytes()
            payload = first.tobytes() + delta_payload

            return {
                "payload": payload,
                "origin": _lidar_float_list(origin),
                "pos_type": pos_type,
                "pos_encoding": "delta_morton",
                "delta_type": delta_type,
                "order": order,
                "uncompressed_bytes": len(payload),
            }
        except Exception:
            # Fall back to absolute quantized positions in case of any sorting error
            pass

    q_abs = np.ascontiguousarray(q_float.astype(abs_dtype))
    payload = q_abs.tobytes()
    return {
        "payload": payload,
        "origin": _lidar_float_list(origin),
        "pos_type": pos_type,
        "pos_encoding": "absolute",
        "delta_type": None,
        "order": None,
        "uncompressed_bytes": len(payload),
    }


def _lidar_encode_chunk_worker(task):
    local_positions = task["positions_flat"].reshape((-1, 3)).astype(np.float64, copy=False)
    matrix_np = task["matrix_np"]
    translation_np = task["translation_np"]

    # Local position to transformed coordinates
    chunk_pos = local_positions @ matrix_np[:3, :3].T
    chunk_pos += translation_np
    chunk_pos = np.ascontiguousarray(chunk_pos)

    source_start = int(task["source_start"])
    source_end = int(task["source_end"])
    source_count = int(source_end - source_start)
    local_chunk_idx = int(task["local_chunk_idx"])
    total_chunks = int(task["total_chunks"])

    col_slice = task["colors_slice"]
    int_slice = task["intensity_slice"]

    count = int(chunk_pos.shape[0])
    if count <= 0:
        return {
            "skipped": True,
            "local_chunk_idx": local_chunk_idx,
            "total_chunks": total_chunks,
            "source_count": source_count,
            "source_end": source_end,
        }

    if task["merge_points"]:
        chunk_pos, col_slice, int_slice = _lidar_deduplicate_points_worker(
            chunk_pos,
            col_slice,
            int_slice,
            task["merge_precision"],
        )
        count = int(chunk_pos.shape[0])
        if count <= 0:
            return {
                "skipped": True,
                "local_chunk_idx": local_chunk_idx,
                "total_chunks": total_chunks,
                "source_count": source_count,
                "source_end": source_end,
            }

    cmin = chunk_pos.min(axis=0).astype(np.float64)
    cmax = chunk_pos.max(axis=0).astype(np.float64)

    encoded = _lidar_encode_positions_worker(
        chunk_pos=chunk_pos,
        quantize=task["quantize"],
        precision=task["precision"],
        optimize_positions=task["optimize_positions"],
    )

    order = encoded["order"]
    if order is not None:
        if col_slice is not None:
            col_slice = col_slice[order]
        if int_slice is not None:
            int_slice = int_slice[order]

    chunk_index = int(task["chunk_index"])
    chunk = {
        "index": chunk_index,
        "objectName": task["obj_name"],
        "sourceStart": source_start,
        "sourceEnd": source_end,
        "count": count,
        "boundsMin": _lidar_float_list(cmin),
        "boundsMax": _lidar_float_list(cmax),
        "origin": encoded["origin"],
        "posType": encoded["pos_type"],
        "posEncoding": encoded["pos_encoding"],
        "precision": int(task["precision"]) if task["quantize"] else 0,
        "scale": int(10 ** int(task["precision"])) if task["quantize"] else 1,
    }
    if encoded["delta_type"] is not None:
        chunk["deltaType"] = encoded["delta_type"]

    compression_level = int(task["compression_level"])
    storage_external = bool(task["storage_external"])
    data_dir_name = task["data_dir_name"]
    data_dir_abs = task["data_dir_abs"]
    file_prefix = task.get("file_prefix", "chunk")

    pos_z = zlib.compress(encoded["payload"], level=compression_level)
    pos_comp = len(pos_z)
    _lidar_store_payload_worker(chunk, "pos", pos_z, storage_external, data_dir_name, data_dir_abs, chunk_index, file_prefix=file_prefix)

    compressed_by_kind = {"position": pos_comp, "color": 0, "intensity": 0}
    uncompressed_by_kind = {"position": encoded["uncompressed_bytes"], "color": 0, "intensity": 0}
    compressed_bytes = pos_comp
    uncompressed_bytes = encoded["uncompressed_bytes"]

    if col_slice is not None:
        col_payload = np.ascontiguousarray(col_slice).tobytes()
        col_z = zlib.compress(col_payload, level=compression_level)
        col_comp = len(col_z)
        _lidar_store_payload_worker(chunk, "color", col_z, storage_external, data_dir_name, data_dir_abs, chunk_index, file_prefix=file_prefix)
        compressed_by_kind["color"] = col_comp
        uncompressed_by_kind["color"] = len(col_payload)
        compressed_bytes += col_comp
        uncompressed_bytes += len(col_payload)

    if int_slice is not None:
        int_payload = np.ascontiguousarray(int_slice).tobytes()
        int_z = zlib.compress(int_payload, level=compression_level)
        int_comp = len(int_z)
        _lidar_store_payload_worker(chunk, "intensity", int_z, storage_external, data_dir_name, data_dir_abs, chunk_index, file_prefix=file_prefix)
        compressed_by_kind["intensity"] = int_comp
        uncompressed_by_kind["intensity"] = len(int_payload)
        compressed_bytes += int_comp
        uncompressed_bytes += len(int_payload)

    return {
        "skipped": False,
        "local_chunk_idx": local_chunk_idx,
        "total_chunks": total_chunks,
        "source_count": source_count,
        "source_end": source_end,
        "chunk": chunk,
        "point_count": count,
        "compressed_bytes": compressed_bytes,
        "uncompressed_bytes": uncompressed_bytes,
        "compressed_by_kind": compressed_by_kind,
        "uncompressed_by_kind": uncompressed_by_kind,
        "pos_type": encoded["pos_type"],
        "pos_encoding": encoded["pos_encoding"],
        "bounds_min": cmin,
        "bounds_max": cmax,
    }


class EXPORT_OT_lidar_html(bpy.types.Operator, ExportHelper):
    """Export multiple selected LiDAR point clouds into a single HTML viewer, with chunked loading, quantization, delta/Morton compression, global point limiting/cutoff, preview LOD, and auto external chunks for large sets."""
    bl_idname = "export_scene.lidar_html"
    bl_label = "Export to HTML"
    bl_options = {'REGISTER', 'UNDO'}

    filename_ext = ".html"
    filter_glob: bpy.props.StringProperty(default="*.html", options={'HIDDEN'}, maxlen=255)
    
    COLORATTRNAMES = ("Col", "color", "Color", "col", "Cd", "rgb", "RGB", "rgba", "RGBA")
    INTENSITYATTRNAMES = ("intensity", "Intensity", "intensities", "Intensities", "reflectance", "Reflectance")
    COLOR_ATTR_NAMES = ["Col", "color", "Color", "col", "Cd", "rgb", "RGB", "rgba", "RGBA"]
    INTENSITY_ATTR_NAMES = ["intensity", "Intensity", "intensities", "Intensity_", "reflectance", "Reflectance"]

    AUTO_EXTERNAL_LIMIT = 100_000_000

    def execute(self, context):
        scene = context.scene
        wm = context.window_manager

        selected = [o for o in context.selected_objects if o.type in {'MESH', 'POINTCLOUD'}]
        if not selected:
            self.report({'ERROR'}, "Select at least one Mesh or PointCloud object.")
            return {'CANCELLED'}
        selected.sort(key=lambda o: o.name.lower())

        active_obj = context.active_object
        if not active_obj or active_obj not in selected:
            active_obj = selected[0]

        origin_offset = active_obj.matrix_world.translation.copy() if active_obj else Vector((0, 0, 0))
        origin_np = np.array((origin_offset.x, origin_offset.y, origin_offset.z), dtype=np.float64)

        object_point_estimates = [self._count_points(obj) for obj in selected]
        total_points_est = int(sum(object_point_estimates))
        if total_points_est <= 0:
            self.report({'ERROR'}, "Selected selected objects contain no source points.")
            return {'CANCELLED'}

        storage_external = bool(scene.lidar_external_chunks)
        chunked_export = bool(scene.lidar_chunked_export)
        spatial_chunking = bool(scene.lidar_spatial_chunking)

        if total_points_est >= self.AUTO_EXTERNAL_LIMIT:
            if not storage_external:
                self.report({'INFO'}, f"Total points ({total_points_est:,}) >= 100M, forcing external chunk files.".replace(',', ' '))
                storage_external = True
            if not chunked_export:
                self.report({'INFO'}, "Large export detected, forcing chunked loading.")
                chunked_export = True
            if spatial_chunking:
                self.report({'INFO'}, "Large multi-object export detected, disabling spatial chunking.")
                spatial_chunking = False

        if len(selected) > 1 and spatial_chunking:
            self.report({'INFO'}, "Multi-object sequential export: disabling spatial chunking to preserve object/room order.")
            spatial_chunking = False

        title = os.path.splitext(os.path.basename(self.filepath))[0]
        output_dir = os.path.dirname(os.path.abspath(self.filepath)) or os.getcwd()
        data_dir_name = f"{title}_data"
        data_dir_abs = os.path.join(output_dir, data_dir_name)
        if storage_external:
            os.makedirs(data_dir_abs, exist_ok=True)

        global_min = np.array([np.inf, np.inf, np.inf], dtype=np.float64)
        global_max = np.array([-np.inf, -np.inf, -np.inf], dtype=np.float64)

        all_chunks = []
        chunk_index = 0
        total_points = 0
        total_compressed = 0
        total_uncompressed = 0
        compressed_by_kind = {"position": 0, "color": 0, "intensity": 0}
        uncompressed_by_kind = {"position": 0, "color": 0, "intensity": 0}
        pos_type_counts = {"uint16": 0, "uint32": 0, "float32": 0}
        pos_encoding_counts = {"absolute": 0, "delta_morton": 0}

        generate_preview_lod = bool(scene.lidar_generate_preview_lod)
        lod_chunks = []
        lod_chunk_index = 0
        lod_total_points = 0
        lod_total_compressed = 0
        lod_total_uncompressed = 0

        request_color = bool(scene.lidar_export_colors)
        request_intensity = bool(scene.lidar_export_intensity)
        exported_any_color = False
        exported_any_intensity = False
        final_intensity_type = "none"

        compression_level = int(scene.lidar_compression_level)
        optimize_positions = bool(scene.lidar_optimize_position_compression and scene.lidar_quantize_positions)
        parallel_chunks = bool(scene.lidar_parallel_chunk_compression)
        parallel_workers = int(scene.lidar_parallel_workers)
        if parallel_workers <= 0:
            cpu = os.cpu_count() or 2
            parallel_workers = max(1, min(max(1, cpu - 1), 4))
        parallel_workers = max(1, min(parallel_workers, 32))
        if parallel_chunks and parallel_workers > 1:
            self.report({'INFO'}, f"Parallel chunk compression enabled: {parallel_workers} workers.")
            print(f"[LiDAR Export] Parallel chunk compression enabled: {parallel_workers} workers", flush=True)

        total_objects = len(selected)
        processed_input_base = 0

        self._last_console_progress = -1.0
        self._last_console_time = 0.0

        wm.progress_begin(0, max(1, total_points_est))
        self._update_progress(
            wm, 0, total_points_est, 0, total_objects, "starting",
            force=True, extra="initializing export"
        )

        try:
            depsgraph = context.evaluated_depsgraph_get()

            for obj_idx, obj in enumerate(selected):
                mesh_owner = None
                object_est_points = object_point_estimates[obj_idx]
                object_progress_base = processed_input_base

                self.report({'INFO'}, f"Processing object {obj_idx + 1}/{total_objects}: {obj.name}")
                self._update_progress(
                    wm, object_progress_base, total_points_est,
                    obj_idx + 1, total_objects, obj.name,
                    force=True, extra="object start"
                )

                try:
                    eval_obj = obj.evaluated_get(depsgraph)
                    positions, mesh_or_pc, num_points, mesh_owner = self._extract_geometry(eval_obj, obj)

                    if num_points <= 0:
                        processed_input_base += object_est_points
                        continue

                    colors_u8 = None
                    if request_color:
                        colors_u8 = self._extract_colors(mesh_or_pc, num_points)
                        if colors_u8 is None:
                            colors_u8 = self._default_colors(num_points)
                        exported_any_color = True

                    intensity_array = None
                    object_intensity_type = "none"
                    if request_intensity:
                        intensity_array, object_intensity_type = self._extract_intensity(
                            mesh_or_pc,
                            num_points,
                            scene.lidar_intensity_format,
                        )
                        if intensity_array is None:
                            intensity_array, object_intensity_type = self._default_intensity(
                                num_points,
                                scene.lidar_intensity_format,
                            )
                        exported_any_intensity = True
                        final_intensity_type = object_intensity_type

                    matrix_np = self._matrix_to_numpy(obj.matrix_world)

                    stats = self._process_object_in_chunks(
                        obj_name=obj.name,
                        positions_flat=positions,
                        matrix_np=matrix_np,
                        origin_np=origin_np,
                        colors_u8=colors_u8,
                        intensity_array=intensity_array,
                        intensity_type=object_intensity_type if intensity_array is not None else "none",
                        compression_level=compression_level,
                        quantize=scene.lidar_quantize_positions,
                        precision=scene.lidar_position_precision,
                        optimize_positions=optimize_positions,
                        parallel_chunks=parallel_chunks,
                        parallel_workers=parallel_workers,
                        chunked=chunked_export,
                        target_mb=scene.lidar_chunk_target_mb,
                        merge_points=scene.lidar_merge_points,
                        merge_precision=float(scene.lidar_merge_precision),
                        storage_external=storage_external,
                        data_dir_name=data_dir_name,
                        data_dir_abs=data_dir_abs,
                        start_chunk_index=chunk_index,
                        all_chunks=all_chunks,
                        file_prefix="chunk",
                        progress_callback=lambda local_done, local_total, source_done, name=obj.name, idx=obj_idx, base=object_progress_base: self._update_progress(
                            wm,
                            min(total_points_est, base + source_done),
                            total_points_est,
                            idx + 1,
                            total_objects,
                            name,
                            chunk_idx=local_done,
                            total_chunks=local_total,
                            extra="chunk encoded",
                        ),
                    )

                    chunk_index += stats.get("allocated_chunk_count", stats["chunk_count"])
                    total_points += stats["point_count"]
                    total_compressed += stats["compressed_bytes"]
                    total_uncompressed += stats["uncompressed_bytes"]

                    for key in compressed_by_kind:
                        compressed_by_kind[key] += stats["compressed_by_kind"][key]
                        uncompressed_by_kind[key] += stats["uncompressed_by_kind"][key]
                    for key in pos_type_counts:
                        pos_type_counts[key] += stats["pos_type_counts"][key]
                    for key in pos_encoding_counts:
                        pos_encoding_counts[key] += stats["pos_encoding_counts"][key]

                    if stats["point_count"] > 0:
                        global_min = np.minimum(global_min, stats["bounds_min"])
                        global_max = np.maximum(global_max, stats["bounds_max"])

                    if generate_preview_lod:
                        try:
                            sample_indices = self._lod_sample_indices(num_points, percent=1)
                            if sample_indices.size > 0:
                                pos_view = positions.reshape((-1, 3))
                                lod_positions = np.ascontiguousarray(pos_view[sample_indices].reshape(-1))
                                lod_colors = np.ascontiguousarray(colors_u8[sample_indices]) if colors_u8 is not None else None
                                lod_intensity = np.ascontiguousarray(intensity_array[sample_indices]) if intensity_array is not None else None

                                lod_stats = self._process_object_in_chunks(
                                    obj_name=obj.name,
                                    positions_flat=lod_positions,
                                    matrix_np=matrix_np,
                                    origin_np=origin_np,
                                    colors_u8=lod_colors,
                                    intensity_array=lod_intensity,
                                    intensity_type=object_intensity_type if lod_intensity is not None else "none",
                                    compression_level=compression_level,
                                    quantize=True,
                                    precision=2,
                                    optimize_positions=True,
                                    parallel_chunks=parallel_chunks,
                                    parallel_workers=parallel_workers,
                                    chunked=True,
                                    target_mb=min(int(scene.lidar_chunk_target_mb), 8),
                                    merge_points=False,
                                    merge_precision=0.01,
                                    storage_external=storage_external,
                                    data_dir_name=data_dir_name,
                                    data_dir_abs=data_dir_abs,
                                    start_chunk_index=lod_chunk_index,
                                    all_chunks=lod_chunks,
                                    file_prefix="lod_preview",
                                    progress_callback=None,
                                )
                                lod_chunk_index += lod_stats.get("allocated_chunk_count", lod_stats["chunk_count"])
                                lod_total_points += lod_stats["point_count"]
                                lod_total_compressed += lod_stats["compressed_bytes"]
                                lod_total_uncompressed += lod_stats["uncompressed_bytes"]
                                del lod_positions
                                del lod_colors
                                del lod_intensity
                                del lod_stats
                        except Exception as lod_exc:
                            self.report({'WARNING'}, f"Preview LOD skipped for '{obj.name}': {lod_exc}")
                            print(f"[LiDAR Export] WARNING preview LOD skipped: {obj.name}: {lod_exc}", flush=True)

                    del positions
                    del colors_u8
                    del intensity_array
                    del stats

                except Exception as exc:
                    self.report({'WARNING'}, f"Object '{obj.name}' skipped: {exc}")
                    print(f"[LiDAR Export] WARNING object skipped: {obj.name}: {exc}", flush=True)
                finally:
                    if mesh_owner is not None:
                        try:
                            # Blender 4.0/5.x method to release mesh
                            mesh_owner.to_mesh_clear()
                        except Exception:
                            pass

                processed_input_base += object_est_points
                self._update_progress(
                    wm, min(total_points_est, processed_input_base), total_points_est,
                    obj_idx + 1, total_objects, obj.name,
                    force=True, extra="object complete"
                )

        finally:
            wm.progress_end()

        if total_points == 0:
            self.report({'ERROR'}, "No points to export after processing all objects.")
            return {'CANCELLED'}

        total_points = self._assign_global_ranges(all_chunks)
        lod_total_points = self._assign_global_ranges(lod_chunks) if lod_chunks else 0

        center = (global_min + global_max) * 0.5
        radius = float(np.linalg.norm((global_max - global_min) * 0.5))
        compression_ratio = float(total_uncompressed / total_compressed) if total_compressed > 0 else 1.0

        lods = []
        if generate_preview_lod and lod_chunks:
            lods.append({
                "name": "preview_1pct_q2",
                "label": "Preview LOD 1% / precision 2",
                "pointPercent": 1,
                "precision": 2,
                "scale": 100,
                "quantized": True,
                "pointCount": int(lod_total_points),
                "chunkCount": len(lod_chunks),
                "storage": "external" if storage_external else "embedded",
                "compressedBytes": int(lod_total_compressed),
                "uncompressedBytes": int(lod_total_uncompressed),
                "chunks": lod_chunks,
            })

        meta = {
            "formatVersion": 5,
            "title": title,
            "generator": "LiDAR WebGL HTML Exporter - Chunked Quantized 3.7.0",
            "pointCount": int(total_points),
            "chunkCount": len(all_chunks),
            "storage": "external" if storage_external else "embedded",
            "quantized": bool(scene.lidar_quantize_positions),
            "positionEncoding": "delta_morton" if optimize_positions else "absolute",
            "precision": int(scene.lidar_position_precision),
            "scale": int(10 ** scene.lidar_position_precision) if scene.lidar_quantize_positions else 1,
            "unit": "meter",
            "hasColor": bool(request_color and exported_any_color),
            "colorType": "rgba8_norm" if (request_color and exported_any_color) else "none",
            "hasIntensity": bool(request_intensity and exported_any_intensity),
            "intensityType": final_intensity_type if (request_intensity and exported_any_intensity) else "none",
            "bounds": {
                "min": self._float_list(global_min),
                "max": self._float_list(global_max),
                "center": self._float_list(center),
                "radius": radius,
            },
            # Custom default parameters defined in the Blender UI Defaults panel
            "viewerDefaults": {
                "opacity": float(scene.lidar_default_opacity),
                "brightness": float(scene.lidar_default_brightness),
                "globalIntensity": float(scene.lidar_default_global_intensity),
                "localIntensity": float(scene.lidar_default_local_intensity),
                "pointLimitMinPct": int(scene.lidar_default_point_limit_min_pct),
                "pointLimitMaxPct": int(scene.lidar_default_point_limit_max_pct),
            },
            "chunkTargetMB": scene.lidar_chunk_target_mb,
            "spatialChunking": bool(spatial_chunking),
            "compressedBytes": int(total_compressed),
            "uncompressedBytes": int(total_uncompressed),
            "compressionRatio": compression_ratio,
            "compressedBytesByKind": {k: int(v) for k, v in compressed_by_kind.items()},
            "uncompressedBytesByKind": {k: int(v) for k, v in uncompressed_by_kind.items()},
            "posTypeChunkCounts": pos_type_counts,
            "posEncodingChunkCounts": pos_encoding_counts,
            "lods": lods,
            "chunks": all_chunks,
        }

        cameras_data = self._extract_cameras(context, active_obj, origin_offset)
        html_content = HTML_TEMPLATE
        html_content = html_content.replace("__TITLE_HTML__", html.escape(title))
        html_content = html_content.replace("__FILE_NAME_JSON__", json.dumps(title))
        html_content = html_content.replace("__PC_META_JSON__", json.dumps(meta, separators=(',', ':')))
        html_content = html_content.replace("__CAMERAS_JSON__", json.dumps(cameras_data, separators=(',', ':')))

        with open(self.filepath, 'w', encoding='utf-8') as f:
            f.write(html_content)

        if storage_external:
            self._write_server_helpers(output_dir, os.path.basename(self.filepath))

        size_hint = self._format_bytes(os.path.getsize(self.filepath))
        print(
            f"[LiDAR Export] DONE 100.00% | objects {len(selected)} | points {total_points:,} | "
            f"chunks {len(all_chunks)} | compressed {self._format_bytes(total_compressed)} | "
            f"ratio {compression_ratio:.2f}:1".replace(',', ' '),
            flush=True,
        )
        self.report(
            {'INFO'},
            f"Exported {len(selected)} objects, {total_points:,} points, {len(all_chunks)} chunks. "
            f"HTML: {size_hint}. Data compression ratio: {compression_ratio:.2f}:1".replace(',', ' ')
        )
        return {'FINISHED'}

    def _lod_sample_indices(self, num_points, percent=1):
        num_points = int(num_points)
        if num_points <= 0:
            return np.empty(0, dtype=np.int64)
        percent = max(0.0001, min(100.0, float(percent)))
        step = max(1, int(round(100.0 / percent)))
        return np.arange(0, num_points, step, dtype=np.int64)

    def _assign_global_ranges(self, chunks):
        cursor = 0
        for sequence_index, chunk in enumerate(chunks):
            count = int(chunk.get("count", 0))
            chunk["sequenceIndex"] = int(sequence_index)
            chunk["globalStart"] = int(cursor)
            cursor += count
            chunk["globalEnd"] = int(cursor)
        return int(cursor)

    def _update_progress(self, wm, processed, total, obj_idx, total_objects, obj_name,
                          chunk_idx=None, total_chunks=None, force=False, extra=""):
        total = max(1, int(total))
        processed = int(max(0, min(processed, total)))
        try:
            wm.progress_update(processed)
        except Exception:
            pass

        pct = 100.0 * processed / total
        now = time.monotonic()
        should_print = (
            force or
            self._last_console_progress < 0 or
            pct - self._last_console_progress >= 1.0 or
            now - self._last_console_time >= 5.0
        )
        if should_print:
            chunk_txt = ""
            if chunk_idx is not None and total_chunks is not None:
                chunk_txt = f" | chunk {chunk_idx}/{total_chunks}"
            extra_txt = f" | {extra}" if extra else ""
            print(
                f"[LiDAR Export] {pct:6.2f}% | object {obj_idx}/{total_objects}: {obj_name}"
                f"{chunk_txt} | source points {processed:,}/{total:,}{extra_txt}".replace(',', ' '),
                flush=True,
            )
            self._last_console_progress = pct
            self._last_console_time = now

    def _count_points(self, obj):
        try:
            if obj.type == 'POINTCLOUD':
                return len(obj.data.points)
            if obj.type == 'MESH':
                return len(obj.data.vertices)
        except Exception:
            return 0
        return 0

    def _matrix_to_numpy(self, matrix):
        # Direct buffer protocol translation is much faster than nested list comprehensions
        return np.array(matrix, dtype=np.float64)

    def _extract_geometry(self, eval_obj, original_obj):
        """Extract local positions from Mesh or PointCloud."""
        if original_obj.type == 'POINTCLOUD':
            pc_data = eval_obj.data
            num_points = len(pc_data.points)
            if num_points <= 0:
                return np.empty(0, dtype=np.float32), pc_data, 0, None
            if "position" not in pc_data.attributes:
                raise RuntimeError("PointCloud has no 'position' attribute.")
            positions = np.empty(num_points * 3, dtype=np.float32)
            pc_data.attributes["position"].data.foreach_get("vector", positions)
            mesh_owner = None
            return positions, pc_data, num_points, mesh_owner
        else:
            mesh = eval_obj.to_mesh()
            num_points = len(mesh.vertices)
            if num_points <= 0:
                return np.empty(0, dtype=np.float32), mesh, 0, eval_obj
            positions = np.empty(num_points * 3, dtype=np.float32)
            mesh.vertices.foreach_get("co", positions)
            mesh_owner = eval_obj
            return positions, mesh, num_points, mesh_owner

    def _default_colors(self, num_points):
        colors = np.empty((num_points, 4), dtype=np.uint8)
        colors[:, 0:3] = 255
        colors[:, 3] = 255
        return colors

    def _default_intensity(self, num_points, mode):
        if mode == 'UINT16':
            return np.zeros(num_points, dtype=np.uint16), "uint16_norm"
        if mode == 'FLOAT32':
            return np.zeros(num_points, dtype=np.float32), "float32"
        return np.zeros(num_points, dtype=np.uint8), "uint8_norm"

    def _points_per_chunk(self, has_color, intensity_array, chunked, target_mb):
        bytes_per_point_est = 12
        if has_color:
            bytes_per_point_est += 4
        if intensity_array is not None:
            if intensity_array.dtype == np.uint8:
                bytes_per_point_est += 1
            elif intensity_array.dtype == np.uint16:
                bytes_per_point_est += 2
            else:
                bytes_per_point_est += 4

        if chunked:
            target_bytes = max(1, int(target_mb)) * 1024 * 1024
            return max(1, int(target_bytes // max(1, bytes_per_point_est)))
        return None

    def _process_object_in_chunks(self, obj_name, positions_flat, matrix_np, origin_np,
                                  colors_u8, intensity_array, intensity_type,
                                  compression_level, quantize, precision, optimize_positions,
                                  parallel_chunks, parallel_workers,
                                  chunked, target_mb, merge_points, merge_precision,
                                  storage_external, data_dir_name, data_dir_abs,
                                  start_chunk_index, all_chunks, file_prefix="chunk", progress_callback=None):
        source_points = int(len(positions_flat) // 3)
        ppc = self._points_per_chunk(colors_u8 is not None, intensity_array, chunked, target_mb)
        if ppc is None:
            ppc = source_points

        chunk_ranges = [(start, min(start + ppc, source_points)) for start in range(0, source_points, ppc)]
        total_chunks = len(chunk_ranges)

        stats = {
            "chunk_count": 0,
            "allocated_chunk_count": total_chunks,
            "point_count": 0,
            "compressed_bytes": 0,
            "uncompressed_bytes": 0,
            "compressed_by_kind": {"position": 0, "color": 0, "intensity": 0},
            "uncompressed_by_kind": {"position": 0, "color": 0, "intensity": 0},
            "pos_type_counts": {"uint16": 0, "uint32": 0, "float32": 0},
            "pos_encoding_counts": {"absolute": 0, "delta_morton": 0},
            "bounds_min": np.array([np.inf, np.inf, np.inf], dtype=np.float64),
            "bounds_max": np.array([-np.inf, -np.inf, -np.inf], dtype=np.float64),
        }

        if merge_points and source_points >= 10_000_000:
            self.report(
                {'WARNING'},
                f"Merge Points enabled for large object '{obj_name}'. Deduplication is now per chunk, not whole object."
            )

        use_parallel = bool(parallel_chunks and parallel_workers > 1 and total_chunks > 1)
        max_in_flight = max(1, int(parallel_workers) * 2)
        completed_chunks = 0
        completed_source_points = 0
        object_results = []

        # Precompute the translation vector in the main thread once to save allocations in worker threads
        translation_np = matrix_np[:3, 3] - origin_np

        def make_task(local_chunk_idx, start, end):
            local_positions = np.ascontiguousarray(positions_flat[start * 3:end * 3])
            color_slice = np.ascontiguousarray(colors_u8[start:end]) if colors_u8 is not None else None
            intensity_slice = np.ascontiguousarray(intensity_array[start:end]) if intensity_array is not None else None
            return {
                "obj_name": obj_name,
                "local_chunk_idx": local_chunk_idx,
                "total_chunks": total_chunks,
                "source_start": int(start),
                "source_end": int(end),
                "chunk_index": int(start_chunk_index + local_chunk_idx - 1),
                "positions_flat": local_positions,
                "matrix_np": matrix_np,
                "translation_np": translation_np,
                "colors_slice": color_slice,
                "intensity_slice": intensity_slice,
                "intensity_type": intensity_type,
                "compression_level": compression_level,
                "quantize": quantize,
                "precision": precision,
                "optimize_positions": optimize_positions,
                "merge_points": merge_points,
                "merge_precision": merge_precision,
                "storage_external": storage_external,
                "data_dir_name": data_dir_name,
                "data_dir_abs": data_dir_abs,
                "file_prefix": file_prefix,
            }

        def consume_result(result):
            nonlocal completed_chunks, completed_source_points
            completed_chunks += 1
            completed_source_points += int(result.get("source_count", 0))

            if progress_callback is not None:
                progress_callback(completed_chunks, total_chunks, min(source_points, completed_source_points))

            if result.get("skipped"):
                return

            object_results.append(result)
            stats["chunk_count"] += 1
            stats["point_count"] += int(result["point_count"])
            stats["compressed_bytes"] += int(result["compressed_bytes"])
            stats["uncompressed_bytes"] += int(result["uncompressed_bytes"])

            for key in stats["compressed_by_kind"]:
                stats["compressed_by_kind"][key] += int(result["compressed_by_kind"][key])
                stats["uncompressed_by_kind"][key] += int(result["uncompressed_by_kind"][key])

            stats["pos_type_counts"][result["pos_type"]] += 1
            stats["pos_encoding_counts"][result["pos_encoding"]] += 1
            stats["bounds_min"] = np.minimum(stats["bounds_min"], result["bounds_min"])
            stats["bounds_max"] = np.maximum(stats["bounds_max"], result["bounds_max"])

        if use_parallel:
            print(
                f"[LiDAR Export] Parallel object worker pool | {obj_name} | "
                f"{total_chunks} chunks | {parallel_workers} workers",
                flush=True,
            )
            with ThreadPoolExecutor(max_workers=int(parallel_workers)) as executor:
                pending = set()
                next_task_idx = 0

                while next_task_idx < total_chunks or pending:
                    while next_task_idx < total_chunks and len(pending) < max_in_flight:
                        start, end = chunk_ranges[next_task_idx]
                        task = make_task(next_task_idx + 1, start, end)
                        pending.add(executor.submit(_lidar_encode_chunk_worker, task))
                        next_task_idx += 1

                    if not pending:
                        break

                    done = set()
                    for future in as_completed(pending):
                        done.add(future)
                        break

                    for future in done:
                        pending.remove(future)
                        consume_result(future.result())
        else:
            for local_chunk_idx, (start, end) in enumerate(chunk_ranges, start=1):
                task = make_task(local_chunk_idx, start, end)
                consume_result(_lidar_encode_chunk_worker(task))

        object_results.sort(key=lambda r: int(r["chunk"]["index"]))
        for result in object_results:
            all_chunks.append(result["chunk"])

        return stats

    def _find_attr(self, mesh_or_pc, candidate_names):
        attrs = mesh_or_pc.attributes
        for name in candidate_names:
            if name in attrs:
                return attrs[name]
        return None

    def _extract_colors(self, meshorpc, numpoints):
        import numpy as np

        def read_color_data(attr):
            attrlen = len(attr.data)
            if attrlen <= 0:
                return None

            rawcolors = np.empty(attrlen * 4, dtype=np.float32)

            for key in ("color", "value"):
                try:
                    attr.data.foreach_get(key, rawcolors)
                    rawcolors = rawcolors.reshape(attrlen, 4)
                    return rawcolors
                except Exception:
                    pass
            return None

        def find_color_attr(meshorpc):
            preferred = [n.lower() for n in self.COLORATTRNAMES]

            color_attrs = getattr(meshorpc, "color_attributes", None)
            if color_attrs:
                for name in self.COLORATTRNAMES:
                    try:
                        attr = color_attrs.get(name)
                        if attr is not None:
                            return attr
                    except Exception:
                        pass

                for attr in color_attrs:
                    try:
                        if attr.name.lower() in preferred:
                            return attr
                    except Exception:
                        pass

                try:
                    active = color_attrs.active_color
                    if active is not None:
                        return active
                except Exception:
                    pass

                try:
                    if len(color_attrs):
                        return color_attrs[0]
                except Exception:
                    pass

            attrs = getattr(meshorpc, "attributes", None)
            if attrs:
                for name in self.COLORATTRNAMES:
                    try:
                        attr = attrs.get(name)
                        if attr is not None:
                            data_type = getattr(attr, "data_type", "")
                            if data_type in {"FLOAT_COLOR", "BYTE_COLOR"}:
                                return attr
                    except Exception:
                        pass

                for attr in attrs:
                    try:
                        if getattr(attr, "data_type", "") in {"FLOAT_COLOR", "BYTE_COLOR"}:
                            return attr
                    except Exception:
                        pass

            return None

        attr = find_color_attr(meshorpc)
        if attr is None:
            return None

        rawcolors = read_color_data(attr)
        if rawcolors is None:
            return None

        domain = getattr(attr, "domain", "POINT")

        if domain == 'CORNER' and hasattr(meshorpc, "loops"):
            loop_vertex_indices = np.empty(len(attr.data), dtype=np.int32)
            meshorpc.loops.foreach_get('vertex_index', loop_vertex_indices)

            vertexcolors = np.zeros((numpoints, 4), dtype=np.float32)
            for i in range(4):
                vertexcolors[:, i] = np.bincount(
                    loop_vertex_indices,
                    weights=rawcolors[:, i],
                    minlength=numpoints
                )

            loopcounts = np.bincount(loop_vertex_indices, minlength=numpoints)
            loopcounts = np.maximum(loopcounts, 1)
            colors = vertexcolors / loopcounts[:, np.newaxis]
        else:
            if rawcolors.shape[0] != numpoints:
                colors = rawcolors[:numpoints]
            else:
                colors = rawcolors

        if colors.size and np.nanmax(colors) > 1.0:
            colors = colors / 255.0

        colors = np.nan_to_num(colors, nan=1.0, posinf=1.0, neginf=0.0)
        colors = np.clip(colors, 0.0, 1.0)

        return np.rint(colors * 255.0).astype(np.uint8)


    def _extract_intensity(self, mesh_or_pc, num_points, mode):
        attr = self._find_attr(mesh_or_pc, self.INTENSITY_ATTR_NAMES)
        if not attr:
            return None, "none"
            
        attr_len = len(attr.data)
        if attr_len <= 0:
            return None, "none"
            
        raw_values = np.empty(attr_len, dtype=np.float32)
        read_ok = False
        for key in ("value", "color"):
            try:
                if key == "color":
                    tmp = np.empty(attr_len * 4, dtype=np.float32)
                    attr.data.foreach_get(key, tmp)
                    raw_values[:] = tmp.reshape((attr_len, 4))[:, 0]
                else:
                    attr.data.foreach_get(key, raw_values)
                read_ok = True
                break
            except Exception:
                pass
                
        if not read_ok:
            return None, "none"
            
        # Check if attribute domain is CORNER. Map to w-vertices.
        if attr.domain == 'CORNER' and hasattr(mesh_or_pc, "loops"):
            loop_vertex_indices = np.empty(attr_len, dtype=np.int32)
            mesh_or_pc.loops.foreach_get("vertex_index", loop_vertex_indices)
            
            vertex_values = np.bincount(loop_vertex_indices, weights=raw_values, minlength=num_points)
            loop_counts = np.bincount(loop_vertex_indices, minlength=num_points)
            loop_counts = np.maximum(loop_counts, 1)
            values = vertex_values / loop_counts
        else:
            values = raw_values

        values = np.nan_to_num(values, nan=0.0, posinf=0.0, neginf=0.0)
        vmin = float(np.min(values)) if values.size else 0.0
        vmax = float(np.max(values)) if values.size else 0.0
        if vmax > vmin:
            values = (values - vmin) / (vmax - vmin)
        elif vmax > 0.0:
            values = values / vmax
        else:
            values.fill(0.0)
        values = np.clip(values, 0.0, 1.0) ** 0.6
        
        if mode == 'UINT16':
            arr = np.rint(values * 65535.0).astype(np.uint16)
            out_type = "uint16_norm"
        elif mode == 'FLOAT32':
            arr = values.astype(np.float32)
            out_type = "float32"
        else:
            arr = np.rint(values * 255.0).astype(np.uint8)
            out_type = "uint8_norm"
        return arr, out_type

    def _extract_cameras(self, context, cloud_obj, origin_offset=None):
        cameras_data = []
        scene = context.scene
        cams = [o for o in scene.objects if o.type == 'CAMERA']
        active = scene.camera if scene.camera and scene.camera in cams else None
        if active:
            cams = [active] + sorted([c for c in cams if c != active], key=lambda c: c.name.lower())
        else:
            cams = sorted(cams, key=lambda c: c.name.lower())

        if origin_offset is None:
            origin_offset = Vector((0, 0, 0))

        translate_to_export_space = Matrix.Translation(-origin_offset)

        for cam in cams:
            export_mat = translate_to_export_space @ cam.matrix_world
            mat = export_mat.transposed()
            mat_flat = [float(val) for row in mat for val in row]
            cameras_data.append({
                "name": cam.name,
                "type": cam.data.type,
                "fov": float(cam.data.angle),
                "ortho_scale": float(cam.data.ortho_scale),
                "clip_start": float(cam.data.clip_start),
                "clip_end": float(cam.data.clip_end),
                "matrix": mat_flat,
            })
        return cameras_data

    def _write_server_helpers(self, output_dir, html_filename):
        bat_path = os.path.join(output_dir, "start_lidar_viewer_windows.bat")
        sh_path = os.path.join(output_dir, "start_lidar_viewer_mac_linux.sh")
        with open(bat_path, 'w', encoding='utf-8') as f:
            f.write(
                "@echo off\n"
                "cd /d \"%~dp0\"\n"
                f"start \"\" \"http://localhost:8000/{html_filename}\"\n"
                "python -m http.server 8000\n"
                "pause\n"
            )
        with open(sh_path, 'w', encoding='utf-8') as f:
            f.write(
                "#!/bin/sh\n"
                "cd \"$(dirname \"$0\")\"\n"
                f"echo Open: http://localhost:8000/{html_filename}\n"
                "python3 -m http.server 8000\n"
            )
        try:
            os.chmod(sh_path, 0o755)
        except Exception:
            pass

    def _float_list(self, values):
        return [float(v) for v in values]

    def _format_bytes(self, n):
        n = float(n)
        for unit in ['B', 'KB', 'MB', 'GB']:
            if n < 1024.0:
                return f"{n:.1f} {unit}"
            n /= 1024.0
        return f"{n:.1f} TB"


class LIDAR_PT_ExportPanel(bpy.types.Panel):
    bl_label = "LiDAR HTML WebGL"
    bl_idname = "LIDAR_PT_ExportPanel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "LiDAR"

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        obj = context.active_object

        layout.label(text="Export Data Options:")
        layout.prop(scene, "lidar_export_colors")
        layout.prop(scene, "lidar_export_intensity")
        if scene.lidar_export_intensity:
            layout.prop(scene, "lidar_intensity_format", text="Intensity Format")

        box = layout.box()
        box.label(text="Compact / Large Clouds:", icon='MOD_BUILD')
        box.prop(scene, "lidar_quantize_positions")
        row = box.row()
        row.enabled = scene.lidar_quantize_positions
        row.prop(scene, "lidar_position_precision", text="Precision")

        opt_row = box.row()
        opt_row.enabled = scene.lidar_quantize_positions
        opt_row.prop(scene, "lidar_optimize_position_compression")

        par_box = box.box()
        par_box.label(text="Parallel Processing:", icon='SETTINGS')
        par_box.prop(scene, "lidar_parallel_chunk_compression")
        row = par_box.row()
        row.enabled = scene.lidar_parallel_chunk_compression
        row.prop(scene, "lidar_parallel_workers", text="Workers")

        box.prop(scene, "lidar_chunked_export")
        row = box.row()
        row.enabled = scene.lidar_chunked_export and scene.lidar_quantize_positions
        row.prop(scene, "lidar_spatial_chunking")
        row = box.row()
        row.enabled = scene.lidar_chunked_export
        row.prop(scene, "lidar_chunk_target_mb", text="Chunk MB")
        box.prop(scene, "lidar_compression_level")
        box.prop(scene, "lidar_external_chunks")
        box.prop(scene, "lidar_generate_preview_lod")

        box.prop(scene, "lidar_merge_points")
        if scene.lidar_merge_points:
            box.prop(scene, "lidar_merge_precision", text="Threshold")
            warn = box.box()
            warn.label(text="Merge Points is processed per chunk.", icon='INFO')
            warn.label(text="This is safer for very large objects.")
            if obj and obj.type in {'MESH', 'POINTCLOUD'}:
                approx = len(obj.data.points) if obj.type == 'POINTCLOUD' else len(obj.data.vertices)
                if approx >= 10_000_000:
                    warn.label(text="Large active object: consider smaller Chunk MB.", icon='ERROR')

        if scene.lidar_external_chunks:
            warn = layout.box()
            warn.label(text="External chunks create HTML + data folder.", icon='INFO')
            warn.label(text="Open with generated local server script.")

        layout.separator()
        layout.label(text="Export to HTML Viewer:", icon='WORLD')
        row = layout.row()
        row.enabled = bool(obj and obj.type in {'MESH', 'POINTCLOUD'})
        row.operator("export_scene.lidar_html", text="Export HTML Viewer", icon='EXPORT')
        
        # Collapsible Viewer Defaults below export button
        row = layout.row(align=True)
        icon = 'TRIA_DOWN' if scene.lidar_show_viewer_defaults else 'TRIA_RIGHT'
        row.prop(scene, "lidar_show_viewer_defaults", text="Viewer Defaults", icon=icon, toggle=True)
        
        if scene.lidar_show_viewer_defaults:
            def_box = layout.box()
            def_box.prop(scene, "lidar_default_opacity", text="Opacity")
            def_box.prop(scene, "lidar_default_brightness", text="Brightness")
            def_box.prop(scene, "lidar_default_global_intensity", text="Elevation Tint")
            def_box.prop(scene, "lidar_default_local_intensity", text="Depth Tint")
            def_box.prop(scene, "lidar_default_point_limit_min_pct", text="Start Cutoff %")
            def_box.prop(scene, "lidar_default_point_limit_max_pct", text="End Cutoff %")


classes = (
    EXPORT_OT_lidar_html,
    LIDAR_PT_ExportPanel,
)


def register():
    bpy.types.Scene.lidar_export_colors = bpy.props.BoolProperty(
        name="Export RGB Colors",
        description="Include point colors in the export. Stored compactly as normalized uint8 RGBA.",
        default=False,
    )
    bpy.types.Scene.lidar_export_intensity = bpy.props.BoolProperty(
        name="Export Intensity",
        description="Include point intensity values in the export.",
        default=False,
    )
    bpy.types.Scene.lidar_intensity_format = bpy.props.EnumProperty(
        name="Intensity Format",
        description="Precision used for exported intensity values.",
        items=(
            ('UINT8', "uint8 compact", "Smallest intensity data, enough for display"),
            ('UINT16', "uint16 higher quality", "Better precision with still compact storage"),
            ('FLOAT32', "float32 legacy", "Largest format, mainly for compatibility/debugging"),
        ),
        default='UINT8',
    )
    bpy.types.Scene.lidar_quantize_positions = bpy.props.BoolProperty(
        name="Quantize Positions",
        description="Store point positions as integers. Precision 3 means a 0.001 m grid.",
        default=True,
    )
    bpy.types.Scene.lidar_position_precision = bpy.props.IntProperty(
        name="Precision",
        description="Decimal places for position quantization. 3 = millimeter grid when 1 unit = 1 m.",
        default=3,
        min=0,
        max=6,
    )
    bpy.types.Scene.lidar_optimize_position_compression = bpy.props.BoolProperty(
        name="Optimize Position Compression",
        description="Sort each chunk by Morton/Z-order and store quantized positions as deltas before zlib compression. Usually smaller, but slower.",
        default=True,
    )
    bpy.types.Scene.lidar_parallel_chunk_compression = bpy.props.BoolProperty(
        name="Parallel Chunk Compression",
        description="Encode and compress chunks in parallel worker threads. Workers do not access bpy; they only process NumPy arrays and zlib payloads.",
        default=True,
    )
    bpy.types.Scene.lidar_parallel_workers = bpy.props.IntProperty(
        name="Parallel Workers",
        description="Number of parallel chunk workers. 0 = Auto, usually up to 4 workers to avoid excessive RAM use.",
        default=0,
        min=0,
        max=32,
    )
    bpy.types.Scene.lidar_chunked_export = bpy.props.BoolProperty(
        name="Chunked Loading",
        description="Split point data into chunks loaded progressively by the browser. Forced on for very large exports.",
        default=True,
    )
    bpy.types.Scene.lidar_spatial_chunking = bpy.props.BoolProperty(
        name="Spatial Chunking",
        description="Legacy option. Multi-object sequential export keeps room/object order and uses optional Morton sorting inside each chunk for compression.",
        default=False,
    )
    bpy.types.Scene.lidar_chunk_target_mb = bpy.props.IntProperty(
        name="Chunk Target MB",
        description="Approximate uncompressed source chunk size. 16-32 MB is recommended when Optimize Position Compression is enabled.",
        default=32,
        min=5,
        max=100,
    )
    bpy.types.Scene.lidar_compression_level = bpy.props.IntProperty(
        name="Compression Level",
        description="Zlib compression level. 6 is a good speed/size compromise; 9 is smaller but slower.",
        default=6,
        min=0,
        max=9,
    )
    bpy.types.Scene.lidar_external_chunks = bpy.props.BoolProperty(
        name="External Chunk Files",
        description="Write chunks into a data folder instead of embedding everything in one HTML. Forced on for exports above 100M points.",
        default=False,
    )
    bpy.types.Scene.lidar_generate_preview_lod = bpy.props.BoolProperty(
        name="Generate 1% Preview LOD",
        description="Create a lightweight preview model with about 1% of points quantized to precision 2 (0.01 m). It loads before the full cloud in the HTML viewer.",
        default=True,
    )
    bpy.types.Scene.lidar_merge_points = bpy.props.BoolProperty(
        name="Merge Points",
        description="Remove duplicate points per processed chunk. Points closer than Merge Precision are merged.",
        default=False,
    )
    bpy.types.Scene.lidar_merge_precision = bpy.props.FloatProperty(
        name="Merge Precision",
        description="Distance threshold for point merging. 0.1 = preview, 0.01 = good balance, 0.001 = fine.",
        default=0.01,
        min=0.0001,
        max=0.1,
        step=0.0001,
        precision=4,
    )
    # Default Viewer properties
    bpy.types.Scene.lidar_show_viewer_defaults = bpy.props.BoolProperty(
        name="Show Viewer Defaults",
        description="Expand or collapse the viewer default settings panel",
        default=False,
    )
    bpy.types.Scene.lidar_default_opacity = bpy.props.FloatProperty(
        name="Opacity",
        description="Initial point cloud opacity in the HTML viewer",
        default=1.0,
        min=0.0,
        max=1.0,
    )
    bpy.types.Scene.lidar_default_brightness = bpy.props.FloatProperty(
        name="Brightness",
        description="Initial brightness adjust in the HTML viewer",
        default=0.0,
        min=-1.0,
        max=1.0,
    )
    bpy.types.Scene.lidar_default_global_intensity = bpy.props.FloatProperty(
        name="Elevation Tint",
        description="Initial elevation tint intensity in the HTML viewer",
        default=0.0,
        min=0.0,
        max=1.0,
    )
    bpy.types.Scene.lidar_default_local_intensity = bpy.props.FloatProperty(
        name="Depth Tint",
        description="Initial depth tint intensity in the HTML viewer",
        default=0.0,
        min=0.0,
        max=1.0,
    )
    bpy.types.Scene.lidar_default_point_limit_min_pct = bpy.props.IntProperty(
        name="Start Cutoff %",
        description="Initial start cutoff percentage in the HTML viewer",
        default=0,
        min=0,
        max=100,
    )
    bpy.types.Scene.lidar_default_point_limit_max_pct = bpy.props.IntProperty(
        name="End Cutoff %",
        description="Initial end cutoff percentage in the HTML viewer",
        default=100,
        min=0,
        max=100,
    )

    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for prop in (
        "lidar_export_colors",
        "lidar_export_intensity",
        "lidar_intensity_format",
        "lidar_quantize_positions",
        "lidar_position_precision",
        "lidar_optimize_position_compression",
        "lidar_parallel_chunk_compression",
        "lidar_parallel_workers",
        "lidar_chunked_export",
        "lidar_spatial_chunking",
        "lidar_chunk_target_mb",
        "lidar_compression_level",
        "lidar_external_chunks",
        "lidar_generate_preview_lod",
        "lidar_merge_points",
        "lidar_merge_precision",
        "lidar_show_viewer_defaults",
        "lidar_default_opacity",
        "lidar_default_brightness",
        "lidar_default_global_intensity",
        "lidar_default_local_intensity",
        "lidar_default_point_limit_min_pct",
        "lidar_default_point_limit_max_pct",
    ):
        if hasattr(bpy.types.Scene, prop):
            delattr(bpy.types.Scene, prop)
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
