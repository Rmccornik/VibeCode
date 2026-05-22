# SPDX-License-Identifier: GPL-3.0-or-later
"""HTML template for LiDAR viewer."""

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

    <!-- Three.js viewer script will be injected here -->
    <script type="module">
        // Viewer JavaScript code will be included from the original template
        // This is a placeholder - the full JS code from the original file should be inserted here
        console.log("LiDAR Viewer initialized with metadata:", window.PC_META);
    </script>
</body>
</html>
'''


def register():
    """Register template module."""
    pass


def unregister():
    """Unregister template module."""
    pass