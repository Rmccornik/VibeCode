# SPDX-License-Identifier: GPL-3.0-or-later

bl_info = {
    "name": "LiDAR WebGL HTML Exporter - Chunked Quantized",
    "author": "AI Assistant, based on user addon",
    "version": (3, 8, 0),
    "blender": (5, 1, 0),
    "location": "View3D > N-Panel > LiDAR",
    "description": (
        "Export multiple LiDAR point cloud objects to a single HTML viewer, "
        "with chunked loading, quantization, delta/Morton compression, global point limiter/cutoff, "
        "preview LOD, parallel chunk workers, auto external chunks for >100M pts, and viewer defaults"
    ),
    "category": "Import-Export",
}

import bpy

from . import properties
from . import preferences
from . import export_operator
from . import ui_panel


def register():
    """Register all addon components."""
    properties.register()
    preferences.register()
    export_operator.register()
    ui_panel.register()


def unregister():
    """Unregister all addon components in reverse order."""
    ui_panel.unregister()
    export_operator.unregister()
    preferences.unregister()
    properties.unregister()


if __name__ == "__main__":
    register()