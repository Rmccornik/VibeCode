# SPDX-License-Identifier: GPL-3.0-or-later
"""Scene properties for LiDAR HTML Exporter."""

import bpy
from bpy.props import (
    BoolProperty,
    EnumProperty,
    IntProperty,
    FloatProperty,
)


def register():
    """Register scene properties."""
    # Export data options
    bpy.types.Scene.lidar_export_colors = BoolProperty(
        name="Export RGB Colors",
        description=(
            "Include point colors in the export. "
            "Stored compactly as normalized uint8 RGBA."
        ),
        default=False,
    )

    bpy.types.Scene.lidar_export_intensity = BoolProperty(
        name="Export Intensity",
        description="Include point intensity values in the export.",
        default=False,
    )

    bpy.types.Scene.lidar_intensity_format = EnumProperty(
        name="Intensity Format",
        description="Precision used for exported intensity values.",
        items=(
            ('UINT8', "uint8 compact", "Smallest intensity data, enough for display"),
            ('UINT16', "uint16 higher quality", "Better precision with still compact storage"),
            ('FLOAT32', "float32 legacy", "Largest format, mainly for compatibility/debugging"),
        ),
        default='UINT8',
    )

    # Compact / Large Clouds settings
    bpy.types.Scene.lidar_quantize_positions = BoolProperty(
        name="Quantize Positions",
        description=(
            "Store point positions as integers. "
            "Precision 3 means a 0.001 m grid."
        ),
        default=True,
    )

    bpy.types.Scene.lidar_position_precision = IntProperty(
        name="Precision",
        description=(
            "Decimal places for position quantization. "
            "3 = millimeter grid when 1 unit = 1 m."
        ),
        default=3,
        min=0,
        max=6,
    )

    bpy.types.Scene.lidar_optimize_position_compression = BoolProperty(
        name="Optimize Position Compression",
        description=(
            "Sort each chunk by Morton/Z-order and store quantized positions as deltas "
            "before zlib compression. Usually smaller, but slower."
        ),
        default=True,
    )

    bpy.types.Scene.lidar_chunked_export = BoolProperty(
        name="Chunked Loading",
        description=(
            "Split point data into chunks loaded progressively by the browser. "
            "Forced on for very large exports."
        ),
        default=True,
    )

    bpy.types.Scene.lidar_spatial_chunking = BoolProperty(
        name="Spatial Chunking",
        description=(
            "Legacy option. Multi-object sequential export keeps room/object order "
            "and uses optional Morton sorting inside each chunk for compression."
        ),
        default=False,
    )

    bpy.types.Scene.lidar_chunk_target_mb = IntProperty(
        name="Chunk Target MB",
        description=(
            "Approximate uncompressed source chunk size. "
            "16-32 MB is recommended when Optimize Position Compression is enabled."
        ),
        default=32,
        min=5,
        max=100,
    )

    bpy.types.Scene.lidar_compression_level = IntProperty(
        name="Compression Level",
        description=(
            "Zlib compression level. "
            "6 is a good speed/size compromise; 9 is smaller but slower."
        ),
        default=6,
        min=0,
        max=9,
    )

    bpy.types.Scene.lidar_external_chunks = BoolProperty(
        name="External Chunk Files",
        description=(
            "Write chunks into a data folder instead of embedding everything in one HTML. "
            "Forced on for exports above 100M points."
        ),
        default=False,
    )

    bpy.types.Scene.lidar_generate_preview_lod = BoolProperty(
        name="Generate 1% Preview LOD",
        description=(
            "Create a lightweight preview model with about 1% of points "
            "quantized to precision 2 (0.01 m). "
            "It loads before the full cloud in the HTML viewer."
        ),
        default=True,
    )

    bpy.types.Scene.lidar_merge_points = BoolProperty(
        name="Merge Points",
        description=(
            "Remove duplicate points per processed chunk. "
            "Points closer than Merge Precision are merged."
        ),
        default=False,
    )

    bpy.types.Scene.lidar_merge_precision = FloatProperty(
        name="Merge Precision",
        description=(
            "Distance threshold for point merging. "
            "0.1 = preview, 0.01 = good balance, 0.001 = fine."
        ),
        default=0.01,
        min=0.0001,
        max=0.1,
        step=0.0001,
        precision=4,
    )

    # Viewer defaults panel toggle
    bpy.types.Scene.lidar_show_viewer_defaults = BoolProperty(
        name="Show Viewer Defaults",
        description="Expand or collapse the viewer default settings panel",
        default=False,
    )

    # Default viewer settings
    bpy.types.Scene.lidar_default_opacity = FloatProperty(
        name="Opacity",
        description="Initial point cloud opacity in the HTML viewer",
        default=1.0,
        min=0.0,
        max=1.0,
    )

    bpy.types.Scene.lidar_default_brightness = FloatProperty(
        name="Brightness",
        description="Initial brightness adjust in the HTML viewer",
        default=0.0,
        min=-1.0,
        max=1.0,
    )

    bpy.types.Scene.lidar_default_global_intensity = FloatProperty(
        name="Elevation Tint",
        description="Initial elevation tint intensity in the HTML viewer",
        default=0.0,
        min=0.0,
        max=1.0,
    )

    bpy.types.Scene.lidar_default_local_intensity = FloatProperty(
        name="Depth Tint",
        description="Initial depth tint intensity in the HTML viewer",
        default=0.0,
        min=0.0,
        max=1.0,
    )

    bpy.types.Scene.lidar_default_point_limit_min_pct = IntProperty(
        name="Start Cutoff %",
        description="Initial start cutoff percentage in the HTML viewer",
        default=0,
        min=0,
        max=100,
    )

    bpy.types.Scene.lidar_default_point_limit_max_pct = IntProperty(
        name="End Cutoff %",
        description="Initial end cutoff percentage in the HTML viewer",
        default=100,
        min=0,
        max=100,
    )


def unregister():
    """Unregister scene properties."""
    props_to_remove = [
        "lidar_export_colors",
        "lidar_export_intensity",
        "lidar_intensity_format",
        "lidar_quantize_positions",
        "lidar_position_precision",
        "lidar_optimize_position_compression",
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
    ]

    for prop_name in props_to_remove:
        if hasattr(bpy.types.Scene, prop_name):
            delattr(bpy.types.Scene, prop_name)