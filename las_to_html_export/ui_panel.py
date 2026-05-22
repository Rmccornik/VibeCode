# SPDX-License-Identifier: GPL-3.0-or-later
"""UI Panel for LiDAR HTML Exporter."""

import bpy


class LIDARHTML_PT_ExportPanel(bpy.types.Panel):
    """Export panel in the 3D Viewport N-panel."""

    bl_label = "LiDAR HTML WebGL"
    bl_idname = "LIDARHTML_PT_ExportPanel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "LiDAR"

    def draw(self, context):
        """Draw the panel UI."""
        layout = self.layout
        scene = context.scene
        obj = context.active_object

        # Export Data Options
        layout.label(text="Export Data Options:")
        layout.prop(scene, "lidar_export_colors")
        layout.prop(scene, "lidar_export_intensity")
        if scene.lidar_export_intensity:
            layout.prop(scene, "lidar_intensity_format", text="Intensity Format")

        # Compact / Large Clouds settings
        box = layout.box()
        box.label(text="Compact / Large Clouds:", icon='MOD_BUILD')
        box.prop(scene, "lidar_quantize_positions")
        row = box.row()
        row.enabled = scene.lidar_quantize_positions
        row.prop(scene, "lidar_position_precision", text="Precision")

        opt_row = box.row()
        opt_row.enabled = scene.lidar_quantize_positions
        opt_row.prop(scene, "lidar_optimize_position_compression")

        # Parallel Processing
        par_box = box.box()
        par_box.label(text="Parallel Processing:", icon='SETTINGS')
        par_box.prop(scene, "lidar_parallel_chunk_compression")
        row = par_box.row()
        row.enabled = scene.lidar_parallel_chunk_compression
        row.prop(scene, "lidar_parallel_workers", text="Workers")

        # Chunked export options
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

        # Merge Points
        box.prop(scene, "lidar_merge_points")
        if scene.lidar_merge_points:
            box.prop(scene, "lidar_merge_precision", text="Threshold")
            warn = box.box()
            warn.label(text="Merge Points is processed per chunk.", icon='INFO')
            warn.label(text="This is safer for very large objects.")
            if obj and obj.type in {'MESH', 'POINTCLOUD'}:
                approx = (
                    len(obj.data.points) if obj.type == 'POINTCLOUD'
                    else len(obj.data.vertices)
                )
                if approx >= 10_000_000:
                    warn.label(
                        text="Large active object: consider smaller Chunk MB.",
                        icon='ERROR'
                    )

        if scene.lidar_external_chunks:
            warn = layout.box()
            warn.label(
                text="External chunks create HTML + data folder.",
                icon='INFO'
            )
            warn.label(text="Open with generated local server script.")

        layout.separator()

        # Export button
        layout.label(text="Export to HTML Viewer:", icon='WORLD')
        row = layout.row()
        row.enabled = bool(obj and obj.type in {'MESH', 'POINTCLOUD'})
        row.operator(
            "export_scene.lidar_html",
            text="Export HTML Viewer",
            icon='EXPORT'
        )

        # Collapsible Viewer Defaults
        row = layout.row(align=True)
        icon = 'TRIA_DOWN' if scene.lidar_show_viewer_defaults else 'TRIA_RIGHT'
        row.prop(
            scene,
            "lidar_show_viewer_defaults",
            text="Viewer Defaults",
            icon=icon,
            toggle=True
        )

        if scene.lidar_show_viewer_defaults:
            def_box = layout.box()
            def_box.prop(scene, "lidar_default_opacity", text="Opacity")
            def_box.prop(scene, "lidar_default_brightness", text="Brightness")
            def_box.prop(
                scene,
                "lidar_default_global_intensity",
                text="Elevation Tint"
            )
            def_box.prop(scene, "lidar_default_local_intensity", text="Depth Tint")
            def_box.prop(
                scene,
                "lidar_default_point_limit_min_pct",
                text="Start Cutoff %"
            )
            def_box.prop(
                scene,
                "lidar_default_point_limit_max_pct",
                text="End Cutoff %"
            )


def register():
    """Register panel class."""
    bpy.utils.register_class(LIDARHTML_PT_ExportPanel)


def unregister():
    """Unregister panel class."""
    bpy.utils.unregister_class(LIDARHTML_PT_ExportPanel)