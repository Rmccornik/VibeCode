# SPDX-License-Identifier: GPL-3.0-or-later
"""Addon preferences for LiDAR HTML Exporter."""

import bpy
from bpy.types import AddonPreferences
from bpy.props import (
    IntProperty,
    BoolProperty,
    EnumProperty,
)


class LIDARHTML_OT_preferences(AddonPreferences):
    """Global preferences for the LiDAR HTML Exporter addon."""

    bl_idname = __package__

    # Parallel processing settings
    parallel_chunk_compression: BoolProperty(
        name="Parallel Chunk Compression",
        description=(
            "Encode and compress chunks in parallel worker threads. "
            "Workers do not access bpy; they only process NumPy arrays and zlib payloads."
        ),
        default=True,
    )

    parallel_workers: IntProperty(
        name="Parallel Workers",
        description=(
            "Number of parallel chunk workers. 0 = Auto, usually up to 4 workers "
            "to avoid excessive RAM use."
        ),
        default=0,
        min=0,
        max=32,
    )

    # UI defaults
    show_advanced_settings: BoolProperty(
        name="Show Advanced Settings",
        description="Expand or collapse advanced export settings in the panel",
        default=False,
    )

    def draw(self, context):
        """Draw the preferences UI."""
        layout = self.layout

        box = layout.box()
        box.label(text="Parallel Processing:", icon='SETTINGS')
        box.prop(self, "parallel_chunk_compression")
        row = box.row()
        row.enabled = self.parallel_chunk_compression
        row.prop(self, "parallel_workers", text="Workers")

        layout.prop(self, "show_advanced_settings")


def register():
    """Register preferences class."""
    bpy.utils.register_class(LIDARHTML_OT_preferences)


def unregister():
    """Unregister preferences class."""
    bpy.utils.unregister_class(LIDARHTML_OT_preferences)