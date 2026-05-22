# SPDX-License-Identifier: GPL-3.0-or-later
"""Templates package for LiDAR HTML Exporter."""

# Use absolute imports for Blender addon compatibility
from las_to_html_export.templates.html_template import HTML_TEMPLATE

__all__ = ["HTML_TEMPLATE"]


def register():
    """Register templates module."""
    pass


def unregister():
    """Unregister templates module."""
    pass