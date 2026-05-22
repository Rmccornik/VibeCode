# SPDX-License-Identifier: GPL-3.0-or-later
"""HTML generator utilities for LiDAR HTML Exporter."""

import json
import html as html_lib
import os

# Use absolute imports for Blender addon compatibility
from las_to_html_export.templates import HTML_TEMPLATE


def generate_html_content(
    title: str,
    meta: dict,
    cameras_data: list[dict],
    filepath: str
) -> str:
    """Generate the final HTML content for the viewer.
    
    Args:
        title: Title for the HTML document.
        meta: Metadata dictionary for the point cloud.
        cameras_data: List of camera dictionaries.
        filepath: Output file path (used for filename reference).
        
    Returns:
        Complete HTML content as string.
    """
    html_content = HTML_TEMPLATE
    
    # Replace placeholders
    html_content = html_content.replace("__TITLE_HTML__", html_lib.escape(title))
    html_content = html_content.replace(
        "__FILE_NAME_JSON__",
        json.dumps(os.path.basename(filepath))
    )
    html_content = html_content.replace(
        "__PC_META_JSON__",
        json.dumps(meta, separators=(',', ':'))
    )
    html_content = html_content.replace(
        "__CAMERAS_JSON__",
        json.dumps(cameras_data, separators=(',', ':'))
    )
    
    return html_content


def write_server_helpers(output_dir: str, html_filename: str) -> None:
    """Write helper scripts for starting a local server.
    
    Args:
        output_dir: Directory to write scripts to.
        html_filename: Name of the generated HTML file.
    """
    # Windows batch script
    bat_path = os.path.join(output_dir, "start_lidar_viewer_windows.bat")
    with open(bat_path, 'w', encoding='utf-8') as f:
        f.write(
            "@echo off\n"
            f'cd /d "%~dp0"\n'
            f'start "" "http://localhost:8000/{html_filename}"\n'
            "python -m http.server 8000\n"
            "pause\n"
        )
    
    # Unix shell script
    sh_path = os.path.join(output_dir, "start_lidar_viewer_mac_linux.sh")
    with open(sh_path, 'w', encoding='utf-8') as f:
        f.write(
            "#!/bin/sh\n"
            f'cd "$(dirname "$0")"\n'
            f"echo Open: http://localhost:8000/{html_filename}\n"
            "python3 -m http.server 8000\n"
        )
    
    # Make shell script executable
    try:
        os.chmod(sh_path, 0o755)
    except Exception:
        pass


def float_list(values) -> list[float]:
    """Convert iterable to list of floats.
    
    Args:
        values: Iterable of numeric values.
        
    Returns:
        List of floats.
    """
    return [float(v) for v in values]


def register():
    """Register HTML generator module."""
    pass


def unregister():
    """Unregister HTML generator module."""
    pass