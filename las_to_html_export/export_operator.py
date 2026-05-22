# SPDX-License-Identifier: GPL-3.0-or-later
"""Export operator for LiDAR HTML Exporter."""

import bpy
import os
import json
import time
import base64
import zlib
import html as html_lib
from concurrent.futures import ThreadPoolExecutor, as_completed
from mathutils import Vector, Matrix
import numpy as np

from bpy_extras.io_utils import ExportHelper

# Use absolute imports for Blender addon compatibility
from las_to_html_export.templates import HTML_TEMPLATE
from las_to_html_export.utils.chunking import (
    calculate_points_per_chunk,
    create_chunk_ranges,
    assign_global_ranges,
)
from las_to_html_export.utils.compression import compress_payload, store_payload, format_bytes
from las_to_html_export.utils.point_processing import (
    deduplicate_points,
    compute_morton_codes,
    lod_sample_indices,
    matrix_to_numpy,
)


class EXPORT_OT_lidar_html(bpy.types.Operator, ExportHelper):
    """Export multiple selected LiDAR point clouds into a single HTML viewer."""

    bl_idname = "export_scene.lidar_html"
    bl_label = "Export to HTML"
    bl_options = {'REGISTER', 'UNDO'}

    filename_ext = ".html"
    filter_glob: bpy.props.StringProperty(
        default="*.html",
        options={'HIDDEN'},
        maxlen=255
    )

    COLOR_ATTR_NAMES = [
        "Col", "color", "Color", "col", "Cd", "rgb", "RGB", "rgba", "RGBA"
    ]
    INTENSITY_ATTR_NAMES = [
        "intensity", "Intensity", "intensities", "Intensities",
        "reflectance", "Reflectance"
    ]

    AUTO_EXTERNAL_LIMIT = 100_000_000

    def __init__(self):
        self._last_console_progress = -1.0
        self._last_console_time = 0.0

    def execute(self, context):
        """Execute the export operation."""
        scene = context.scene
        wm = context.window_manager

        # Get selected objects
        selected = [
            o for o in context.selected_objects
            if o.type in {'MESH', 'POINTCLOUD'}
        ]
        if not selected:
            self.report({'ERROR'}, "Select at least one Mesh or PointCloud object.")
            return {'CANCELLED'}

        selected.sort(key=lambda o: o.name.lower())

        active_obj = context.active_object
        if not active_obj or active_obj not in selected:
            active_obj = selected[0]

        # Calculate origin offset
        origin_offset = (
            active_obj.matrix_world.translation.copy()
            if active_obj else Vector((0, 0, 0))
        )
        origin_np = np.array(
            (origin_offset.x, origin_offset.y, origin_offset.z),
            dtype=np.float64
        )

        # Estimate total points
        object_point_estimates = [self._count_points(obj) for obj in selected]
        total_points_est = int(sum(object_point_estimates))
        if total_points_est <= 0:
            self.report({'ERROR'}, "Selected objects contain no source points.")
            return {'CANCELLED'}

        # Get settings
        storage_external = bool(scene.lidar_external_chunks)
        chunked_export = bool(scene.lidar_chunked_export)
        spatial_chunking = bool(scene.lidar_spatial_chunking)

        # Auto-enable external chunks for very large exports
        if total_points_est >= self.AUTO_EXTERNAL_LIMIT:
            if not storage_external:
                self.report(
                    {'INFO'},
                    f"Total points ({total_points_est:,}) >= 100M, "
                    "forcing external chunk files.".replace(',', ' ')
                )
                storage_external = True
            if not chunked_export:
                self.report({'INFO'}, "Large export detected, forcing chunked loading.")
                chunked_export = True
            if spatial_chunking:
                self.report(
                    {'INFO'},
                    "Large multi-object export detected, disabling spatial chunking."
                )
                spatial_chunking = False

        if len(selected) > 1 and spatial_chunking:
            self.report(
                {'INFO'},
                "Multi-object sequential export: disabling spatial chunking "
                "to preserve object/room order."
            )
            spatial_chunking = False

        # Prepare output paths
        title = os.path.splitext(os.path.basename(self.filepath))[0]
        output_dir = os.path.dirname(os.path.abspath(self.filepath)) or os.getcwd()
        data_dir_name = f"{title}_data"
        data_dir_abs = os.path.join(output_dir, data_dir_name)

        if storage_external:
            os.makedirs(data_dir_abs, exist_ok=True)

        # Initialize tracking variables
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

        # LOD chunks
        lod_chunks = []
        lod_chunk_index = 0
        lod_total_points = 0
        lod_total_compressed = 0
        lod_total_uncompressed = 0

        # Export settings
        request_color = bool(scene.lidar_export_colors)
        request_intensity = bool(scene.lidar_export_intensity)
        compression_level = int(scene.lidar_compression_level)
        optimize_positions = bool(scene.lidar_optimize_position_compression)
        parallel_chunks = bool(scene.lidar_parallel_chunk_compression)
        parallel_workers = max(1, int(scene.lidar_parallel_workers) or 4)
        generate_preview_lod = bool(scene.lidar_generate_preview_lod)

        exported_any_color = False
        exported_any_intensity = False
        final_intensity_type = "none"

        # Start progress
        wm.progress_begin(0, total_points_est)
        processed_input_base = 0

        try:
            depsgraph = context.evaluated_depsgraph_get()

            for obj_idx, obj in enumerate(selected):
                total_objects = len(selected)
                mesh_owner = None
                object_est_points = object_point_estimates[obj_idx]
                object_progress_base = processed_input_base

                self.report(
                    {'INFO'},
                    f"Processing object {obj_idx + 1}/{total_objects}: {obj.name}"
                )
                self._update_progress(
                    wm, object_progress_base, total_points_est,
                    obj_idx + 1, total_objects, obj.name,
                    force=True, extra="object start"
                )

                try:
                    eval_obj = obj.evaluated_get(depsgraph)
                    positions, mesh_or_pc, num_points, mesh_owner = (
                        self._extract_geometry(eval_obj, obj)
                    )

                    if num_points <= 0:
                        processed_input_base += object_est_points
                        continue

                    # Extract colors
                    colors_u8 = None
                    if request_color:
                        colors_u8 = self._extract_colors(mesh_or_pc, num_points)
                        if colors_u8 is None:
                            colors_u8 = self._default_colors(num_points)
                        exported_any_color = True

                    # Extract intensity
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

                    # Get transformation matrix
                    matrix_np = matrix_to_numpy(obj.matrix_world)

                    # Process object in chunks
                    stats = self._process_object_in_chunks(
                        obj_name=obj.name,
                        positions_flat=positions,
                        matrix_np=matrix_np,
                        origin_np=origin_np,
                        colors_u8=colors_u8,
                        intensity_array=intensity_array,
                        intensity_type=(
                            object_intensity_type
                            if intensity_array is not None else "none"
                        ),
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
                        progress_callback=(
                            lambda local_done, local_total, source_done,
                                   name=obj.name, idx=obj_idx,
                                   base=object_progress_base:
                                self._update_progress(
                                    wm,
                                    min(total_points_est, base + source_done),
                                    total_points_est,
                                    idx + 1,
                                    total_objects,
                                    name,
                                    chunk_idx=local_done,
                                    total_chunks=local_total,
                                    extra="chunk encoded",
                                )
                        ),
                    )

                    chunk_index += stats.get(
                        "allocated_chunk_count", stats["chunk_count"]
                    )
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

                    # Generate preview LOD
                    if generate_preview_lod:
                        try:
                            sample_indices = lod_sample_indices(num_points, percent=1)
                            if sample_indices.size > 0:
                                pos_view = positions.reshape((-1, 3))
                                lod_positions = np.ascontiguousarray(
                                    pos_view[sample_indices].reshape(-1)
                                )
                                lod_colors = (
                                    np.ascontiguousarray(colors_u8[sample_indices])
                                    if colors_u8 is not None else None
                                )
                                lod_intensity = (
                                    np.ascontiguousarray(intensity_array[sample_indices])
                                    if intensity_array is not None else None
                                )

                                lod_stats = self._process_object_in_chunks(
                                    obj_name=obj.name,
                                    positions_flat=lod_positions,
                                    matrix_np=matrix_np,
                                    origin_np=origin_np,
                                    colors_u8=lod_colors,
                                    intensity_array=lod_intensity,
                                    intensity_type=(
                                        object_intensity_type
                                        if lod_intensity is not None else "none"
                                    ),
                                    compression_level=compression_level,
                                    quantize=True,
                                    precision=2,
                                    optimize_positions=True,
                                    parallel_chunks=parallel_chunks,
                                    parallel_workers=parallel_workers,
                                    chunked=True,
                                    target_mb=min(
                                        int(scene.lidar_chunk_target_mb), 8
                                    ),
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
                                lod_chunk_index += lod_stats.get(
                                    "allocated_chunk_count", lod_stats["chunk_count"]
                                )
                                lod_total_points += lod_stats["point_count"]
                                lod_total_compressed += lod_stats["compressed_bytes"]
                                lod_total_uncompressed += lod_stats["uncompressed_bytes"]

                                del lod_positions
                                del lod_colors
                                del lod_intensity
                                del lod_stats
                        except Exception as lod_exc:
                            self.report(
                                {'WARNING'},
                                f"Preview LOD skipped for '{obj.name}': {lod_exc}"
                            )
                            print(
                                f"[LiDAR Export] WARNING preview LOD skipped: "
                                f"{obj.name}: {lod_exc}",
                                flush=True,
                            )

                    # Cleanup
                    del positions
                    del colors_u8
                    del intensity_array
                    del stats

                except Exception as exc:
                    self.report(
                        {'WARNING'},
                        f"Object '{obj.name}' skipped: {exc}"
                    )
                    print(
                        f"[LiDAR Export] WARNING object skipped: {obj.name}: {exc}",
                        flush=True,
                    )
                finally:
                    if mesh_owner is not None:
                        try:
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

        # Assign global ranges
        total_points = assign_global_ranges(all_chunks)
        lod_total_points = assign_global_ranges(lod_chunks) if lod_chunks else 0

        # Calculate metadata
        center = (global_min + global_max) * 0.5
        radius = float(np.linalg.norm((global_max - global_min) * 0.5))
        compression_ratio = (
            float(total_uncompressed / total_compressed)
            if total_compressed > 0 else 1.0
        )

        # Build LOD list
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

        # Build metadata dictionary
        meta = {
            "formatVersion": 5,
            "title": title,
            "generator": "LiDAR WebGL HTML Exporter - Chunked Quantized 3.8.0",
            "pointCount": int(total_points),
            "chunkCount": len(all_chunks),
            "storage": "external" if storage_external else "embedded",
            "quantized": bool(scene.lidar_quantize_positions),
            "positionEncoding": (
                "delta_morton" if optimize_positions else "absolute"
            ),
            "precision": int(scene.lidar_position_precision),
            "scale": (
                int(10 ** scene.lidar_position_precision)
                if scene.lidar_quantize_positions else 1
            ),
            "unit": "meter",
            "hasColor": bool(request_color and exported_any_color),
            "colorType": (
                "rgba8_norm" if (request_color and exported_any_color) else "none"
            ),
            "hasIntensity": bool(request_intensity and exported_any_intensity),
            "intensityType": (
                final_intensity_type
                if (request_intensity and exported_any_intensity) else "none"
            ),
            "bounds": {
                "min": [float(v) for v in global_min],
                "max": [float(v) for v in global_max],
                "center": [float(v) for v in center],
                "radius": radius,
            },
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
            "uncompressedBytesByKind": {
                k: int(v) for k, v in uncompressed_by_kind.items()
            },
            "posTypeChunkCounts": pos_type_counts,
            "posEncodingChunkCounts": pos_encoding_counts,
            "lods": lods,
            "chunks": all_chunks,
        }

        # Extract cameras
        cameras_data = self._extract_cameras(context, active_obj, origin_offset)

        # Generate HTML content
        html_content = self._generate_html(title, meta, cameras_data)

        # Write HTML file
        with open(self.filepath, 'w', encoding='utf-8') as f:
            f.write(html_content)

        # Write server helper scripts if using external chunks
        if storage_external:
            self._write_server_helpers(output_dir, os.path.basename(self.filepath))

        # Report completion
        size_hint = format_bytes(os.path.getsize(self.filepath))
        print(
            f"[LiDAR Export] DONE 100.00% | objects {len(selected)} | "
            f"points {total_points:,} | chunks {len(all_chunks)} | "
            f"compressed {format_bytes(total_compressed)} | "
            f"ratio {compression_ratio:.2f}:1".replace(',', ' '),
            flush=True,
        )
        self.report(
            {'INFO'},
            f"Exported {len(selected)} objects, {total_points:,} points, "
            f"{len(all_chunks)} chunks. HTML: {size_hint}. "
            f"Data compression ratio: {compression_ratio:.2f}:1".replace(',', ' ')
        )

        return {'FINISHED'}

    def _generate_html(self, title: str, meta: dict, cameras_data: list) -> str:
        """Generate HTML content with embedded metadata."""
        html_content = HTML_TEMPLATE
        html_content = html_content.replace("__TITLE_HTML__", html_lib.escape(title))
        html_content = html_content.replace(
            "__FILE_NAME_JSON__",
            json.dumps(title)
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

    def _write_server_helpers(self, output_dir: str, html_filename: str) -> None:
        """Write helper scripts for starting a local server."""
        bat_path = os.path.join(output_dir, "start_lidar_viewer_windows.bat")
        with open(bat_path, 'w', encoding='utf-8') as f:
            f.write(
                "@echo off\n"
                f'cd /d "%~dp0"\n'
                f'start "" "http://localhost:8000/{html_filename}"\n'
                "python -m http.server 8000\n"
                "pause\n"
            )

        sh_path = os.path.join(output_dir, "start_lidar_viewer_mac_linux.sh")
        with open(sh_path, 'w', encoding='utf-8') as f:
            f.write(
                "#!/bin/sh\n"
                f'cd "$(dirname "$0")"\n'
                f"echo Open: http://localhost:8000/{html_filename}\n"
                "python3 -m http.server 8000\n"
            )

        try:
            os.chmod(sh_path, 0o755)
        except Exception:
            pass

    def _count_points(self, obj) -> int:
        """Count points in an object."""
        try:
            if obj.type == 'POINTCLOUD':
                return len(obj.data.points)
            if obj.type == 'MESH':
                return len(obj.data.vertices)
        except Exception:
            return 0
        return 0

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

    def _default_colors(self, num_points: int) -> np.ndarray:
        """Create default white colors."""
        colors = np.empty((num_points, 4), dtype=np.uint8)
        colors[:, 0:3] = 255
        colors[:, 3] = 255
        return colors

    def _default_intensity(
        self, num_points: int, mode: str
    ) -> tuple[np.ndarray, str]:
        """Create default zero intensity values."""
        if mode == 'UINT16':
            return np.zeros(num_points, dtype=np.uint16), "uint16_norm"
        if mode == 'FLOAT32':
            return np.zeros(num_points, dtype=np.float32), "float32"
        return np.zeros(num_points, dtype=np.uint8), "uint8_norm"

    def _extract_colors(self, meshorpc, numpoints: int) -> np.ndarray | None:
        """Extract color attributes from mesh or point cloud."""
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
            preferred = [n.lower() for n in self.COLOR_ATTR_NAMES]

            color_attrs = getattr(meshorpc, "color_attributes", None)
            if color_attrs:
                for name in self.COLOR_ATTR_NAMES:
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
                for name in self.COLOR_ATTR_NAMES:
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

        raw = read_color_data(attr)
        if raw is None:
            return None

        colors = np.empty((numpoints, 4), dtype=np.uint8)
        colors[:, 0] = np.clip(np.rint(raw[:, 0] * 255), 0, 255).astype(np.uint8)
        colors[:, 1] = np.clip(np.rint(raw[:, 1] * 255), 0, 255).astype(np.uint8)
        colors[:, 2] = np.clip(np.rint(raw[:, 2] * 255), 0, 255).astype(np.uint8)
        colors[:, 3] = (
            np.clip(np.rint(raw[:, 3] * 255), 0, 255).astype(np.uint8)
            if raw.shape[1] >= 4 else np.uint8(255)
        )
        return colors

    def _find_attr(self, mesh_or_pc, candidate_names):
        """Find an attribute by candidate names."""
        attrs = mesh_or_pc.attributes
        for name in candidate_names:
            if name in attrs:
                return attrs[name]
        return None

    def _extract_intensity(
        self, meshorpc, numpoints: int, mode: str
    ) -> tuple[np.ndarray | None, str]:
        """Extract intensity attributes."""
        attr = self._find_attr(meshorpc, self.INTENSITY_ATTR_NAMES)
        if attr is None:
            return None, "none"

        try:
            arr = np.empty(numpoints, dtype=np.float32)
            attr.data.foreach_get("value", arr)

            if mode == 'UINT8':
                out_type = "uint8_norm"
                arr = np.clip(np.rint(arr * 255.0), 0, 255).astype(np.uint8)
            elif mode == 'UINT16':
                out_type = "uint16_norm"
                arr = np.clip(np.rint(arr * 65535.0), 0, 65535).astype(np.uint16)
            else:
                out_type = "float32"

            return arr, out_type
        except Exception:
            return None, "none"

    def _extract_cameras(self, context, cloud_obj, origin_offset=None):
        """Extract camera data from scene."""
        cameras_data = []
        scene = context.scene
        cams = [o for o in scene.objects if o.type == 'CAMERA']
        active = scene.camera if scene.camera and scene.camera in cams else None

        if active:
            cams = (
                [active] +
                sorted([c for c in cams if c != active], key=lambda c: c.name.lower())
            )
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

    def _update_progress(
        self, wm, processed, total, obj_idx, total_objects, obj_name,
        chunk_idx=None, total_chunks=None, force=False, extra=""
    ):
        """Update progress bar and console output."""
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
                f"{chunk_txt} | source points {processed:,}/{total:,}{extra_txt}".replace(
                    ',', ' '
                ),
                flush=True,
            )
            self._last_console_progress = pct
            self._last_console_time = now

    def _process_object_in_chunks(
        self, obj_name, positions_flat, matrix_np, origin_np,
        colors_u8, intensity_array, intensity_type,
        compression_level, quantize, precision, optimize_positions,
        parallel_chunks, parallel_workers,
        chunked, target_mb, merge_points, merge_precision,
        storage_external, data_dir_name, data_dir_abs,
        start_chunk_index, all_chunks, file_prefix="chunk", progress_callback=None
    ):
        """Process object geometry in chunks."""
        source_points = int(len(positions_flat) // 3)
        ppc = calculate_points_per_chunk(
            colors_u8 is not None, intensity_array, chunked, target_mb
        )
        if ppc is None:
            ppc = source_points

        chunk_ranges = create_chunk_ranges(source_points, ppc)
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
                f"Merge Points enabled for large object '{obj_name}'. "
                "Deduplication is now per chunk, not whole object."
            )

        use_parallel = bool(
            parallel_chunks and parallel_workers > 1 and total_chunks > 1
        )
        max_in_flight = max(1, int(parallel_workers) * 2)
        completed_chunks = 0
        completed_source_points = 0
        object_results = []

        # Precompute translation vector
        translation_np = matrix_np[:3, 3] - origin_np

        def make_task(local_chunk_idx, start, end):
            local_positions = np.ascontiguousarray(
                positions_flat[start * 3:end * 3]
            )
            color_slice = (
                np.ascontiguousarray(colors_u8[start:end])
                if colors_u8 is not None else None
            )
            intensity_slice = (
                np.ascontiguousarray(intensity_array[start:end])
                if intensity_array is not None else None
            )
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
                progress_callback(
                    completed_chunks, total_chunks,
                    min(source_points, completed_source_points)
                )

            if result.get("skipped"):
                return

            object_results.append(result)
            stats["chunk_count"] += 1
            stats["point_count"] += int(result["point_count"])
            stats["compressed_bytes"] += int(result["compressed_bytes"])
            stats["uncompressed_bytes"] += int(result["uncompressed_bytes"])

            for key in stats["compressed_by_kind"]:
                stats["compressed_by_kind"][key] += int(
                    result["compressed_by_kind"][key]
                )
                stats["uncompressed_by_kind"][key] += int(
                    result["uncompressed_by_kind"][key]
                )

            stats["pos_type_counts"][result["pos_type"]] += 1
            stats["pos_encoding_counts"][result["pos_encoding"]] += 1
            stats["bounds_min"] = np.minimum(
                stats["bounds_min"], result["bounds_min"]
            )
            stats["bounds_max"] = np.maximum(
                stats["bounds_max"], result["bounds_max"]
            )

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


def _lidar_float_list(values):
    """Convert values to list of floats."""
    return [float(v) for v in values]


def _lidar_store_payload_worker(
    chunk, key, compressed_bytes, storage_external,
    data_dir_name, data_dir_abs, chunk_index, file_prefix="chunk"
):
    """Store payload either externally or as base64."""
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


def _lidar_deduplicate_points_worker(
    positions3, colors_u8, intensity_array, threshold
):
    """Remove duplicate points based on threshold."""
    threshold = max(float(threshold), 1e-9)
    grid = np.rint(positions3 / threshold).astype(np.int64)
    grid = np.ascontiguousarray(grid)

    void_view = grid.view(
        np.dtype((np.void, grid.dtype.itemsize * grid.shape[1]))
    )
    _, unique_indices = np.unique(void_view, return_index=True)
    unique_indices.sort()

    new_positions = positions3[unique_indices]
    new_colors = colors_u8[unique_indices] if colors_u8 is not None else None
    new_intensity = intensity_array[unique_indices] if intensity_array is not None else None
    return new_positions, new_colors, new_intensity


def _lidar_morton3_10_worker(chunk_pos):
    """Compute Morton codes for chunk positions."""
    c_min = chunk_pos.min(axis=0)
    c_max = chunk_pos.max(axis=0)
    span = np.maximum(c_max - c_min, 1e-9)

    norm = np.floor(((chunk_pos - c_min) / span) * 1023.0).astype(np.uint32)
    x, y, z = norm[:, 0], norm[:, 1], norm[:, 2]

    def part1by2(n):
        n = n & np.uint32(0x000003ff)
        n = (n | (n << np.uint32(16))) & np.uint32(0x030000ff)
        n = (n | (n << np.uint32(8))) & np.uint32(0x0300f00f)
        n = (n | (n << np.uint32(4))) & np.uint32(0x030c30c3)
        n = (n | (n << np.uint32(2))) & np.uint32(0x09249249)
        return n.astype(np.uint64)

    return part1by2(x) | (part1by2(y) << np.uint64(1)) | (part1by2(z) << np.uint64(2))


def _lidar_encode_positions_worker(
    chunk_pos, quantize, precision, optimize_positions
):
    """Encode position data."""
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
        raise RuntimeError(
            "Chunk exceeds uint32 range. Reduce precision or chunk size."
        )

    q_int64 = q_float.astype(np.int64)

    if optimize_positions and count > 1:
        try:
            morton = _lidar_morton3_10_worker(chunk_pos)
            order = np.argsort(morton, kind='stable')
            q_sorted = np.ascontiguousarray(q_int64[order])
            deltas = np.diff(q_sorted, axis=0)
            dmin = int(deltas.min()) if deltas.size else 0
            dmax = int(deltas.max()) if deltas.size else 0

            if (
                dmin >= np.iinfo(np.int16).min and
                dmax <= np.iinfo(np.int16).max
            ):
                delta_dtype = np.int16
                delta_type = "int16"
            elif (
                dmin >= np.iinfo(np.int32).min and
                dmax <= np.iinfo(np.int32).max
            ):
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
    """Worker function to encode a single chunk."""
    local_positions = task["positions_flat"].reshape((-1, 3)).astype(
        np.float64, copy=False
    )
    matrix_np = task["matrix_np"]
    translation_np = task["translation_np"]

    # Transform positions
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
            chunk_pos, col_slice, int_slice, task["merge_precision"]
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

    # Compress positions
    pos_z = zlib.compress(encoded["payload"], level=compression_level)
    pos_comp = len(pos_z)
    _lidar_store_payload_worker(
        chunk, "pos", pos_z, storage_external,
        data_dir_name, data_dir_abs, chunk_index, file_prefix=file_prefix
    )

    compressed_by_kind = {
        "position": pos_comp,
        "color": 0,
        "intensity": 0
    }
    uncompressed_by_kind = {
        "position": encoded["uncompressed_bytes"],
        "color": 0,
        "intensity": 0
    }
    compressed_bytes = pos_comp
    uncompressed_bytes = encoded["uncompressed_bytes"]

    # Compress colors
    if col_slice is not None:
        col_payload = np.ascontiguousarray(col_slice).tobytes()
        col_z = zlib.compress(col_payload, level=compression_level)
        col_comp = len(col_z)
        _lidar_store_payload_worker(
            chunk, "color", col_z, storage_external,
            data_dir_name, data_dir_abs, chunk_index, file_prefix=file_prefix
        )
        compressed_by_kind["color"] = col_comp
        uncompressed_by_kind["color"] = len(col_payload)
        compressed_bytes += col_comp
        uncompressed_bytes += len(col_payload)

    # Compress intensity
    if int_slice is not None:
        int_payload = np.ascontiguousarray(int_slice).tobytes()
        int_z = zlib.compress(int_payload, level=compression_level)
        int_comp = len(int_z)
        _lidar_store_payload_worker(
            chunk, "intensity", int_z, storage_external,
            data_dir_name, data_dir_abs, chunk_index, file_prefix=file_prefix
        )
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


def register():
    """Register export operator."""
    bpy.utils.register_class(EXPORT_OT_lidar_html)


def unregister():
    """Unregister export operator."""
    bpy.utils.unregister_class(EXPORT_OT_lidar_html)