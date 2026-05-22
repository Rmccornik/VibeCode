# SPDX-License-Identifier: GPL-3.0-or-later
"""Chunking utilities for LiDAR HTML Exporter."""

import numpy as np


def calculate_points_per_chunk(
    has_color: bool,
    intensity_array: np.ndarray | None,
    chunked: bool,
    target_mb: int
) -> int | None:
    """Calculate the number of points per chunk based on target size.
    
    Args:
        has_color: Whether color data is included.
        intensity_array: Intensity array if present.
        chunked: Whether chunked export is enabled.
        target_mb: Target chunk size in megabytes.
        
    Returns:
        Number of points per chunk, or None if not chunked.
    """
    bytes_per_point_est = 12  # Base: 3 * float32 for positions
    
    if has_color:
        bytes_per_point_est += 4  # RGBA uint8
    
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


def create_chunk_ranges(total_points: int, points_per_chunk: int) -> list[tuple[int, int]]:
    """Create ranges for chunking point data.
    
    Args:
        total_points: Total number of points to chunk.
        points_per_chunk: Number of points per chunk.
        
    Returns:
        List of (start, end) tuples for each chunk.
    """
    return [
        (start, min(start + points_per_chunk, total_points))
        for start in range(0, total_points, points_per_chunk)
    ]


def assign_global_ranges(chunks: list[dict]) -> int:
    """Assign global start/end indices to chunks.
    
    Args:
        chunks: List of chunk dictionaries to update.
        
    Returns:
        Total point count across all chunks.
    """
    cursor = 0
    for sequence_index, chunk in enumerate(chunks):
        count = int(chunk.get("count", 0))
        chunk["sequenceIndex"] = int(sequence_index)
        chunk["globalStart"] = int(cursor)
        cursor += count
        chunk["globalEnd"] = int(cursor)
    return int(cursor)


def register():
    """Register chunking module."""
    pass


def unregister():
    """Unregister chunking module."""
    pass