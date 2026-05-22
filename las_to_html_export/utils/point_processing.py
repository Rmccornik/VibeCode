# SPDX-License-Identifier: GPL-3.0-or-later
"""Point processing utilities for LiDAR HTML Exporter."""

import numpy as np


def deduplicate_points(
    positions: np.ndarray,
    colors: np.ndarray | None,
    intensity: np.ndarray | None,
    threshold: float
) -> tuple[np.ndarray, np.ndarray | None, np.ndarray | None]:
    """Remove duplicate points based on distance threshold.
    
    Args:
        positions: Nx3 array of point positions.
        colors: Nx4 array of colors (optional).
        intensity: N array of intensity values (optional).
        threshold: Distance threshold for merging points.
        
    Returns:
        Tuple of (positions, colors, intensity) with duplicates removed.
    """
    threshold = max(float(threshold), 1e-9)
    
    # Scale coordinates to grid cell integers
    grid = np.rint(positions / threshold).astype(np.int64)
    grid = np.ascontiguousarray(grid)
    
    # High-performance deduplication: view rows as void structure
    void_view = grid.view(np.dtype((np.void, grid.dtype.itemsize * grid.shape[1])))
    _, unique_indices = np.unique(void_view, return_index=True)
    unique_indices.sort()  # Keep original order
    
    new_positions = positions[unique_indices]
    new_colors = colors[unique_indices] if colors is not None else None
    new_intensity = intensity[unique_indices] if intensity is not None else None
    
    return new_positions, new_colors, new_intensity


def compute_morton_codes(positions: np.ndarray) -> np.ndarray:
    """Compute Morton/Z-order codes for 3D positions.
    
    Args:
        positions: Nx3 array of positions.
        
    Returns:
        Array of Morton codes.
    """
    c_min = positions.min(axis=0)
    c_max = positions.max(axis=0)
    span = np.maximum(c_max - c_min, 1e-9)
    
    # Normalize to 10-bit integer space (0 to 1023)
    norm = np.floor(((positions - c_min) / span) * 1023.0).astype(np.uint32)
    x, y, z = norm[:, 0], norm[:, 1], norm[:, 2]
    
    def part1by2(n: np.ndarray) -> np.ndarray:
        """Spread bits for Morton encoding."""
        n = n & np.uint32(0x000003ff)
        n = (n | (n << np.uint32(16))) & np.uint32(0x030000ff)
        n = (n | (n << np.uint32(8))) & np.uint32(0x0300f00f)
        n = (n | (n << np.uint32(4))) & np.uint32(0x030c30c3)
        n = (n | (n << np.uint32(2))) & np.uint32(0x09249249)
        return n.astype(np.uint64)
    
    return part1by2(x) | (part1by2(y) << np.uint64(1)) | (part1by2(z) << np.uint64(2))


def lod_sample_indices(num_points: int, percent: float = 1.0) -> np.ndarray:
    """Generate indices for level-of-detail sampling.
    
    Args:
        num_points: Total number of points.
        percent: Percentage of points to sample.
        
    Returns:
        Array of sampled indices.
    """
    num_points = int(num_points)
    if num_points <= 0:
        return np.empty(0, dtype=np.int64)
    
    percent = max(0.0001, min(100.0, float(percent)))
    step = max(1, int(round(100.0 / percent)))
    return np.arange(0, num_points, step, dtype=np.int64)


def matrix_to_numpy(matrix) -> np.ndarray:
    """Convert Blender matrix to NumPy array.
    
    Args:
        matrix: Blender mathutils Matrix.
        
    Returns:
        NumPy array (4x4 float64).
    """
    return np.array(matrix, dtype=np.float64)


def register():
    """Register point processing module."""
    pass


def unregister():
    """Unregister point processing module."""
    pass