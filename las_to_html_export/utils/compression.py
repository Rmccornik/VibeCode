# SPDX-License-Identifier: GPL-3.0-or-later
"""Compression utilities for LiDAR HTML Exporter."""

import zlib
import base64
import os
import numpy as np


def compress_payload(data: bytes, level: int = 6) -> bytes:
    """Compress data using zlib.
    
    Args:
        data: Raw bytes to compress.
        level: Compression level (0-9).
        
    Returns:
        Compressed bytes.
    """
    return zlib.compress(data, level=level)


def encode_to_base64(data: bytes) -> str:
    """Encode bytes to base64 string.
    
    Args:
        data: Bytes to encode.
        
    Returns:
        Base64 encoded string.
    """
    return base64.b64encode(data).decode('ascii')


def store_payload(
    chunk: dict,
    key: str,
    compressed_bytes: bytes,
    storage_external: bool,
    data_dir_name: str,
    data_dir_abs: str,
    chunk_index: int,
    file_prefix: str = "chunk"
) -> None:
    """Store compressed payload either as external file or base64 in chunk.
    
    Args:
        chunk: Chunk dictionary to update.
        key: Data type key ('pos', 'color', 'intensity').
        compressed_bytes: Compressed data bytes.
        storage_external: Whether to save as external file.
        data_dir_name: Name of data directory (relative).
        data_dir_abs: Absolute path to data directory.
        chunk_index: Index of the chunk.
        file_prefix: Prefix for external filenames.
    """
    if storage_external:
        suffix = {"pos": "pos", "color": "col", "intensity": "int"}[key]
        safe_prefix = file_prefix or "chunk"
        filename = f"{safe_prefix}_{chunk_index:05d}.{suffix}.z"
        abs_path = os.path.join(data_dir_abs, filename)
        
        with open(abs_path, 'wb') as f:
            f.write(compressed_bytes)
        
        chunk[f"{key}File"] = f"{data_dir_name}/{filename}"
    else:
        chunk[f"{key}B64"] = encode_to_base64(compressed_bytes)


def format_bytes(n: int | float) -> str:
    """Format byte count to human-readable string.
    
    Args:
        n: Number of bytes.
        
    Returns:
        Formatted string (e.g., '1.5 MB').
    """
    n = float(n)
    for unit in ['B', 'KB', 'MB', 'GB']:
        if n < 1024.0:
            return f"{n:.1f} {unit}"
        n /= 1024.0
    return f"{n:.1f} TB"


def register():
    """Register compression module."""
    pass


def unregister():
    """Unregister compression module."""
    pass