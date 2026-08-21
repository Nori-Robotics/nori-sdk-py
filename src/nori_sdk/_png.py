"""Minimal PNG encoding for verifier artifacts — stdlib only, on purpose.

`RemoteTeleop.snapshot()` hands back an rgb24 ndarray; the policy loop's trial
records want PNG files on disk. The obvious `pip install pillow` would be the
first image dependency in a package whose core deliberately has zero runtime
deps, for a job that is one zlib stream and four struct-packed chunks. So:
truecolor 8-bit RGB, filter 0 on every scanline, one IDAT. Nothing else — no
palettes, no alpha, no interlacing. Verifier consumers (VLMs, humans, PIL)
read it like any other PNG; it is simply not a general-purpose encoder.
"""

from __future__ import annotations

import struct
import zlib
from typing import Any

__all__ = ["encode_rgb24"]

_MAGIC = b"\x89PNG\r\n\x1a\n"


def _chunk(tag: bytes, body: bytes) -> bytes:
    return (
        struct.pack("!I", len(body))
        + tag
        + body
        + struct.pack("!I", zlib.crc32(tag + body) & 0xFFFFFFFF)
    )


def encode_rgb24(image: Any) -> bytes:
    """An (H, W, 3) uint8 rgb24 array -> PNG file bytes.

    Accepts exactly what `snapshot(role=...)` returns. Raises ValueError on any
    other shape/dtype rather than guessing — a silently mis-encoded artifact
    would poison a verifier judgment, which is the one consumer this exists for.
    """
    shape = getattr(image, "shape", None)
    if shape is None or len(shape) != 3 or shape[2] != 3:
        raise ValueError(f"expected an (H, W, 3) rgb24 array, got shape {shape}")
    if getattr(image, "dtype", None) is not None and str(image.dtype) != "uint8":
        raise ValueError(f"expected uint8 pixels, got {image.dtype}")
    height, width = int(shape[0]), int(shape[1])
    if height <= 0 or width <= 0:
        raise ValueError(f"empty image: {width}x{height}")

    raw = image.tobytes()
    stride = width * 3
    # filter byte 0 (None) before every scanline — simplest legal stream
    scanlines = b"".join(
        b"\x00" + raw[y * stride : (y + 1) * stride] for y in range(height)
    )
    ihdr = struct.pack("!IIBBBBB", width, height, 8, 2, 0, 0, 0)  # 8-bit, truecolor
    return (
        _MAGIC
        + _chunk(b"IHDR", ihdr)
        + _chunk(b"IDAT", zlib.compress(scanlines, 6))
        + _chunk(b"IEND", b"")
    )
