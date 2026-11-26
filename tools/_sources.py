"""Enumerate plugin code for reproducible scenario snapshots."""

from pathlib import Path


def source_files(root):
    """Include nested modules and tools without caches, environments or run outputs."""
    root = Path(root)
    files = list(root.glob("*.py"))
    for directory in (
        "planning",
        "scenarios",
        "parking",
        "perception",
        "execution",
        "avlite_signal_perception",
        "tools",
    ):
        files.extend((root / directory).rglob("*.py"))
    return sorted(path for path in files if "__pycache__" not in path.parts)
