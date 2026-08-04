"""Compatibility package for the repository flattened vision layout.

The original deployment imports g1_race_vision.*. Keep that public package
name while loading the maintained modules from the parent directory, so
existing VM deployments and clean repository clones use the same code.
"""

from pathlib import Path


_MODULE_ROOT = str(Path(__file__).resolve().parents[1])
if _MODULE_ROOT not in __path__:
    __path__.append(_MODULE_ROOT)
