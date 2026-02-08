"""
MariSense: A Multimodal Dataset for Environmental Perception in Maritime Autonomy

This package provides tools and utilities for working with the MariSense dataset.
"""

__version__ = "1.0.0"
__author__ = "[Author Name]"
__email__ = "[author@email.com]"
__license__ = "MIT"

from . import loaders
from . import visualization
from . import evaluation
from . import converters
from . import augmentation

__all__ = [
    "loaders",
    "visualization",
    "evaluation",
    "converters",
    "augmentation",
]
