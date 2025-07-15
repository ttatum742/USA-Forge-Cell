"""
DieScanner Production Package

Production-ready die scanning system for FANUC robot forging cell.
Provides high-precision die center detection using Keyence laser sensor.

Safety Critical: This package is designed for 2000°F forging operations.
All calculations must be validated before robot communication.
"""

from .core.scanner import DieScannerProd
from .models.data_models import ScanPoint, EdgePoint, DieCenter
from .exceptions.custom_exceptions import DieScannerError, ValidationError, SafetyError

__version__ = "1.0.0"
__author__ = "USA Forge Cell Team"

__all__ = [
    "DieScannerProd",
    "ScanPoint", 
    "EdgePoint",
    "DieCenter",
    "DieScannerError",
    "ValidationError", 
    "SafetyError"
]