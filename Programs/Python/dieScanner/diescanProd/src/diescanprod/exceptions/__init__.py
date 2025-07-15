"""Custom exception classes for die scanner package."""

from .custom_exceptions import (
    DieScannerError,
    ValidationError,
    SafetyError,
    CommunicationError,
    CalibrationError,
    ScanningError
)

__all__ = [
    "DieScannerError",
    "ValidationError", 
    "SafetyError",
    "CommunicationError",
    "CalibrationError",
    "ScanningError"
]