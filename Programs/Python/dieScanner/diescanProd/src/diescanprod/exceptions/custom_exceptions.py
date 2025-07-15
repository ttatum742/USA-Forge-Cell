"""Custom exceptions for die scanner production package."""


class DieScannerError(Exception):
    """Base exception for all die scanner related errors."""
    pass


class ValidationError(DieScannerError):
    """Raised when data validation fails."""
    pass


class SafetyError(DieScannerError):
    """
    Raised when safety validation fails.
    
    Critical: This exception indicates unsafe operating conditions
    that could result in operator injury or death with 2000°F forging parts.
    """
    pass


class CommunicationError(DieScannerError):
    """Raised when robot or sensor communication fails."""
    pass


class CalibrationError(DieScannerError):
    """Raised when sensor calibration fails."""
    pass


class ScanningError(DieScannerError):
    """Raised when scanning operation fails."""
    pass


class InsufficientDataError(SafetyError):
    """Raised when insufficient scan data for safe center calculation."""
    pass


class EdgeDetectionError(DieScannerError):
    """Raised when edge detection fails."""
    pass


class CenterCalculationError(SafetyError):
    """Raised when center calculation fails safety validation."""
    pass


class AngularCoverageError(SafetyError):
    """Raised when angular coverage is insufficient for safe operation."""
    pass


class DiameterValidationError(SafetyError):
    """Raised when calculated diameter is outside safe tolerances."""
    pass


class ConfidenceError(SafetyError):
    """Raised when confidence score is below safety threshold."""
    pass