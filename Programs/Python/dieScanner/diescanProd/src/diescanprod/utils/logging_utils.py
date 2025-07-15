"""Logging utilities for die scanner production package."""

import logging
import sys
from typing import Optional


def setup_logger(name: str, level: str = "INFO", 
                format_string: Optional[str] = None) -> logging.Logger:
    """
    Setup logger for production use with operator-friendly output.
    
    Args:
        name: Logger name
        level: Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        format_string: Custom format string
    
    Returns:
        Configured logger instance
    """
    logger = logging.getLogger(name)
    
    # Clear existing handlers to avoid duplicates
    logger.handlers.clear()
    
    # Set level
    numeric_level = getattr(logging, level.upper(), logging.INFO)
    logger.setLevel(numeric_level)
    
    # Create console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(numeric_level)
    
    # Create formatter - simple for production operators
    if format_string is None:
        if level.upper() == "DEBUG":
            format_string = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
        else:
            # Simplified format for operators
            format_string = "%(levelname)s: %(message)s"
    
    formatter = logging.Formatter(format_string)
    console_handler.setFormatter(formatter)
    
    # Add handler to logger
    logger.addHandler(console_handler)
    
    # Prevent propagation to avoid duplicate messages
    logger.propagate = False
    
    return logger


def log_scan_start(logger: logging.Logger) -> None:
    """Log scan start with operator-friendly message."""
    logger.info("DIE SCAN STARTING - Robot ready for scanning sequence")


def log_scan_progress(logger: logging.Logger, phase: str, progress: str) -> None:
    """Log scan progress for operators."""
    logger.info(f"SCAN PROGRESS - {phase}: {progress}")


def log_scan_success(logger: logging.Logger, center_x: float, center_y: float, confidence: float) -> None:
    """Log successful scan completion."""
    logger.info(f"SCAN SUCCESS - Die center: [{center_x:.1f}, {center_y:.1f}] - Confidence: {confidence:.1f}%")


def log_scan_failure(logger: logging.Logger, error_message: str) -> None:
    """Log scan failure with operator-friendly message."""
    logger.error(f"SCAN FAILED - {error_message}")
    logger.error("OPERATOR ACTION REQUIRED - Check die position and retry scan")


def log_safety_violation(logger: logging.Logger, violation: str) -> None:
    """Log safety violation with critical alert."""
    logger.critical(f"SAFETY VIOLATION - {violation}")
    logger.critical("SCAN ABORTED - Do not proceed with robot operation")


def log_calibration_status(logger: logging.Logger, success: bool, details: str = "") -> None:
    """Log calibration status."""
    if success:
        logger.info(f"CALIBRATION SUCCESS - {details}")
    else:
        logger.error(f"CALIBRATION FAILED - {details}")


def log_system_status(logger: logging.Logger, component: str, status: str) -> None:
    """Log system component status."""
    logger.info(f"SYSTEM STATUS - {component}: {status}")


def suppress_debug_output() -> None:
    """Suppress debug output from dependencies for cleaner operator interface."""
    # Suppress CPPPO debug output
    logging.getLogger("cpppo").setLevel(logging.WARNING)
    
    # Suppress serial communication debug
    logging.getLogger("serial").setLevel(logging.WARNING)
    
    # Suppress numpy warnings
    import warnings
    warnings.filterwarnings("ignore", category=RuntimeWarning)