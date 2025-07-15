"""Custom exceptions for robot logger package."""


class LoggerError(Exception):
    """Base exception for logger package."""
    pass


class CommunicationError(LoggerError):
    """Raised when robot communication fails."""
    pass


class ConfigurationError(LoggerError):
    """Raised when logger configuration is invalid.""" 
    pass


class FileWriteError(LoggerError):
    """Raised when CSV file writing fails."""
    pass