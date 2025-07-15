"""Custom exceptions for logger package."""

from .custom_exceptions import LoggerError, CommunicationError, ConfigurationError, FileWriteError

__all__ = ["LoggerError", "CommunicationError", "ConfigurationError", "FileWriteError"]