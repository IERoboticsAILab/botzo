"""Logging utilities for the robotics control library.

This module provides a centralized logging system that can be customized
by library users.
"""

import logging
from typing import Any, Callable, Dict, Optional

import coloredlogs


class LoggerManager:
    """Centralized logging manager for the library.

    Provides consistent logging across the library with support for
    custom logger injection.
    """

    _instance: Optional["LoggerManager"] = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return

        self._loggers: Dict[str, logging.Logger] = {}
        self._custom_logger_factory: Optional[Callable[[str], logging.Logger]] = None
        self._default_level = logging.INFO
        self._use_colors = True
        self._log_format = "[%(name)s] %(levelname)s %(message)s"

        self._level_styles = {
            "debug": {"color": "green"},
            "info": {"color": "cyan"},
            "warning": {"color": "yellow"},
            "error": {"color": "red"},
            "critical": {"color": "red", "bold": True},
        }

        self._field_styles = {
            "levelname": {"color": "white", "bold": True},
            "name": {"color": "blue"},
        }

        self._initialized = True

    def configure(
        self,
        level: int = logging.INFO,
        use_colors: bool = True,
        log_format: Optional[str] = None,
        level_styles: Optional[Dict[str, Dict[str, Any]]] = None,
        field_styles: Optional[Dict[str, Dict[str, Any]]] = None,
    ) -> None:
        """Configure global logging settings.

        Args:
            level: Default logging level.
            use_colors: Enable colored output.
            log_format: Custom log format string.
            level_styles: Custom level color styles.
            field_styles: Custom field color styles.
        """
        self._default_level = level
        self._use_colors = use_colors

        if log_format:
            self._log_format = log_format
        if level_styles:
            self._level_styles = level_styles
        if field_styles:
            self._field_styles = field_styles

        # Reconfigure existing loggers
        for logger in self._loggers.values():
            self._apply_configuration(logger)

    def set_logger_factory(self, factory: Callable[[str], logging.Logger]) -> None:
        """Set a custom logger factory.

        Args:
            factory: Function that takes a name and returns a logger.
        """
        self._custom_logger_factory = factory
        self._loggers.clear()

    def get_logger(self, name: str) -> logging.Logger:
        """Get or create a logger.

        Args:
            name: Logger name, typically __name__.

        Returns:
            Configured logger instance.
        """
        if name in self._loggers:
            return self._loggers[name]

        if self._custom_logger_factory:
            logger = self._custom_logger_factory(name)
        else:
            logger = logging.getLogger(name)
            self._apply_configuration(logger)

        self._loggers[name] = logger
        return logger

    def _apply_configuration(self, logger: logging.Logger) -> None:
        """Apply default configuration to a logger."""
        if logger.handlers:
            return

        logger.setLevel(self._default_level)

        if self._use_colors:
            coloredlogs.install(
                level=self._default_level,
                logger=logger,
                fmt=self._log_format,
                level_styles=self._level_styles,
                field_styles=self._field_styles,
            )
        else:
            handler = logging.StreamHandler()
            handler.setLevel(self._default_level)
            formatter = logging.Formatter(self._log_format)
            handler.setFormatter(formatter)
            logger.addHandler(handler)

    def set_level(self, level: int, logger_name: Optional[str] = None) -> None:
        """Change logging level at runtime."""
        self._default_level = level

        if logger_name:
            if logger_name in self._loggers:
                self._loggers[logger_name].setLevel(level)
        else:
            for logger in self._loggers.values():
                logger.setLevel(level)


# Global singleton
logger_manager = LoggerManager()


def get_logger(name: str) -> logging.Logger:
    """Convenience function to get a logger.

    Args:
        name: Logger name, typically __name__.

    Returns:
        Configured logger instance.
    """
    return logger_manager.get_logger(name)


__all__ = ["LoggerManager", "logger_manager", "get_logger"]
