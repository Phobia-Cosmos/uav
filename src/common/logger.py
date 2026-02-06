#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Logger Module

双位置日志模块，同时输出到文件和journalctl。
"""

import os
import sys
import logging
import logging.handlers
from pathlib import Path
from typing import Optional


class Logger:
    """日志器类"""

    def __init__(
        self,
        name: str = "ground_coop",
        log_file: Optional[str] = None,
        level: str = "INFO",
        max_bytes: int = 10 * 1024 * 1024,
        backup_count: int = 5
    ):
        """
        初始化日志器
        
        Args:
            name: 日志器名称
            log_file: 日志文件路径
            level: 日志级别
            max_bytes: 单个日志文件最大大小
            backup_count: 保留的备份数量
        """
        self.name = name
        self.log_file = log_file or "/tmp/ground_coop.log"
        self.level = getattr(logging, level.upper(), logging.INFO)
        
        log_path = Path(self.log_file)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        
        self.logger = logging.getLogger(name)
        self.logger.setLevel(self.level)
        self.logger.handlers.clear()
        
        self.formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        
        self._setup_file_handler(max_bytes, backup_count)
        self._setup_journal_handler()
        self._setup_console_handler()

    def _setup_file_handler(self, max_bytes: int, backup_count: int):
        try:
            file_handler = logging.handlers.RotatingFileHandler(
                self.log_file, maxBytes=max_bytes,
                backupCount=backup_count, encoding='utf-8'
            )
            file_handler.setLevel(self.level)
            file_handler.setFormatter(self.formatter)
            self.logger.addHandler(file_handler)
        except Exception:
            pass

    def _setup_journal_handler(self):
        try:
            if os.environ.get('INVOCATION_ID'):
                from systemd.journal import JournalHandler
                journal_handler = JournalHandler(SYSLOG_IDENTIFIER=self.name)
                journal_handler.setLevel(self.level)
                journal_handler.setFormatter(self.formatter)
                self.logger.addHandler(journal_handler)
        except ImportError:
            pass
        except Exception:
            pass

    def _setup_console_handler(self):
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(self.level)
        console_handler.setFormatter(self.formatter)
        self.logger.addHandler(console_handler)

    def debug(self, msg: str, *args, **kwargs):
        self.logger.debug(msg, *args, **kwargs)

    def info(self, msg: str, *args, **kwargs):
        self.logger.info(msg, *args, **kwargs)

    def warning(self, msg: str, *args, **kwargs):
        self.logger.warning(msg, *args, **kwargs)

    def error(self, msg: str, *args, **kwargs):
        self.logger.error(msg, *args, **kwargs)

    def critical(self, msg: str, *args, **kwargs):
        self.logger.critical(msg, *args, **kwargs)

    def exception(self, msg: str, *args, **kwargs):
        self.logger.exception(msg, *args, **kwargs)

    def log(self, level: int, msg: str, *args, **kwargs):
        self.logger.log(level, msg, *args, **kwargs)

    def set_level(self, level: str):
        self.level = getattr(logging, level.upper(), logging.INFO)
        self.logger.setLevel(self.level)

    def get_logger(self) -> logging.Logger:
        return self.logger

    @property
    def log_file_path(self) -> str:
        return self.log_file


_global_logger: Optional[Logger] = None


def get_logger(
    name: str = "ground_coop",
    log_file: Optional[str] = None,
    level: str = "INFO"
) -> Logger:
    global _global_logger
    
    if _global_logger is None:
        _global_logger = Logger(name, log_file, level)
    else:
        _global_logger.set_level(level)
    
    return _global_logger


def reset_logger():
    global _global_logger
    _global_logger = None


if __name__ == "__main__":
    logger = get_logger("test", "/tmp/test_ground_coop.log", "DEBUG")
    logger.debug("Debug message")
    logger.info("Info message")
    logger.warning("Warning message")
    logger.error("Error message")
    print(f"\nLog file: {logger.log_file_path}")
