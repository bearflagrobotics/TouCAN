"""
    ColoredLogger.py

        Custom colored logging library

        Author: Austin chun
        Date: Aug 2022
"""

import logging

import os
if os.name == 'nt': # Windows
    os.system('color')
else: # other (unix)
    pass

class ColoredFormatter(logging.Formatter):

    cyan = "\033[96m"
    yellow = "\033[93m"
    red = "\033[91m"
    bold = "\033[95m"
    reset = "\033[0m"

    simple_format = "%(levelname)s: %(message)s"
    verbose_format = "%(asctime)s %(name)s %(levelname)s: %(message)s"

    VERBOSE_COLOR_FORMATS = {
        logging.DEBUG: cyan + verbose_format + reset,
        logging.INFO: reset + verbose_format + reset,
        logging.WARNING: yellow + verbose_format + reset,
        logging.ERROR: red + verbose_format + reset,
        logging.CRITICAL: bold + verbose_format + reset,
    }

    VERBOSE_FORMATS = {
        logging.DEBUG: verbose_format,
        logging.INFO: verbose_format,
        logging.WARNING: verbose_format,
        logging.ERROR: verbose_format,
        logging.CRITICAL: verbose_format,
    }

    COLOR_FORMATS = {
        logging.DEBUG: cyan  + simple_format + reset,
        logging.INFO: reset  + simple_format + reset,
        logging.WARNING: yellow  + simple_format + reset,
        logging.ERROR: red  + simple_format + reset,
        logging.CRITICAL: bold  + simple_format + reset,
    }

    FORMATS = {
        logging.DEBUG: simple_format,
        logging.INFO: simple_format,
        logging.WARNING: simple_format,
        logging.ERROR: simple_format,
        logging.CRITICAL: simple_format,
    }


    def __init__(self, verbose=False, color=False):
        super().__init__()

        if verbose and color:
            self.formats = self.VERBOSE_COLOR_FORMATS
        elif verbose:
            self.formats = self.VERBOSE_FORMATS
        elif color:
            self.formats = self.COLOR_FORMATS
        else:
            self.formats = self.FORMATS

    def format(self, record):
        log_fmt = self.formats.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)

class CustomLogger(logging.Logger):

    def __init__(self, name, level=logging.DEBUG, filename=None, verbose=False, color=False):
        logging.Logger.__init__(self, name, level)
        self.setLevel(level)

        if filename:
            file_handler = logging.FileHandler(filename)
            file_handler.setFormatter(ColoredFormatter(verbose=verbose))
            self.addHandler(file_handler)

        ch = logging.StreamHandler()
        ch.setLevel(level)
        ch.setFormatter(ColoredFormatter(verbose=verbose, color=color))
        self.addHandler(ch)


# ==================================================================================================

if __name__ == "__main__":

    # Example usage
    logger = CustomLogger("TestTest.py", level=logging.DEBUG, verbose=True)
    logger.debug('debug message')
    logger.info('info message')
    logger.warning('warn message')
    logger.error('error message')
    logger.critical('critical message')

    # Example usage
    logger2 = CustomLogger(__file__, color=True)
    logger2.debug('debug message')
    logger2.info('info message')
    logger2.warning('warn message')
    logger2.error('error message')
    logger2.critical('critical message')
