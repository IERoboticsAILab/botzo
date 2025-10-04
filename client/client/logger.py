import logging

import coloredlogs

LOG_LEVEL = "DEBUG"

LEVEL_STYLES = dict(
    spam=dict(color="green", faint=True),
    debug=dict(color="green"),
    verbose=dict(color="blue"),
    info=dict(color="green"),
    notice=dict(color="magenta"),
    warning=dict(color="yellow"),
    success=dict(color="green", bold=True),
    error=dict(color="red"),
    critical=dict(color="red", bold=True),
)

FIELD_STYLES = dict(
    asctime=dict(color="green"),
    hostname=dict(color="magenta"),
    levelname=dict(color="white", bold=True),
    name=dict(color="blue"),
    programname=dict(color="cyan"),
    username=dict(color="yellow"),
    filename=dict(color="cyan"),
)

logger = logging.getLogger("smapper_api")


def get_logger(name: str) -> logging.Logger:
    logger = logging.getLogger(name)
    coloredlogs.install(
        level=LOG_LEVEL,
        logger=logger,
        fmt="[%(name)s] %(levelname)s %(message)s",
        level_styles=LEVEL_STYLES,
        field_styles=FIELD_STYLES,
    )
    return logger
