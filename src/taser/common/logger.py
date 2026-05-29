import logging

logger = logging.getLogger("TASER")
logger.setLevel(logging.INFO)

if not logger.handlers:
	handler = logging.StreamHandler()
	handler.setFormatter(logging.Formatter("[TASER] %(message)s"))
	logger.addHandler(handler)

logger.propagate = False
