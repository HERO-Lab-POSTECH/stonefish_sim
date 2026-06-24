# SPDX-FileCopyrightText: 2016-2019 The UUV Simulator Authors
# SPDX-FileCopyrightText: 2025 Seungmin Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later

LOGGER = None

def get_logger():
    """Get or create the global logger for control interfaces.

    Note: In ROS2, this is primarily used for standalone testing.
    For ROS2 nodes, use node.get_logger() instead.
    """
    global LOGGER
    if LOGGER is None:
        import logging
        import sys

        LOGGER = logging.getLogger('stonefish_control_interfaces')
        out_hdlr = logging.StreamHandler(sys.stdout)
        out_hdlr.setFormatter(logging.Formatter(' %(asctime)s | %(levelname)s | %(module)s | %(message)s'))
        out_hdlr.setLevel(logging.INFO)
        LOGGER.addHandler(out_hdlr)
        LOGGER.setLevel(logging.INFO)

    return LOGGER
