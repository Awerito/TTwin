"""
OAK-D Configuration

Reads from .env file in project root.
"""

import os

from dotenv import load_dotenv

load_dotenv()

CAMERA_IP = os.environ.get("CAMERA_IP", "CHANGE_ME")
