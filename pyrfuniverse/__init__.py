# Version of the library that will be used to upload to pypi
__version__ = "0.10.1"

import os.path
import json
from pyrfuniverse.utils.locker import Locker


def read_config():
    if not os.path.exists(config_path):
        print("creating config")
        config = {}
        config["assets_path"] = ""
        config["executable_file"] = ""
        with open(config_path, "w", encoding="utf-8") as file:
            json.dump(config, file, indent=True)
    with open(config_path, "r", encoding="utf-8") as file:
        return json.load(file)


user_path = os.path.expanduser("~/.rfuniverse")
if not os.path.exists(user_path):
    os.makedirs(user_path)
config_path = os.path.join(user_path, "config.json")
with Locker("config"):
    config = read_config()
assets_path = config["assets_path"]
executable_file = config["executable_file"]
