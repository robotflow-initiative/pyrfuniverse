# Version of the library that will be used to upload to pypi
__version__ = "0.12.3"

import os.path
import json
import threading
import requests
from pyrfuniverse.utils.locker import Locker
from pyrfuniverse.utils.version import Version


def check_for_updates():
    try:
        response = requests.get(f'https://pypi.org/pypi/pyrfuniverse/json')
        response.raise_for_status()
        data = response.json()
        current_version = Version(__version__)

        versions = Version.sorted([Version(i) for i in data['releases']], reverse=True)
        for i in versions:
            if i[0] == current_version[0] and i[1] == current_version[1] and i[2] == current_version[2]:
                if i[3] > current_version[3]:
                    print(f'\033[33mThere is a new patch version available: {i}, please consider upgrading!\033[0m')
                    break
    except Exception as e:
        print(e)


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


update_thread = threading.Thread(target=check_for_updates)
update_thread.daemon = True
update_thread.start()
