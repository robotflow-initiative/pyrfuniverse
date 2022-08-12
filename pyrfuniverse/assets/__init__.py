import pyrfuniverse
import os


__path__ = os.path.join(pyrfuniverse.__path__[0], 'assets')


def join_path(path: str):
    return os.path.join(__path__, path)
