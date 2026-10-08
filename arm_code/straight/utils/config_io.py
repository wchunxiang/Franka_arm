"""Load config.json and resolve file paths relative to it."""

import json
import os


def load_config(path):
    path = os.path.abspath(os.path.expanduser(path))
    with open(path, 'r') as f:
        cfg = json.load(f)
    cfg["_config_dir"] = os.path.dirname(path)
    print(f"Loaded config: {path}")
    return cfg


def resolve_path(cfg, path):
    """Absolute paths are kept; relative paths are relative to the config file."""
    if not path:
        return path
    path = os.path.expanduser(path)
    if os.path.isabs(path):
        return path
    return os.path.normpath(os.path.join(cfg.get("_config_dir", os.getcwd()), path))


def load_trajectory(path):
    """Load a 5xN (x,y,z,alpha,phi) or 6xN (+yaw) trajectory -> array of shape (6, N)."""
    import numpy as np

    raw = np.loadtxt(path)
    if raw.ndim == 1:
        raw = raw.reshape(-1, 1)
    if raw.shape[0] == 5:
        raw = np.vstack([raw, np.zeros((1, raw.shape[1]))])
    elif raw.shape[0] != 6:
        raise ValueError(f"Trajectory {path}: shape {raw.shape} not recognised (expected 5xN or 6xN)")
    return raw
