"""
Session configuration for the UAV-Triage pilot script.

Reads ~/UAV-Triage/session.json (or $UAV_SESSION) and returns the settings
for the current block. The pilot script refuses to start if the file is
missing or invalid, so a study block can never run with an unintended tau.

    tau_s = number  -> agent takes over after tau_s seconds of operator inaction
    tau_s = null    -> control condition: agent never takes over, vehicle holds
"""
import hashlib
import json
import os

DEFAULT_PATH = os.path.expanduser("~/UAV-Triage/session.json")


def load_session(path=None):
    path = path or os.environ.get("UAV_SESSION", DEFAULT_PATH)
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"Session file not found: {path}. Create it (see session.json in the repo) "
            f"or set UAV_SESSION.")
    with open(path, "rb") as f:
        raw = f.read()
    cfg = json.loads(raw.decode("utf-8"))

    levels = [float(v) for v in cfg.get("tau_levels_s", [])]
    episode_min = float(cfg.get("episode_min_s", 1.0))
    block = dict(cfg.get("block", {}))
    if "tau_s" not in block:
        raise ValueError(f"{path}: block.tau_s is missing (use null for the control condition).")
    tau = block["tau_s"]
    if tau is not None:
        if isinstance(tau, bool) or not isinstance(tau, (int, float)) or tau <= 0:
            raise ValueError(f"{path}: block.tau_s must be a positive number or null, got {tau!r}.")
        tau = float(tau)
        if tau < episode_min:
            raise ValueError(f"{path}: block.tau_s ({tau}) is shorter than episode_min_s ({episode_min}).")
    if episode_min <= 0:
        raise ValueError(f"{path}: episode_min_s must be positive.")

    warnings = []
    if tau is not None and levels and tau not in levels:
        warnings.append(f"tau_s={tau} is not one of tau_levels_s {levels}")

    return {
        "path": path,
        "sha256": hashlib.sha256(raw).hexdigest(),
        "tau_s": tau,
        "is_control": tau is None,
        "tau_levels_s": levels,
        "episode_min_s": episode_min,
        "block": block,
        "warnings": warnings,
    }
