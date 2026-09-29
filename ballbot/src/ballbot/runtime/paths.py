from pathlib import Path
import os

def repo_root() -> Path:
    # this file lives in src/ballbot/runtime/
    return Path(__file__).resolve().parents[3]

def cfg_path(env_key: str, rel: str) -> Path:
    # Allow override from env, otherwise use repo_root()/rel
    return Path(os.getenv(env_key, str(repo_root() / rel)))

def config_dir() -> Path:
    return repo_root() / "config"

def logs_dir() -> Path:
    d = repo_root() / "logs"
    d.mkdir(exist_ok=True)
    return d
