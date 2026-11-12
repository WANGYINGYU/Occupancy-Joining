"""Portable output locations shared by the launcher, Docker and display cache."""
import os
from pathlib import Path


def workspace_path():
    value = os.environ.get('OCCUPANCY_JOINING_WORKSPACE')
    return (Path(value).expanduser() if value else Path.home()/'occupancy-joining-workspace').resolve()


def require_workspace():
    root = workspace_path()
    if root == Path(root.anchor):
        raise ValueError('Choose a dedicated OCCUPANCY_JOINING_WORKSPACE directory, not the filesystem root')
    root.mkdir(parents=True, exist_ok=True)
    if os.statvfs(root).f_flag & os.ST_RDONLY or not os.access(root, os.W_OK | os.X_OK):
        raise RuntimeError(f'Workspace is not writable: {root}')
    return root


def current_run():
    value = os.environ.get('OCCUPANCY_JOINING_RUN_DIR')
    if not value:
        raise RuntimeError('Start this command through tools/run.py')
    run = Path(value).expanduser().resolve()
    if not run.is_relative_to(workspace_path()):
        raise ValueError('OCCUPANCY_JOINING_RUN_DIR must be inside OCCUPANCY_JOINING_WORKSPACE')
    return run
