from pathlib import Path
try:
    from ament_index_python.packages import get_package_share_directory
    ROOT = Path(get_package_share_directory('occupancy_joining'))
except (ImportError, LookupError):
    ROOT = Path(__file__).resolve().parent.parent
