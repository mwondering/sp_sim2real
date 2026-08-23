from pathlib import Path

SIM2REAL_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = SIM2REAL_ROOT / "config"
SUPPORTED_ROBOTS = ("g1",)
RETARGET_IK_FILENAMES = {"g1": "xrobot_to_g1.json"}
ROBOT_XML_FILENAMES = {"g1": "g1.xml"}


def robot_config_dir(robot: str | Path) -> Path:
    robot_key = str(robot).strip().lower()
    return CONFIG_DIR / robot_key


def robot_config_path(robot: str | Path, name: str | Path) -> Path:
    name_path = Path(name)
    return name_path if name_path.is_absolute() else (robot_config_dir(robot) / name_path)


def robot_assets_dir(robot: str | Path) -> Path:
    return robot_config_path(robot, "assets")


def robot_xml_path(robot: str | Path) -> Path:
    robot_key = str(robot).strip().lower()
    return robot_assets_dir(robot_key) / ROBOT_XML_FILENAMES[robot_key]


def retarget_config_dir(robot: str | Path) -> Path:
    return robot_config_path(robot, "retarget")


def retarget_teleop_config_path(robot: str | Path) -> Path:
    return retarget_config_dir(robot) / "teleop.yaml"


def retarget_ik_config_path(robot: str | Path) -> Path:
    robot_key = str(robot).strip().lower()
    return retarget_config_dir(robot_key) / RETARGET_IK_FILENAMES[robot_key]
