import os
from pathlib import Path

from scripts.validation.launcher_recovery import validation_pythonpath


def test_validation_subprocess_path_includes_runtime_packages_and_inherits_existing():
    repo = Path("/workspace/VIRA")
    value = validation_pythonpath(repo, "/opt/ros/site-packages:/custom")
    paths = value.split(os.pathsep)

    assert paths[:4] == [
        str(repo), str(repo / "adapters" / "sdk"), str(repo / "vira_interaction"),
        str(repo / "ros2_ws" / "src" / "gracemo_bridge"),
    ]
    assert paths[-2:] == ["/opt/ros/site-packages", "/custom"]
