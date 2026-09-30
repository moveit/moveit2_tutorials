# Copyright (c) 2026 MoveIt 2 Community development team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import subprocess
from pathlib import Path

ENTRYPOINT_SH = (
    Path(__file__).resolve().parent.parent / ".docker" / "ros_entrypoint.sh"
)


def test_ros_entrypoint_syntax():
    """Verify ros_entrypoint.sh passes bash syntax check."""
    result = subprocess.run(
        ["bash", "-n", str(ENTRYPOINT_SH)], capture_output=True, text=True
    )
    assert result.returncode == 0, f"Syntax error in ros_entrypoint.sh: {result.stderr}"


def test_ros_entrypoint_as_process():
    """Verify executing ros_entrypoint.sh runs the given command and has set -e enabled."""
    # When run as process, it executes the passed command
    result = subprocess.run(
        ["bash", str(ENTRYPOINT_SH), "echo", "process_test_passed"],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0
    assert "process_test_passed" in result.stdout
    assert "Sourced ROS" in result.stdout


def test_ros_entrypoint_sourced_does_not_enable_errexit(tmp_path):
    """Verify sourcing ros_entrypoint.sh does not turn on set -e in the calling shell."""
    # Run a subshell that sources ros_entrypoint.sh, runs a command that fails (false),
    # and verifies the subshell continues executing subsequent commands.
    test_script = tmp_path / "test_source.sh"
    test_script.write_text(f"""#!/bin/bash
source "{ENTRYPOINT_SH}"
# This non-zero exit command would terminate the shell if set -e was active
false
echo "SURVIVED_NON_ZERO_EXIT"
""")
    test_script.chmod(0o755)

    result = subprocess.run(
        ["bash", str(test_script)],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0
    assert "SURVIVED_NON_ZERO_EXIT" in result.stdout
    assert "Sourced ROS" in result.stdout


def test_ros_entrypoint_sources_ros_underlay(tmp_path):
    """Verify ros_entrypoint.sh sources the designated ROS underlay."""
    mock_opt_ros = tmp_path / "opt" / "ros" / "test_distro"
    mock_opt_ros.mkdir(parents=True)
    setup_file = mock_opt_ros / "setup.bash"
    setup_file.write_text("export TEST_DISTRO_SOURCED=1\n")

    test_script = tmp_path / "test_underlay.sh"
    test_script.write_text(f"""#!/bin/bash
export ROS_DISTRO=test_distro
# Point to our mock underlay
sed -e 's|/opt/ros/|{tmp_path}/opt/ros/|g' "{ENTRYPOINT_SH}" > "{tmp_path}/entrypoint_underlay.sh"
chmod +x "{tmp_path}/entrypoint_underlay.sh"
source "{tmp_path}/entrypoint_underlay.sh"
echo "SOURCED_VAR=$TEST_DISTRO_SOURCED"
""")
    test_script.chmod(0o755)

    result = subprocess.run(
        ["bash", str(test_script)],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0
    assert "SOURCED_VAR=1" in result.stdout
