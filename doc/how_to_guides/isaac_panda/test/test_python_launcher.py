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

import os
import stat
import subprocess
from pathlib import Path
import pytest

LAUNCH_DIR = Path(__file__).resolve().parent.parent / "launch"
PYTHON_SH = LAUNCH_DIR / "python.sh"


def test_python_sh_syntax():
    """Verify python.sh passes bash syntax check."""
    result = subprocess.run(
        ["bash", "-n", str(PYTHON_SH)], capture_output=True, text=True
    )
    assert result.returncode == 0, f"Syntax error in python.sh: {result.stderr}"


def test_python_sh_no_install_fails_with_help():
    """Verify python.sh exits with code 1 and prints troubleshooting message when no install is found."""
    # Clean environment without any candidate directories or running container
    clean_env = {
        "PATH": "/usr/bin:/bin",
        "HOME": "/tmp/nonexistent_home",
    }
    result = subprocess.run(
        ["bash", str(PYTHON_SH)], capture_output=True, text=True, env=clean_env
    )
    assert result.returncode == 1
    assert (
        "No valid Isaac Sim installation or running container found." in result.stdout
    )
    assert "Troubleshooting:" in result.stdout
    assert "export ISAAC_SIM_PATH=/path/to/isaacsim" in result.stdout


def test_isaac_path_override(tmp_path):
    """Verify --isaac-path points to custom Isaac Sim directory and executes its python.sh."""
    fake_isaac_dir = tmp_path / "mock_isaac"
    fake_isaac_dir.mkdir()
    fake_python = fake_isaac_dir / "python.sh"

    log_file = tmp_path / "call.log"
    fake_python.write_text(f"""#!/bin/bash
echo "CALLED with args: $@" > "{log_file}"
exit 0
""")
    fake_python.chmod(fake_python.stat().st_mode | stat.S_IEXEC)

    dummy_script = tmp_path / "test_script.py"
    dummy_script.write_text("print('hello')")

    result = subprocess.run(
        [
            "bash",
            str(PYTHON_SH),
            "--isaac-path",
            str(fake_isaac_dir),
            str(dummy_script),
            "--headless",
        ],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0
    assert log_file.exists()
    content = log_file.read_text()
    assert str(dummy_script.resolve()) in content
    assert "--headless" in content


def test_isaac_sim_path_env_var(tmp_path):
    """Verify ISAAC_SIM_PATH environment variable is honored."""
    fake_isaac_dir = tmp_path / "mock_isaac_env"
    fake_isaac_dir.mkdir()
    fake_python = fake_isaac_dir / "python.sh"

    log_file = tmp_path / "env_call.log"
    fake_python.write_text(f"""#!/bin/bash
echo "ENV_CALLED: $@" > "{log_file}"
exit 0
""")
    fake_python.chmod(fake_python.stat().st_mode | stat.S_IEXEC)

    env = os.environ.copy()
    env["ISAAC_SIM_PATH"] = str(fake_isaac_dir)

    result = subprocess.run(
        ["bash", str(PYTHON_SH), "--headless"],
        capture_output=True,
        text=True,
        env=env,
    )
    assert result.returncode == 0
    assert log_file.exists()
    assert "--headless" in log_file.read_text()


def test_relative_script_path_resolution(tmp_path):
    """Verify relative script names inside the launch dir are automatically resolved."""
    fake_isaac_dir = tmp_path / "mock_isaac"
    fake_isaac_dir.mkdir()
    fake_python = fake_isaac_dir / "python.sh"

    log_file = tmp_path / "rel_call.log"
    fake_python.write_text(f"""#!/bin/bash
echo "ARGS: $@" > "{log_file}"
exit 0
""")
    fake_python.chmod(fake_python.stat().st_mode | stat.S_IEXEC)

    # Pass isaac_moveit.py relative name
    result = subprocess.run(
        [
            "bash",
            str(PYTHON_SH),
            "--isaac-path",
            str(fake_isaac_dir),
            "isaac_moveit.py",
        ],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0
    assert log_file.exists()
    expected_path = str((LAUNCH_DIR / "isaac_moveit.py").resolve())
    assert expected_path in log_file.read_text()


def test_docker_container_forwarding(tmp_path):
    """Verify execution forwards into running Isaac Sim container when detected."""
    # Create a mock docker command in tmp_path/bin
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    mock_docker = bin_dir / "docker"
    docker_log = tmp_path / "docker.log"

    mock_docker.write_text(f"""#!/bin/bash
echo "docker called with: $@" >> "{docker_log}"
if [ "$1" = "ps" ]; then
    echo "isaac-sim"
    exit 0
elif [ "$1" = "cp" ]; then
    exit 0
elif [ "$1" = "exec" ]; then
    exit 0
fi
exit 0
""")
    mock_docker.chmod(mock_docker.stat().st_mode | stat.S_IEXEC)

    dummy_script = tmp_path / "my_sim.py"
    dummy_script.write_text("# dummy")

    env = {
        "PATH": f"{bin_dir}:/usr/bin:/bin",
        "HOME": str(tmp_path),
    }

    result = subprocess.run(
        ["bash", str(PYTHON_SH), str(dummy_script), "--headless"],
        capture_output=True,
        text=True,
        env=env,
    )
    assert result.returncode == 0
    assert docker_log.exists()
    log_content = docker_log.read_text()
    assert "ps" in log_content
    assert "cp" in log_content
    assert "exec" in log_content
    assert "/isaac-sim/python.sh" in log_content
