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

import contextlib
import py_compile
import runpy
import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

LAUNCH_DIR = Path(__file__).resolve().parent.parent / "launch"
ISAAC_MOVEIT_PY = LAUNCH_DIR / "isaac_moveit.py"


def test_isaac_moveit_syntax():
    """Verify isaac_moveit.py compiles cleanly under Python 3."""
    py_compile.compile(str(ISAAC_MOVEIT_PY), doraise=True)


@contextlib.contextmanager
def mock_isaac_sim_environment(version_tuple, is_ge_4_5=True, camera_exists=False):
    """Context manager to mock Isaac Sim modules for testing script logic without GPU/binaries."""
    # Track mock objects to assert behavior after script execution
    mocks = {
        "enabled_extensions": [],
        "created_prims": [],
        "defined_cameras": [],
        "simulation_app_config": None,
        "set_targets_calls": [],
    }

    mock_carb = MagicMock()
    mock_carb.log_warn = MagicMock()
    mock_carb.log_error = MagicMock()

    mock_simulation_app_instance = MagicMock()
    # is_running returns False immediately so while loop terminates
    mock_simulation_app_instance.is_running.return_value = False

    def mock_simulation_app_init(config):
        mocks["simulation_app_config"] = config
        return mock_simulation_app_instance

    mock_simulation_app_cls = MagicMock(side_effect=mock_simulation_app_init)

    # Core context and utils
    mock_context_instance = MagicMock()
    mock_context_cls = MagicMock(return_value=mock_context_instance)

    mock_extensions = MagicMock()
    mock_extensions.enable_extension.side_effect = lambda ext: mocks[
        "enabled_extensions"
    ].append(ext)

    mock_prims = MagicMock()
    mock_prims.create_prim.side_effect = lambda *args, **kwargs: mocks[
        "created_prims"
    ].append((args, kwargs))

    mock_rotations = MagicMock()
    mock_rotations.gf_rotation_to_np_array.return_value = [0, 0, 0, 1]

    mock_stage = MagicMock()
    mock_usd_stage = MagicMock()
    mock_camera_prim = MagicMock()
    mock_camera_prim.IsValid.return_value = camera_exists
    mock_usd_stage.GetPrimAtPath.return_value = mock_camera_prim
    mock_stage.get_current_stage.return_value = mock_usd_stage

    mock_viewports = MagicMock()

    mock_nucleus = MagicMock()
    mock_nucleus.get_assets_root_path.return_value = "/mock/assets"

    def mock_set_targets(*args, **kwargs):
        mocks["set_targets_calls"].append((args, kwargs))

    # PXR USD mocks
    mock_gf = MagicMock()
    mock_usd_geom = MagicMock()

    mock_defined_camera = MagicMock()
    mock_defined_camera.IsValid.return_value = True

    def mock_camera_define(stage_obj, path):
        mocks["defined_cameras"].append(path)
        return mock_defined_camera

    mock_usd_geom.Camera.Define.side_effect = mock_camera_define

    mock_og = MagicMock()

    # Dictionary of modules to inject
    injected_modules = {
        "carb": mock_carb,
        "pxr": MagicMock(Gf=mock_gf, UsdGeom=mock_usd_geom),
        "omni": MagicMock(),
        "omni.graph.core": mock_og,
        "omni.ui": MagicMock(),
    }

    if is_ge_4_5:
        # Isaac Sim 4.5+ uses isaacsim.* namespace
        mock_isaacsim = MagicMock(SimulationApp=mock_simulation_app_cls)
        mock_core_version = MagicMock(get_version=MagicMock(return_value=version_tuple))
        mock_core_api = MagicMock(SimulationContext=mock_context_cls)
        mock_core_utils = MagicMock(
            extensions=mock_extensions,
            prims=mock_prims,
            rotations=mock_rotations,
            stage=mock_stage,
            viewports=mock_viewports,
        )
        mock_core_utils_prims = MagicMock(set_targets=mock_set_targets)
        mock_storage_native = MagicMock(nucleus=mock_nucleus)

        injected_modules.update(
            {
                "isaacsim": mock_isaacsim,
                "isaacsim.core.version": mock_core_version,
                "isaacsim.core.api": mock_core_api,
                "isaacsim.core.utils": mock_core_utils,
                "isaacsim.core.utils.prims": mock_core_utils_prims,
                "isaacsim.storage.native": mock_storage_native,
            }
        )
    else:
        # Pre-4.5 uses omni.isaac.* namespace
        mock_omni_kit = MagicMock(SimulationApp=mock_simulation_app_cls)
        mock_omni_version = MagicMock(get_version=MagicMock(return_value=version_tuple))
        mock_omni_core = MagicMock(SimulationContext=mock_context_cls)
        mock_omni_core_utils = MagicMock(
            extensions=mock_extensions,
            prims=mock_prims,
            rotations=mock_rotations,
            stage=mock_stage,
            viewports=mock_viewports,
            nucleus=mock_nucleus,
        )
        mock_omni_core_utils_prims = MagicMock(set_targets=mock_set_targets)
        mock_omni_core_nodes_scripts_utils = MagicMock(
            set_target_prims=mock_set_targets
        )

        injected_modules.update(
            {
                "omni.isaac.kit": mock_omni_kit,
                "omni.isaac.version": mock_omni_version,
                "omni.isaac.core": mock_omni_core,
                "omni.isaac.core.utils": mock_omni_core_utils,
                "omni.isaac.core.utils.prims": mock_omni_core_utils_prims,
                "omni.isaac.core_nodes.scripts.utils": mock_omni_core_nodes_scripts_utils,
            }
        )

    # Save original sys.modules state
    saved_modules = {k: sys.modules.get(k) for k in injected_modules}

    try:
        # Ensure any isaacsim / omni modules not in injected_modules are removed from sys.modules
        for k in list(sys.modules.keys()):
            if k.startswith("isaacsim") or k.startswith("omni.isaac"):
                if k not in injected_modules:
                    sys.modules.pop(k, None)

        sys.modules.update(injected_modules)
        yield mocks
    finally:
        # Restore sys.modules
        for k, v in saved_modules.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v


def test_isaac_sim_6_1_execution():
    """Verify script behavior and asset path selection under Isaac Sim 6.1."""
    # Isaac Sim 6.1 returns version with major '6'
    v6_version = ["6.1.0", "release", "6", "1", "0", "0"]
    test_argv = ["isaac_moveit.py", "--headless"]

    with patch.object(sys, "argv", test_argv):
        with mock_isaac_sim_environment(
            v6_version, is_ge_4_5=True, camera_exists=False
        ) as mocks:
            env = runpy.run_path(str(ISAAC_MOVEIT_PY), run_name="__main__")

            # Check version detection
            assert env["isaac_sim_ge_4_5_version"] is True
            assert env["is_legacy_isaacsim"] is False

            # Check Franka USD path uses multiphysics path
            assert (
                env["FRANKA_USD_PATH"]
                == "/Isaac/Robots_Multiphysics/FrankaRobotics/FrankaPanda/franka/franka.usda"
            )

            # Check bridge extension
            assert "isaacsim.ros2.bridge" in mocks["enabled_extensions"]

            # Check headless configuration
            assert mocks["simulation_app_config"]["headless"] is True

            # Camera was missing, should have been defined
            assert len(mocks["defined_cameras"]) == 1
            assert mocks["defined_cameras"][0] == env["CAMERA_PRIM_PATH"]


def test_isaac_sim_4_5_execution():
    """Verify script behavior and asset path selection under Isaac Sim 4.5."""
    # Isaac Sim 4.5 returns version with major '4'
    v45_version = ["4.5.0", "release", "4", "5", "0", "0"]
    test_argv = ["isaac_moveit.py", "--headless", "--livestream"]

    with patch.object(sys, "argv", test_argv):
        with mock_isaac_sim_environment(
            v45_version, is_ge_4_5=True, camera_exists=False
        ) as mocks:
            env = runpy.run_path(str(ISAAC_MOVEIT_PY), run_name="__main__")

            # Check version detection
            assert env["isaac_sim_ge_4_5_version"] is True
            assert env["is_legacy_isaacsim"] is False

            # Check Franka USD path uses multiphysics path
            assert (
                env["FRANKA_USD_PATH"]
                == "/Isaac/Robots_Multiphysics/FrankaRobotics/FrankaPanda/franka/franka.usda"
            )

            # Check bridge extension
            assert "isaacsim.ros2.bridge" in mocks["enabled_extensions"]

            # Check livestream configuration
            assert mocks["simulation_app_config"]["headless"] is True
            assert mocks["simulation_app_config"]["livestream"] == 2


def test_legacy_isaac_sim_execution():
    """Verify script behavior under legacy Isaac Sim (e.g. 2023.1.1)."""
    legacy_version = ["2023.1.1", "release", "2023", "1", "1", "0"]
    test_argv = ["isaac_moveit.py"]

    with patch.object(sys, "argv", test_argv):
        with mock_isaac_sim_environment(
            legacy_version, is_ge_4_5=False, camera_exists=True
        ) as mocks:
            env = runpy.run_path(str(ISAAC_MOVEIT_PY), run_name="__main__")

            # Legacy version detected
            assert env["isaac_sim_ge_4_5_version"] is False
            assert env["is_legacy_isaacsim"] is True

            # Uses legacy Franka USD path
            assert (
                env["FRANKA_USD_PATH"] == "/Isaac/Robots/Franka/franka_alt_fingers.usd"
            )

            # Uses omni.isaac.ros2_bridge
            assert "omni.isaac.ros2_bridge" in mocks["enabled_extensions"]

            # Camera was already present, should not define new camera
            assert len(mocks["defined_cameras"]) == 0
