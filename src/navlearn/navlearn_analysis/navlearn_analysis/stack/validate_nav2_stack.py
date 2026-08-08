#!/usr/bin/env python3
# Copyright 2026 Mihir Kulkarni
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

"""Static parity checks for the composed nav2_stack configuration.

Purpose
    ``navigation.launch.py`` runs a preflight on the one stack it is about to launch.
    This script checks every stack the campaign will ever launch, without starting ROS,
    so a parity defect is caught at edit time rather than on the cell that happens to
    trip it. Intended for CI and as a precondition on the campaign orchestrator.

Checks
    1. Every fragment parses and carries the top-level key its role requires.
    2. No controller fragment defines a success criterion. The goal checker and progress
       checker belong to ``common/controller_common.yaml`` alone; a copy in a controller
       fragment is how ``xy_goal_tolerance`` came to be 0.05, 0.08 and 0.10 across arms
       in the retired campaign (audit F-5).
    3. No controller fragment defines a costmap. What the robot perceives is shared, so
       arms cannot differ in costmap size or inflation the way the old baseline (3x3 m,
       inflation 0.55) and aggressive (4x4 m, inflation 0.35) profiles did.
    4. MPPI's ObstaclesCritic inflation parameters equal the local costmap's. Nav2
       reconstructs obstacle distance by inverting the inflation curve, so a critic
       ``cost_scaling_factor`` above the costmap's makes obstacles read as nearer than
       they are. It was 10.0 against costmaps of 3.0 and 3.5 in the retired campaign,
       which handicapped both MPPI arms.
    5. All three controllers share one maximum linear velocity. Otherwise "controller X
       beat controller Y" is confounded with "X was allowed to drive faster".
    6. Every ablation changes exactly one parameter against the composed baseline, and
       names a fragment that exists.

Usage
    python3 validate_nav2_stack.py [--stack-dir PATH]

Exit status
    0 if every check passes, 1 otherwise. Failures print as
    ``FAIL <check>: <detail>`` on stdout.
"""

import argparse
import importlib.util
import itertools
import os
import sys

import yaml

COMMON_FRAGMENTS = {
    "controller_common": "controller_server",
    "planner_common": "planner_server",
    "bt_navigator": "bt_navigator",
    "behavior_server": "behavior_server",
    "smoother_server": "smoother_server",
}

# Keys that answer "has the robot arrived?". Exactly one file may define them.
SUCCESS_CRITERION_KEYS = (
    "general_goal_checker",
    "progress_checker",
    "goal_checker_plugins",
    "progress_checker_plugin",
)

# Each controller plugin names its speed limit differently.
MAX_SPEED_KEYS = {
    "rpp": "desired_linear_vel",
    "mppi": "vx_max",
    "dwb": "max_vel_x",
}


def _workspace_root():
    """Return the enclosing git workspace root, walking up from the CWD.

    Derived from the invocation directory rather than this file's location: as an
    installed module this file lives under install/, from which no fixed number of
    ".." reaches the source tree, and counting parents is exactly the pattern that
    silently broke when files moved.
    """
    root = os.path.abspath(os.getcwd())
    while root != os.path.dirname(root) and not os.path.isdir(
        os.path.join(root, ".git")
    ):
        root = os.path.dirname(root)
    return root


def _default_stack_dir():
    """Locate config/nav2_stack/, preferring the installed share directory."""
    try:
        from ament_index_python.packages import get_package_share_directory

        installed = os.path.join(
            get_package_share_directory("bumperbot_navigation"), "config", "nav2_stack"
        )
        if os.path.isdir(installed):
            return installed
    except Exception:  # noqa: BLE001 - ament absent in a bare checkout
        pass
    for rel in (
        ("src", "bumperbot", "bumperbot_navigation"),
        ("src", "bumperbot_navigation"),
    ):
        candidate = os.path.join(_workspace_root(), *rel, "config", "nav2_stack")
        if os.path.isdir(candidate):
            return candidate
    return os.path.join(
        _workspace_root(), "src", "bumperbot_navigation", "config", "nav2_stack"
    )


def _load_stack_spec():
    """Load bumperbot_navigation's stack_spec module, preferring the installed copy.

    The validator deliberately reuses the same discovery and merge code the launch files
    use. A second implementation here would be a second thing to drift, which is the defect
    class this whole restructure exists to remove.
    """
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(
            os.path.join(
                get_package_share_directory("bumperbot_navigation"),
                "launch",
                "stack_spec.py",
            )
        )
    except Exception:  # noqa: BLE001 - ament is absent in a bare CI checkout
        pass
    for rel in (
        ("src", "bumperbot", "bumperbot_navigation"),
        ("src", "bumperbot_navigation"),
    ):
        candidates.append(
            os.path.join(_workspace_root(), *rel, "launch", "stack_spec.py")
        )

    for path in candidates:
        if os.path.isfile(path):
            spec = importlib.util.spec_from_file_location("navlearn_stack_spec", path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module
    raise FileNotFoundError(f"stack_spec.py not found; looked in {candidates}")


STACK_SPEC = _load_stack_spec()


def _roles(stack, role):
    """Return the legal names for a role, excluding the 'none' ablation sentinel."""
    return [n for n in STACK_SPEC.available(stack, role) if n != STACK_SPEC.NO_ABLATION]


def _load(path):
    """Parse a YAML fragment, returning an empty dict for an empty file."""
    with open(path, "r") as handle:
        return yaml.safe_load(handle) or {}


def _controller_params(doc):
    """Extract the controller_server ros__parameters mapping from a parsed fragment."""
    return doc.get("controller_server", {}).get("ros__parameters", {})


def _flatten(mapping, prefix=""):
    """Flatten a nested mapping into dotted-path -> scalar pairs."""
    flat = {}
    for key, value in mapping.items():
        path = f"{prefix}.{key}" if prefix else str(key)
        if isinstance(value, dict):
            flat.update(_flatten(value, path))
        else:
            flat[path] = value
    return flat


def check_fragments_present(stack, failures):
    """Check 1: every expected fragment exists, parses, and has its role's top-level key."""
    expected = [
        (os.path.join(stack, "common", f"{name}.yaml"), key)
        for name, key in COMMON_FRAGMENTS.items()
    ]
    expected += [
        (os.path.join(stack, "controllers", f"{n}.yaml"), "controller_server")
        for n in _roles(stack, "controller")
    ]
    expected += [
        (os.path.join(stack, "planners", f"{n}.yaml"), "planner_server")
        for n in _roles(stack, "planner")
    ]
    expected += [
        (os.path.join(stack, "localizers", f"{n}.yaml"), "amcl")
        for n in _roles(stack, "localizer")
    ]
    expected += [
        (os.path.join(stack, "ablations", f"{n}.yaml"), "controller_server")
        for n in _roles(stack, "ablation")
    ]

    for path, required_key in expected:
        if not os.path.isfile(path):
            failures.append(("fragments-present", f"missing {path}"))
            continue
        try:
            doc = _load(path)
        except yaml.YAMLError as exc:
            failures.append(("fragments-present", f"{path} does not parse: {exc}"))
            continue
        if required_key not in doc:
            failures.append(
                (
                    "fragments-present",
                    f"{os.path.basename(path)} has no top-level '{required_key}' key "
                    f"(found {list(doc.keys())})",
                )
            )


def check_no_success_criteria_in_controllers(stack, failures):
    """Check 2: controller fragments must not define the goal or progress checker."""
    for name in _roles(stack, "controller"):
        path = os.path.join(stack, "controllers", f"{name}.yaml")
        if not os.path.isfile(path):
            continue
        params = _controller_params(_load(path))
        leaked = [key for key in SUCCESS_CRITERION_KEYS if key in params]
        if leaked:
            failures.append(
                (
                    "success-criteria-parity",
                    f"controllers/{name}.yaml defines {leaked}; these belong only to "
                    "common/controller_common.yaml",
                )
            )


def check_no_costmap_in_controllers(stack, failures):
    """Check 3: controller fragments must not define a costmap."""
    for name in _roles(stack, "controller"):
        path = os.path.join(stack, "controllers", f"{name}.yaml")
        if not os.path.isfile(path):
            continue
        doc = _load(path)
        for key in ("local_costmap", "global_costmap"):
            if key in doc:
                failures.append(
                    (
                        "costmap-parity",
                        f"controllers/{name}.yaml defines '{key}'; the costmap is shared "
                        "and belongs in common/",
                    )
                )


def check_mppi_inflation_parity(stack, failures):
    """Check 4: MPPI's ObstaclesCritic must mirror the local costmap's inflation."""
    common_path = os.path.join(stack, "common", "controller_common.yaml")
    mppi_path = os.path.join(stack, "controllers", "mppi.yaml")
    if not (os.path.isfile(common_path) and os.path.isfile(mppi_path)):
        return

    critic = (
        _controller_params(_load(mppi_path))
        .get("FollowPath", {})
        .get("ObstaclesCritic", {})
    )
    inflation = (
        _load(common_path)
        .get("local_costmap", {})
        .get("local_costmap", {})
        .get("ros__parameters", {})
        .get("inflation_layer", {})
    )

    for key in ("inflation_radius", "cost_scaling_factor"):
        if key not in critic:
            failures.append(
                ("mppi-inflation-parity", f"mppi.yaml ObstaclesCritic has no '{key}'")
            )
        elif key in inflation and critic[key] != inflation[key]:
            failures.append(
                (
                    "mppi-inflation-parity",
                    f"ObstaclesCritic.{key}={critic[key]} but local_costmap "
                    f"inflation_layer.{key}={inflation[key]}; Nav2 requires a match",
                )
            )


def check_velocity_parity(stack, failures):
    """Check 5: all controllers share one maximum linear velocity."""
    speeds = {}
    for name in _roles(stack, "controller"):
        path = os.path.join(stack, "controllers", f"{name}.yaml")
        if not os.path.isfile(path):
            continue

        # A controller whose speed key is unknown cannot be parity-checked, and silently
        # skipping it would let a new arm join the campaign with an unaudited speed
        # envelope. Fail instead, and require MAX_SPEED_KEYS to be extended deliberately.
        key = MAX_SPEED_KEYS.get(name)
        if key is None:
            failures.append(
                (
                    "velocity-parity",
                    f"controllers/{name}.yaml exists but MAX_SPEED_KEYS has no entry for "
                    f"'{name}', so its speed envelope cannot be checked; add the plugin's "
                    "maximum-linear-velocity parameter name to MAX_SPEED_KEYS",
                )
            )
            continue

        follow_path = _controller_params(_load(path)).get("FollowPath", {})
        if key not in follow_path:
            failures.append(
                ("velocity-parity", f"controllers/{name}.yaml has no '{key}'")
            )
            continue
        speeds[name] = follow_path[key]

    for (name_a, speed_a), (name_b, speed_b) in itertools.combinations(
        speeds.items(), 2
    ):
        if speed_a != speed_b:
            failures.append(
                (
                    "velocity-parity",
                    f"{name_a} max speed {speed_a} != {name_b} max speed {speed_b}; "
                    "differing speed envelopes confound the controller comparison",
                )
            )


def check_ablations_single_variable(stack, failures):
    """Check 6: each ablation changes exactly one parameter against the baseline."""
    common_path = os.path.join(stack, "common", "controller_common.yaml")
    if not os.path.isfile(common_path):
        return

    baseline = _flatten(_controller_params(_load(common_path)))
    for name in _roles(stack, "controller"):
        controller_path = os.path.join(stack, "controllers", f"{name}.yaml")
        if os.path.isfile(controller_path):
            baseline.update(_flatten(_controller_params(_load(controller_path))))

    for name in _roles(stack, "ablation"):
        path = os.path.join(stack, "ablations", f"{name}.yaml")
        if not os.path.isfile(path):
            continue
        overrides = _flatten(_controller_params(_load(path)))

        changed = [
            key
            for key, value in overrides.items()
            if key in baseline and baseline[key] != value
        ]
        unknown = [key for key in overrides if key not in baseline]

        # high_vx names the speed key of all three controllers; only the loaded plugin
        # consumes its own, so the sibling keys count as one variable. DWB's max_speed_xy
        # must track max_vel_x, so it rides along as part of the same knob.
        if name == "high_vx":
            speed_keys = tuple(MAX_SPEED_KEYS.values()) + ("max_speed_xy",)
            collapsed = [key for key in changed if key.endswith(speed_keys)]
            changed = [key for key in changed if key not in collapsed]
            if collapsed:
                changed.append("<max linear velocity>")
            unknown = [key for key in unknown if not key.endswith(speed_keys)]

        if unknown:
            failures.append(
                (
                    "ablation-single-variable",
                    f"ablations/{name}.yaml sets {unknown}, which no baseline fragment "
                    "declares — a typo here is silently ignored at runtime",
                )
            )
        if len(changed) != 1:
            failures.append(
                (
                    "ablation-single-variable",
                    f"ablations/{name}.yaml changes {len(changed)} parameters {changed}; "
                    "an ablation must isolate exactly one",
                )
            )


def main():
    """Run every check against the stack directory and report the outcome."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument(
        "--stack-dir",
        default=_default_stack_dir(),
        help="Path to config/nav2_stack/ (default: resolved from this script's location)",
    )
    args = parser.parse_args()

    if not os.path.isdir(args.stack_dir):
        print(f"FAIL stack-dir: not a directory: {args.stack_dir}")
        return 1

    print(f"Validating {args.stack_dir}")
    failures = []
    checks = (
        check_fragments_present,
        check_no_success_criteria_in_controllers,
        check_no_costmap_in_controllers,
        check_mppi_inflation_parity,
        check_velocity_parity,
        check_ablations_single_variable,
    )
    for check in checks:
        check(args.stack_dir, failures)

    if failures:
        for name, detail in failures:
            print(f"FAIL {name}: {detail}")
        print(f"\n{len(failures)} failure(s).")
        return 1

    print(f"All {len(checks)} checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
