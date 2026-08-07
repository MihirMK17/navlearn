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

"""Tests for nav2_stack composition, parity guards and run provenance.

Two kinds of test live here and both are necessary.

The gate
    :func:`test_real_stack_passes_validator` runs the parity validator against the
    configuration the campaign will actually launch. It fails the build if the shipped
    stack ever develops the drift the 2026-07-25 audit found.

The proofs
    A guard that has never been observed to fail is not known to work. Every check the
    validator makes is paired here with a temporary stack that deliberately violates it,
    asserting the specific failure is raised. Without these, a validator broken into
    always-passing would look identical to a validator that is working.
"""

import importlib.util
import json
import os
import shutil
import subprocess
import sys

import pytest

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(TEST_DIR)
WORKSPACE = os.path.abspath(os.path.join(PACKAGE_DIR, "..", ".."))
REAL_STACK = os.path.join(
    WORKSPACE, "src", "bumperbot_navigation", "config", "nav2_stack"
)
VALIDATOR = os.path.join(PACKAGE_DIR, "scripts", "validate_nav2_stack.py")
HARNESS = os.path.join(PACKAGE_DIR, "scripts", "multi_run_harness.py")


def _load(name, path):
    """Import a module from an explicit path, as the launch files themselves do."""
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def stack_spec():
    """The shared discovery/merge module owned by bumperbot_navigation."""
    return _load(
        "stack_spec",
        os.path.join(WORKSPACE, "src", "bumperbot_navigation", "launch", "stack_spec.py"),
    )


@pytest.fixture(scope="module")
def harness():
    """The benchmark harness, imported for its spec-reading helpers."""
    return _load("multi_run_harness", HARNESS)


@pytest.fixture
def temp_stack(tmp_path):
    """A writable copy of the real stack, for tests that must break something."""
    destination = tmp_path / "nav2_stack"
    shutil.copytree(REAL_STACK, destination)
    return str(destination)


def run_validator(stack_dir):
    """Run the validator against a stack directory and return (exit_code, stdout)."""
    result = subprocess.run(
        [sys.executable, VALIDATOR, "--stack-dir", stack_dir],
        capture_output=True,
        text=True,
        timeout=120,
    )
    return result.returncode, result.stdout


# --------------------------------------------------------------------------------------
# The gate
# --------------------------------------------------------------------------------------


def test_real_stack_passes_validator():
    """The configuration the campaign will launch must satisfy every parity check."""
    code, output = run_validator(REAL_STACK)
    assert code == 0, f"shipped nav2_stack fails validation:\n{output}"


# --------------------------------------------------------------------------------------
# Discovery: the filesystem is the only source of legal names
# --------------------------------------------------------------------------------------


def test_discovery_reads_the_filesystem(stack_spec):
    """Legal names come from files on disk, not from a list in code."""
    assert stack_spec.available(REAL_STACK, "controller") == ["dwb", "mppi", "rpp"]
    assert "smac2d" in stack_spec.available(REAL_STACK, "planner")
    assert stack_spec.available(REAL_STACK, "ablation")[0] == stack_spec.NO_ABLATION


def test_new_fragment_is_discovered_without_code_change(stack_spec, temp_stack):
    """Adding a fragment file makes it selectable, with nothing else edited."""
    before = stack_spec.available(temp_stack, "controller")
    with open(os.path.join(temp_stack, "controllers", "zzz_new.yaml"), "w") as handle:
        handle.write("controller_server:\n  ros__parameters:\n    FollowPath:\n"
                     "      plugin: \"test::Plugin\"\n")

    after = stack_spec.available(temp_stack, "controller")
    assert set(after) - set(before) == {"zzz_new"}
    assert stack_spec.validate_selection(temp_stack, "controller", "zzz_new") is None


def test_unknown_selection_is_rejected(stack_spec):
    """An unknown name produces an error naming the legal alternatives."""
    problem = stack_spec.validate_selection(REAL_STACK, "controller", "nope")
    assert problem is not None
    assert "nope" in problem and "mppi" in problem


# --------------------------------------------------------------------------------------
# Composition: load order and merge semantics
# --------------------------------------------------------------------------------------


def test_ablation_overrides_common(stack_spec):
    """An ablation loaded last wins over the parity file it overrides."""
    common = os.path.join(REAL_STACK, "common", "controller_common.yaml")
    baseline = stack_spec.compose([common])
    with_ablation = stack_spec.compose(
        [common, os.path.join(REAL_STACK, "ablations", "high_tolerance.yaml")]
    )

    def tolerance(tree):
        return tree["controller_server"]["ros__parameters"]["general_goal_checker"][
            "xy_goal_tolerance"
        ]

    assert tolerance(baseline) == 0.05
    assert tolerance(with_ablation) == 0.10


def test_merge_preserves_untouched_siblings(stack_spec):
    """Overriding one nested key must not drop its siblings.

    A shallow merge would replace the whole general_goal_checker mapping and silently
    delete yaw_goal_tolerance, changing the success criterion the ablation was not
    supposed to touch.
    """
    composed = stack_spec.compose([
        os.path.join(REAL_STACK, "common", "controller_common.yaml"),
        os.path.join(REAL_STACK, "ablations", "high_tolerance.yaml"),
    ])
    checker = composed["controller_server"]["ros__parameters"]["general_goal_checker"]
    assert checker["xy_goal_tolerance"] == 0.10
    assert checker["yaw_goal_tolerance"] == 0.05
    assert checker["stateful"] is True


def test_controllers_share_one_success_criterion(stack_spec):
    """Composed stacks agree on the goal checker regardless of controller."""
    common = os.path.join(REAL_STACK, "common", "controller_common.yaml")
    tolerances = set()
    for controller in stack_spec.available(REAL_STACK, "controller"):
        composed = stack_spec.compose(
            [common, stack_spec.fragment_path(REAL_STACK, "controller", controller)]
        )
        params = composed["controller_server"]["ros__parameters"]
        tolerances.add(
            (
                params["general_goal_checker"]["xy_goal_tolerance"],
                params["progress_checker"]["movement_time_allowance"],
            )
        )
    assert len(tolerances) == 1, f"success criterion differs across controllers: {tolerances}"


# --------------------------------------------------------------------------------------
# Guard proofs: each validator check must actually fire
# --------------------------------------------------------------------------------------


def test_catches_success_criterion_leak(temp_stack):
    """A goal checker copied into a controller fragment fails validation."""
    path = os.path.join(temp_stack, "controllers", "rpp.yaml")
    with open(path) as handle:
        text = handle.read()
    with open(path, "w") as handle:
        handle.write(text.replace(
            "    FollowPath:",
            "    general_goal_checker:\n      xy_goal_tolerance: 0.08\n    FollowPath:",
            1,
        ))

    code, output = run_validator(temp_stack)
    assert code == 1
    assert "success-criteria-parity" in output


def test_catches_mppi_inflation_mismatch(temp_stack):
    """Restoring the retired critic/costmap mismatch fails validation."""
    path = os.path.join(temp_stack, "controllers", "mppi.yaml")
    with open(path) as handle:
        text = handle.read()
    with open(path, "w") as handle:
        handle.write(text.replace("cost_scaling_factor: 3.0", "cost_scaling_factor: 10.0", 1))

    code, output = run_validator(temp_stack)
    assert code == 1
    assert "mppi-inflation-parity" in output


def test_catches_velocity_desync(temp_stack):
    """Letting one controller drive faster than the others fails validation."""
    path = os.path.join(temp_stack, "controllers", "rpp.yaml")
    with open(path) as handle:
        text = handle.read()
    with open(path, "w") as handle:
        handle.write(text.replace("desired_linear_vel: 0.30", "desired_linear_vel: 0.26", 1))

    code, output = run_validator(temp_stack)
    assert code == 1
    assert "velocity-parity" in output


def test_catches_costmap_in_controller_fragment(temp_stack):
    """A per-controller costmap fails validation."""
    path = os.path.join(temp_stack, "controllers", "dwb.yaml")
    with open(path, "a") as handle:
        handle.write(
            "\nlocal_costmap:\n  local_costmap:\n    ros__parameters:\n      width: 4\n"
        )

    code, output = run_validator(temp_stack)
    assert code == 1
    assert "costmap-parity" in output


def test_catches_multi_variable_ablation(temp_stack):
    """An ablation that moves two knobs at once fails validation."""
    path = os.path.join(temp_stack, "ablations", "high_tolerance.yaml")
    with open(path, "a") as handle:
        handle.write("    progress_checker:\n      movement_time_allowance: 15.0\n")

    code, output = run_validator(temp_stack)
    assert code == 1
    assert "ablation-single-variable" in output


def test_catches_unaudited_new_controller(temp_stack):
    """A controller with no known speed parameter fails rather than being skipped."""
    with open(os.path.join(temp_stack, "controllers", "graceful.yaml"), "w") as handle:
        handle.write("controller_server:\n  ros__parameters:\n    FollowPath:\n"
                     "      plugin: \"nav2_graceful_controller::GracefulController\"\n")

    code, output = run_validator(temp_stack)
    assert code == 1
    assert "MAX_SPEED_KEYS" in output


# --------------------------------------------------------------------------------------
# Provenance
# --------------------------------------------------------------------------------------


def test_stack_spec_records_fragments_and_identity(stack_spec):
    """The spec captures every contributing file and the resolved cell identity."""
    selection = {
        "controller": "mppi",
        "planner": "smac2d",
        "localizer": "amcl_tuned",
        "ablation": "none",
    }
    spec = stack_spec.build_stack_spec(REAL_STACK, selection)

    assert spec["selection"] == selection
    assert spec["identity"]["controller_plugin"] == "nav2_mppi_controller::MPPIController"
    assert spec["identity"]["xy_goal_tolerance"] == 0.05
    assert all(len(f["sha256"]) == 64 for f in spec["fragments"])
    assert {f["role"] for f in spec["fragments"]} >= {
        "common/controller_common", "controllers/mppi", "localizers/amcl_tuned"
    }


def test_stack_spec_changes_when_configuration_changes(stack_spec, temp_stack):
    """Editing any fragment changes the composed hash, so drift cannot go unnoticed."""
    selection = {
        "controller": "mppi", "planner": "smac2d",
        "localizer": "amcl_tuned", "ablation": "none",
    }
    before = stack_spec.build_stack_spec(temp_stack, selection)["composed_sha256"]

    path = os.path.join(temp_stack, "common", "controller_common.yaml")
    with open(path) as handle:
        text = handle.read()
    with open(path, "w") as handle:
        handle.write(text.replace("xy_goal_tolerance: 0.05", "xy_goal_tolerance: 0.07", 1))

    after = stack_spec.build_stack_spec(temp_stack, selection)["composed_sha256"]
    assert before != after


def test_different_ablations_produce_different_specs(stack_spec):
    """Two cells that differ only by ablation are distinguishable from their specs."""
    base = {"controller": "mppi", "planner": "smac2d", "localizer": "amcl_tuned"}
    plain = stack_spec.build_stack_spec(REAL_STACK, {**base, "ablation": "none"})
    fast = stack_spec.build_stack_spec(REAL_STACK, {**base, "ablation": "high_vx"})
    assert plain["composed_sha256"] != fast["composed_sha256"]


# --------------------------------------------------------------------------------------
# Harness refuses to run without trustworthy provenance
# --------------------------------------------------------------------------------------


def test_harness_rejects_missing_spec(harness, tmp_path):
    """A missing spec aborts rather than producing unattributable results."""
    with pytest.raises(SystemExit) as excinfo:
        harness.read_stack_spec(str(tmp_path / "absent.json"))
    assert "no stack spec" in str(excinfo.value)


def test_harness_rejects_stale_spec(harness, stack_spec, tmp_path):
    """A spec whose launch process has exited is treated as stale, not as truth."""
    spec = stack_spec.build_stack_spec(
        REAL_STACK,
        {"controller": "rpp", "planner": "smac2d", "localizer": "amcl_tuned",
         "ablation": "none"},
        extra={"launch_pid": 2 ** 22},  # above PID_MAX on Linux, so never live
    )
    path = tmp_path / "stale.json"
    stack_spec.write_stack_spec(spec, str(path))

    with pytest.raises(SystemExit) as excinfo:
        harness.read_stack_spec(str(path))
    assert "no longer running" in str(excinfo.value)


def test_harness_rejects_malformed_spec(harness, tmp_path):
    """Corrupt JSON aborts with a specific message rather than a traceback."""
    path = tmp_path / "bad.json"
    path.write_text("{not json")
    with pytest.raises(SystemExit) as excinfo:
        harness.read_stack_spec(str(path))
    assert "not valid JSON" in str(excinfo.value)


def test_harness_rejects_incomplete_spec(harness, tmp_path):
    """A spec missing a required field aborts."""
    path = tmp_path / "partial.json"
    path.write_text(json.dumps({"schema": "navlearn.stack_spec/1"}))
    with pytest.raises(SystemExit) as excinfo:
        harness.read_stack_spec(str(path))
    assert "selection" in str(excinfo.value)


# --------------------------------------------------------------------------------------
# Compute sampler classification
# --------------------------------------------------------------------------------------


@pytest.fixture(scope="module")
def sampler():
    """The compute sampler, imported for its classifier."""
    return _load(
        "compute_sampler", os.path.join(PACKAGE_DIR, "scripts", "compute_sampler.py")
    )


@pytest.mark.parametrize(
    "name,expected",
    [
        ("controller_server", "ROBOT"),
        ("planner_server", "ROBOT"),
        ("amcl", "ROBOT"),
        ("bt_navigator", "ROBOT"),
        ("gzserver", "SIMULATION"),
        ("rviz2", "SIMULATION"),
        ("episode_manager", "MEASUREMENT"),
        ("robot_state_publisher", "INFRA"),
        ("sshd", None),
    ],
)
def test_classifies_by_process_name(sampler, name, expected):
    """Stack processes land in the right class; unrelated processes are ignored."""
    assert sampler.classify(name, f"/usr/bin/{name}") == expected


@pytest.mark.parametrize(
    "name,cmdline",
    [
        ("bash", "bash -c 'run controller_server and gz_sim'"),
        ("grep", "grep -r controller_server src/"),
        ("sh", "sh -c 'echo planner_server'"),
        ("ps", "ps -C controller_server -o %cpu"),
    ],
)
def test_shells_mentioning_nodes_are_not_classified(sampler, name, cmdline):
    """A process that merely names a node must not be counted as that node.

    Regression test. Classifying on the full command line swept in the harness itself,
    orchestrator shells and `ros2 launch` wrappers — all near-idle — which silently halved
    the reported mean CPU of whichever class they matched.
    """
    assert sampler.classify(name, cmdline) is None


def test_interpreter_hosted_nodes_are_resolved_by_cmdline(sampler):
    """A Python ROS node reports 'python3' as its name and must be resolved via cmdline."""
    assert sampler.classify("python3", "python3 /opt/ros/humble/lib/episode_manager") == (
        "MEASUREMENT"
    )
    assert sampler.classify("python3", "python3 -u /some/path/no_node_here.py") is None


def test_simulation_is_separable_from_robot(sampler):
    """Gazebo must never be counted as robot compute."""
    assert sampler.classify("gzserver", "gzserver world.sdf") == "SIMULATION"
    assert sampler.classify("controller_server", "") == "ROBOT"


def test_gpu_state_is_recorded_not_omitted(sampler):
    """GPU availability is always reported, including when absent."""
    state = sampler.gpu_state()
    assert "present_in_hardware" in state
    assert "driver_available" in state
    if not state["driver_available"]:
        assert state["note"], "absent GPU must carry an explanation, not an empty field"


def test_thermal_counters_are_readable(sampler):
    """Throttle counters must be readable, or a throttled campaign looks clean."""
    counters = sampler.throttle_counters()
    assert "package_throttle_count" in counters
    assert all(isinstance(v, int) for v in counters.values())


def test_harness_accepts_live_spec(harness, stack_spec, tmp_path):
    """A spec written by a running process is accepted."""
    spec = stack_spec.build_stack_spec(
        REAL_STACK,
        {"controller": "rpp", "planner": "smac2d", "localizer": "amcl_tuned",
         "ablation": "none"},
        extra={"launch_pid": os.getpid()},
    )
    path = tmp_path / "live.json"
    stack_spec.write_stack_spec(spec, str(path))

    loaded = harness.read_stack_spec(str(path))
    assert loaded["selection"]["controller"] == "rpp"
