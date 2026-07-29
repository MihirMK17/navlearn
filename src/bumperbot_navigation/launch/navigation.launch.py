"""Nav2 bringup for the NavLearn benchmark stack.

Composes a navigation stack from role-scoped YAML fragments under
``config/nav2_stack/`` instead of loading one monolithic profile directory.

Purpose
    A profile directory held a full copy of every server's YAML, so seven profiles meant
    seven copies of the goal checker, the progress checker and the costmaps. They drifted:
    ``xy_goal_tolerance`` was 0.05, 0.08 and 0.10 across arms, ``movement_time_allowance``
    was 6.0 against 60.0, and ``bt_loop_duration`` was 20 on one profile and 10 on the rest.
    A controller comparison run across those arms measures the difference in the question
    being asked, not the difference between controllers.

    Composition removes the copies. Anything that must not vary lives in exactly one file
    under ``common/`` and is loaded first; the controller, planner and optional ablation
    fragments layer on top. ROS 2 merges a node's ``parameters=[...]`` list in order with
    later entries winning, so no new machinery is needed — only ordering.

Launch arguments
    use_sim_time     bool, default True
    controller       one of controllers/*.yaml, default rpp
    planner          one of planners/*.yaml, default smac2d
    ablation         ``none`` or one of ablations/*.yaml, loaded last
    stack_spec_out   where to write the provenance record; empty disables writing

    Legal values are discovered from the filesystem by ``stack_spec.available`` rather than
    listed here, so adding a controller means adding one file and nothing else.

    The localizer is selected at bringup, not here: AMCL is launched by
    ``bumperbot_localization/launch/global_localization.launch.py`` via its ``amcl_config``
    path argument, which ``simulated_robot.launch.py`` resolves from
    ``config/nav2_stack/localizers/``.

Nodes launched
    controller_server, planner_server, smoother_server, bt_navigator, behavior_server,
    and the navigation lifecycle manager that autostarts them.

Preflight
    ``_compose_stack`` refuses to launch on an unknown fragment name, a missing file, a
    goal-checker or progress-checker key that has leaked into a controller fragment, or an
    MPPI ObstaclesCritic whose inflation parameters disagree with the local costmap's.
    Each of those is a defect that produces plausible-looking but invalid results, so it is
    caught before the sim starts rather than found in the data afterwards.

Provenance
    On a successful preflight the composed stack is written to ``stack_spec_out`` as JSON:
    every contributing fragment with its SHA-256, plus the resolved parameter values that
    define the cell. The benchmark harness copies that record into each run directory, so a
    result's configuration is recoverable from the result itself rather than from a flag
    someone typed. This replaces the retired ``nav2_profile`` argument, which named a
    profile that nothing verified was the one actually loaded.
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Launch files are loaded by path rather than imported as a package, so a sibling module
# has to be loaded explicitly. stack_spec.py is installed alongside this file.
_spec = importlib.util.spec_from_file_location(
    "navlearn_stack_spec", os.path.join(os.path.dirname(__file__), "stack_spec.py")
)
stack_spec = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(stack_spec)

DEFAULT_STACK_SPEC_OUT = os.path.join(
    os.path.expanduser("~"), ".navlearn", "current_stack_spec.json"
)

LIFECYCLE_NODES = [
    "controller_server",
    "planner_server",
    "smoother_server",
    "bt_navigator",
    "behavior_server",
]

# Keys that define what counts as arriving at a goal. They belong to
# common/controller_common.yaml alone. A controller fragment that sets one of these has
# reintroduced audit finding F-5, so the preflight rejects it by name.
PARITY_GUARDED_KEYS = (
    "general_goal_checker",
    "progress_checker",
    "goal_checker_plugins",
    "progress_checker_plugin",
)


def _abort(message):
    """Return launch actions that log a preflight failure and shut the launch down."""
    return [
        LogInfo(msg=f"[nav2_stack] PREFLIGHT FAILED: {message}"),
        EmitEvent(event=Shutdown(reason=f"nav2_stack preflight: {message}")),
    ]


def _controller_params(doc):
    """Extract the controller_server ros__parameters mapping from a parsed fragment."""
    return doc.get("controller_server", {}).get("ros__parameters", {})


def _check_parity_leak(fragment_path):
    """Return an error string if a controller fragment redefines a success-criterion key."""
    params = _controller_params(stack_spec.compose([fragment_path]))
    leaked = [key for key in PARITY_GUARDED_KEYS if key in params]
    if leaked:
        return (
            f"{os.path.basename(fragment_path)} defines {leaked}, which must live only in "
            "common/controller_common.yaml. A controller fragment that sets its own goal "
            "or progress checker makes the controller comparison invalid."
        )
    return None


def _check_mppi_inflation_parity(common_path, controller_path):
    """Return an error string if MPPI's ObstaclesCritic disagrees with the local costmap.

    Nav2's ObstaclesCritic recovers obstacle distance by inverting the costmap's inflation
    curve, so its ``inflation_radius`` and ``cost_scaling_factor`` must equal the values the
    inflation layer actually used. They did not in the retired campaign: the critic ran at
    ``cost_scaling_factor`` 10.0 against costmaps of 3.0 and 3.5, making every obstacle read
    as nearer than it was and handicapping both MPPI arms.
    """
    critic = (
        _controller_params(stack_spec.compose([controller_path]))
        .get("FollowPath", {})
        .get("ObstaclesCritic", {})
    )
    if not critic:
        return None

    inflation = (
        stack_spec.compose([common_path])
        .get("local_costmap", {})
        .get("local_costmap", {})
        .get("ros__parameters", {})
        .get("inflation_layer", {})
    )

    mismatches = [
        f"{key}: critic {critic[key]} != costmap {inflation[key]}"
        for key in ("inflation_radius", "cost_scaling_factor")
        if key in critic and key in inflation and critic[key] != inflation[key]
    ]
    if mismatches:
        return (
            "MPPI ObstaclesCritic disagrees with local_costmap inflation_layer — "
            + "; ".join(mismatches)
            + ". Nav2 requires these to match; a mismatch silently distorts perceived "
            "obstacle distance."
        )
    return None


def _compose_stack(context, *args, **kwargs):
    """Validate the requested fragments, record provenance, and build the Nav2 node set."""
    selection = {
        role: LaunchConfiguration(role).perform(context)
        for role in ("controller", "planner", "ablation")
    }
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    spec_out = LaunchConfiguration("stack_spec_out").perform(context)

    pkg = get_package_share_directory("bumperbot_navigation")
    stack = stack_spec.stack_dir(pkg)

    for role, name in selection.items():
        problem = stack_spec.validate_selection(stack, role, name)
        if problem:
            return _abort(problem)

    common = {
        role: os.path.join(stack, "common", f"{role}.yaml")
        for role in (
            "controller_common",
            "planner_common",
            "bt_navigator",
            "behavior_server",
            "smoother_server",
        )
    }
    missing = [path for path in common.values() if not os.path.isfile(path)]
    if missing:
        return _abort(
            "common fragment(s) not found: "
            + ", ".join(missing)
            + ". If the file exists in src/, the package needs rebuilding — config/ is "
            "installed to share/ at build time."
        )

    controller_yaml = stack_spec.fragment_path(stack, "controller", selection["controller"])
    planner_yaml = stack_spec.fragment_path(stack, "planner", selection["planner"])
    ablation_yaml = stack_spec.fragment_path(stack, "ablation", selection["ablation"])

    for problem in (
        _check_parity_leak(controller_yaml),
        _check_mppi_inflation_parity(common["controller_common"], controller_yaml),
    ):
        if problem:
            return _abort(problem)

    # The behavior trees ship inside this package. They were previously referenced by an
    # absolute path into a different workspace (/home/mihirmk/bumperbot_ws), which resolved
    # only by coincidence of that directory existing on this machine.
    behavior_tree = os.path.join(pkg, "behavior_tree")
    bt_paths = {
        "default_nav_to_pose_bt_xml": os.path.join(
            behavior_tree, "simple_navigation_w_replanning_and_recoveries.xml"
        ),
        "default_nav_through_poses_bt_xml": os.path.join(
            behavior_tree, "simple_navigation.xml"
        ),
    }
    missing_bt = [path for path in bt_paths.values() if not os.path.isfile(path)]
    if missing_bt:
        return _abort("behavior tree XML not found: " + ", ".join(missing_bt))

    spec_note = "not written (stack_spec_out empty)"
    if spec_out:
        try:
            spec = stack_spec.build_stack_spec(
                stack, selection, extra={"launch_pid": os.getpid()}
            )
            stack_spec.write_stack_spec(spec, spec_out)
            spec_note = spec_out
        except OSError as exc:
            # Provenance is not optional. A run whose configuration cannot be recovered
            # from its own output is exactly what the rebuild exists to stop producing.
            return _abort(f"could not write stack spec to {spec_out}: {exc}")

    sim_time = {"use_sim_time": use_sim_time}

    # Load order is the whole mechanism: parity first, role second, ablation last.
    controller_params = [common["controller_common"], controller_yaml]
    if ablation_yaml:
        controller_params.append(ablation_yaml)
    controller_params.append(sim_time)

    def nav2_node(package, name, parameters):
        return Node(
            package=package,
            executable=name,
            name=name,
            output="screen",
            parameters=parameters,
        )

    nodes = [
        nav2_node("nav2_controller", "controller_server", controller_params),
        nav2_node(
            "nav2_planner", "planner_server", [common["planner_common"], planner_yaml, sim_time]
        ),
        nav2_node("nav2_smoother", "smoother_server", [common["smoother_server"], sim_time]),
        nav2_node(
            "nav2_bt_navigator", "bt_navigator", [common["bt_navigator"], bt_paths, sim_time]
        ),
        nav2_node("nav2_behaviors", "behavior_server", [common["behavior_server"], sim_time]),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {"node_names": LIFECYCLE_NODES},
                sim_time,
                {"autostart": True},
            ],
        ),
    ]

    banner = (
        f"[nav2_stack] controller={selection['controller']} planner={selection['planner']} "
        f"ablation={selection['ablation']} — preflight OK, spec: {spec_note}"
    )
    return [LogInfo(msg=banner), *nodes]


def generate_launch_description():
    """Declare the composition arguments and defer node construction to the preflight."""
    stack = stack_spec.stack_dir(get_package_share_directory("bumperbot_navigation"))

    def describe(role, purpose):
        """Build help text listing what is actually on disk for a role."""
        try:
            return f"{purpose} One of {stack_spec.available(stack, role)}."
        except OSError:
            return f"{purpose} (fragment directory unreadable)"

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument(
                "controller",
                default_value="rpp",
                description=describe("controller", "Local controller."),
            ),
            DeclareLaunchArgument(
                "planner",
                default_value="smac2d",
                description=describe("planner", "Global planner."),
            ),
            DeclareLaunchArgument(
                "ablation",
                default_value=stack_spec.NO_ABLATION,
                description=describe("ablation", "Single-variable override, loaded last."),
            ),
            DeclareLaunchArgument(
                "stack_spec_out",
                default_value=DEFAULT_STACK_SPEC_OUT,
                description=(
                    "Path for the JSON provenance record of the composed stack. The "
                    "benchmark harness reads it and copies it into each run directory. "
                    "Set empty to disable writing."
                ),
            ),
            OpaqueFunction(function=_compose_stack),
        ]
    )
