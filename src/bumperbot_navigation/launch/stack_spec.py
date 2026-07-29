"""Discovery, validation and provenance for the composed nav2_stack configuration.

Why this module exists
    Two defects in the retired campaign came from configuration that was *claimed* rather
    than *observed*. Valid profile names were hardcoded in several files and drifted apart,
    and the benchmark harness recorded a ``--profile`` flag it had no power to enforce —
    ``benchmarks.launch.py`` declared the argument and never consumed it, so a run's
    recorded stack and its actual stack could disagree with nothing to catch it.

    Both are addressed structurally here rather than by convention:

    Discovery
        The set of legal fragment names is derived from the filesystem, never written down.
        ``controllers/mppi.yaml`` existing is what makes ``controller:=mppi`` legal. There
        is no list to drift, so adding a controller means adding one file and nothing else.

    Provenance
        :func:`build_stack_spec` records what the launch actually composed — every fragment
        path with its SHA-256, plus the resolved values of the parameters that define a
        cell. Written to disk at launch and copied into each run directory by the harness,
        it makes "which configuration produced this CSV" answerable from the data itself,
        permanently, without trusting any flag a human typed.

Location
    Lives in ``bumperbot_navigation`` because that package owns ``config/nav2_stack/``.
    ``navlearn_benchmarks`` loads it from the installed share directory via
    :func:`load_stack_spec_module`-style importlib, which keeps the dependency pointing
    from the benchmark layer to the robot packages and never the reverse.
"""

import hashlib
import json
import os

import yaml

# Role -> subdirectory of config/nav2_stack/. The keys are the launch argument names.
ROLE_DIRS = {
    "controller": "controllers",
    "planner": "planners",
    "localizer": "localizers",
    "ablation": "ablations",
}

# Selecting no ablation is legal and is not a fragment on disk.
NO_ABLATION = "none"


def stack_dir(package_share):
    """Return the absolute path to config/nav2_stack/ inside an installed package."""
    return os.path.join(package_share, "config", "nav2_stack")


def available(stack, role):
    """List the legal values for a role, newest state of the filesystem being the truth.

    Returns names sorted for stable help text and error messages. ``ablation`` always
    includes the ``none`` sentinel, which selects no overlay.
    """
    if role not in ROLE_DIRS:
        raise ValueError(f"unknown role '{role}'; expected one of {sorted(ROLE_DIRS)}")

    role_dir = os.path.join(stack, ROLE_DIRS[role])
    names = sorted(
        entry[: -len(".yaml")]
        for entry in os.listdir(role_dir)
        if entry.endswith(".yaml")
    ) if os.path.isdir(role_dir) else []

    if role == "ablation":
        return [NO_ABLATION, *names]
    return names


def fragment_path(stack, role, name):
    """Return the fragment path for a role/name pair, or None for the 'none' ablation."""
    if role == "ablation" and name == NO_ABLATION:
        return None
    return os.path.join(stack, ROLE_DIRS[role], f"{name}.yaml")


def validate_selection(stack, role, name):
    """Return an error string if a role/name selection is not available, else None."""
    legal = available(stack, role)
    if name not in legal:
        return f"{role}='{name}' is not one of {legal}"

    path = fragment_path(stack, role, name)
    if path is not None and not os.path.isfile(path):
        return (
            f"{role}='{name}' resolves to {path}, which does not exist. If the file is "
            "present in src/, the package needs rebuilding — config/ is installed to "
            "share/ at build time."
        )
    return None


def sha256(path):
    """Return the hex SHA-256 of a file, read in chunks to bound memory."""
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load(path):
    """Parse a YAML fragment, returning an empty dict for an empty file."""
    with open(path, "r") as handle:
        return yaml.safe_load(handle) or {}


def _merge(base, overlay):
    """Recursively merge overlay into base, mirroring how ROS 2 layers parameter files."""
    merged = dict(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def compose(paths):
    """Merge fragments in load order and return the resulting parameter tree.

    Mirrors the ROS 2 rule that a node's ``parameters=[...]`` list is applied in order with
    later entries winning, so the returned tree is what the servers will actually see.
    """
    result = {}
    for path in paths:
        if path is not None:
            result = _merge(result, _load(path))
    return result


# Parameters that define a cell's identity. Recorded verbatim in the spec so a run can be
# audited without re-reading the config that produced it — which may have since changed.
IDENTITY_PARAMS = (
    ("controller_plugin", ("controller_server", "ros__parameters", "FollowPath", "plugin")),
    ("planner_plugin", ("planner_server", "ros__parameters", "GridBased", "plugin")),
    (
        "xy_goal_tolerance",
        ("controller_server", "ros__parameters", "general_goal_checker", "xy_goal_tolerance"),
    ),
    (
        "yaw_goal_tolerance",
        ("controller_server", "ros__parameters", "general_goal_checker", "yaw_goal_tolerance"),
    ),
    (
        "movement_time_allowance",
        ("controller_server", "ros__parameters", "progress_checker", "movement_time_allowance"),
    ),
    (
        "controller_frequency",
        ("controller_server", "ros__parameters", "controller_frequency"),
    ),
    (
        "local_costmap_inflation_radius",
        ("local_costmap", "local_costmap", "ros__parameters",
         "inflation_layer", "inflation_radius"),
    ),
    (
        "local_costmap_cost_scaling_factor",
        ("local_costmap", "local_costmap", "ros__parameters",
         "inflation_layer", "cost_scaling_factor"),
    ),
)


def _dig(tree, path):
    """Walk a nested mapping by key path, returning None if any level is absent."""
    node = tree
    for key in path:
        if not isinstance(node, dict) or key not in node:
            return None
        node = node[key]
    return node


def build_stack_spec(stack, selection, extra=None):
    """Describe the composed stack: fragment hashes plus resolved identity parameters.

    Args:
        stack: path to config/nav2_stack/.
        selection: mapping of role -> chosen name, e.g. ``{"controller": "mppi", ...}``.
        extra: optional mapping merged into the result, for caller-supplied context such
            as the git SHA or a timestamp. Callers own those because this module must stay
            free of clock and repository access to remain deterministic under test.

    Returns:
        A JSON-serialisable dict. ``fragments`` lists every file that contributed, in load
        order, each with its SHA-256. ``identity`` holds the resolved parameter values that
        define the cell.
    """
    load_order = [
        ("common/controller_common", os.path.join(stack, "common", "controller_common.yaml")),
        (f"controllers/{selection['controller']}",
         fragment_path(stack, "controller", selection["controller"])),
    ]
    if selection.get("ablation", NO_ABLATION) != NO_ABLATION:
        load_order.append(
            (f"ablations/{selection['ablation']}",
             fragment_path(stack, "ablation", selection["ablation"]))
        )
    load_order += [
        ("common/planner_common", os.path.join(stack, "common", "planner_common.yaml")),
        (f"planners/{selection['planner']}",
         fragment_path(stack, "planner", selection["planner"])),
        ("common/bt_navigator", os.path.join(stack, "common", "bt_navigator.yaml")),
        ("common/behavior_server", os.path.join(stack, "common", "behavior_server.yaml")),
        ("common/smoother_server", os.path.join(stack, "common", "smoother_server.yaml")),
    ]
    if selection.get("localizer"):
        load_order.append(
            (f"localizers/{selection['localizer']}",
             fragment_path(stack, "localizer", selection["localizer"]))
        )

    fragments = [
        {"role": role, "path": path, "sha256": sha256(path)}
        for role, path in load_order
        if path is not None and os.path.isfile(path)
    ]

    composed = compose([path for _, path in load_order])
    identity = {name: _dig(composed, path) for name, path in IDENTITY_PARAMS}

    spec = {
        "schema": "navlearn.stack_spec/1",
        "selection": dict(selection),
        "identity": identity,
        "fragments": fragments,
        "composed_sha256": hashlib.sha256(
            json.dumps(composed, sort_keys=True, default=str).encode()
        ).hexdigest(),
    }
    if extra:
        spec.update(extra)
    return spec


def write_stack_spec(spec, path):
    """Write a stack spec to disk as indented JSON, creating parent directories."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as handle:
        json.dump(spec, handle, indent=2, sort_keys=True, default=str)
        handle.write("\n")
    return path
