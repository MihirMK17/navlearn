import os
from glob import glob

from setuptools import find_packages, setup

package_name = "navlearn_analysis"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Campaign shell drivers ride along in share/ for reference; operators run them
        # from the checkout (they cd into the workspace and manage results/ there).
        (os.path.join("share", package_name, "scripts"), glob("scripts/*.sh")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mihirmk",
    maintainer_email="mihir.kulkarni17@gmail.com",
    description="NavLearn campaign harness, probes, claim analyses and figure tooling.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Harness
            "multi_run_harness = navlearn_analysis.harness.multi_run_harness:main",
            # Probes (also spawned via `python -m` by the harness; entry points make
            # them individually runnable for diagnosis)
            "rate_monitor = navlearn_analysis.probes.rate_monitor:main",
            "compute_sampler = navlearn_analysis.probes.compute_sampler:main",
            "costmap_corruption_monitor = "
            "navlearn_analysis.probes.costmap_corruption_monitor:main",
            "collision_positive_control = "
            "navlearn_analysis.probes.collision_positive_control:main",
            # Claim analyses
            "run_yaw_claim = navlearn_analysis.claims.run_yaw_claim:main",
            "analyze_curve_campaign = "
            "navlearn_analysis.claims.analyze_curve_campaign:main",
            "compare_yaw_slopes = navlearn_analysis.claims.compare_yaw_slopes:main",
            "recompute_ttr_from_bags = "
            "navlearn_analysis.claims.recompute_ttr_from_bags:main",
            "run_claim2_models = navlearn_analysis.claims.run_claim2_models:main",
            "rate_mechanism = navlearn_analysis.rate_mechanism:main",
            "mechanism_by_bin = navlearn_analysis.mechanism_by_bin:main",
            "sigma_curves = navlearn_analysis.sigma_curves:main",
            # Map / world tooling
            "world_to_map = navlearn_analysis.world_to_map:main",
            "map_ambiguity = navlearn_analysis.map_ambiguity:main",
            "map_spread_gate = navlearn_analysis.map_spread_gate:main",
            "kidnap_feasibility = navlearn_analysis.kidnap_feasibility:main",
            # Stack parity gate
            "validate_nav2_stack = navlearn_analysis.stack.validate_nav2_stack:main",
            # Figure generators — every image in the repo README regenerates from these
            "render_map = navlearn_analysis.figures.render_map:main",
            "plot_yaw_cliff = navlearn_analysis.figures.plot_yaw_cliff:main",
            "animate_kidnap_recovery = "
            "navlearn_analysis.figures.animate_kidnap_recovery:main",
        ],
    },
)
