from setuptools import find_packages, setup


package_name = "scan_feature_matcher"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/feature_matcher.launch.py"]),
        (f"share/{package_name}/config", ["config/feature_matcher.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cursor",
    maintainer_email="cursor@example.com",
    description="Single-line LaserScan feature extraction and frame-to-frame matching.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "scan_feature_matcher = scan_feature_matcher.scan_feature_matcher_node:main",
        ],
    },
)
