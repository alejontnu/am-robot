import setuptools

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setuptools.setup(
    name="am-robot",
    version="0.0.1",
    author="Alexander",
    author_email="alro2011@hotmail.com",
    description="Controller package for using the Franka Emika Panda robot manipulator for additive manufacturing",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/alejontnu/am-robot",
    project_urls={
        "Bug Tracker": "https://github.com/alejontnu/am-robot/issues",
    },
    #entry_points = {
    #    'console_scripts': ['run=am-robot.command_line:main'],
    #},
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    packages=setuptools.find_packages(where='src'),
    package_dir={"": "src"},
    #package_data={'': ['benchmark.txt']},
    include_package_data=True,
    install_requires=["argparse","gcodeparser","pandas","matplotlib","plotly","numpy"],# frankx must be installed through cmake for dependencies??
    python_requires=">=3.7",
)
