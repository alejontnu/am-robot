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
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    packages=setuptools.find_packages(where='src'),
    package_dir={"": "src"},
    install_requires=["argparse", "gcodeparser", "pandas", "plotly", "numpy", "pyserial"],  # frankx must be installed from source
    python_requires=">=3.7",
)
