![GitHub contributors](https://img.shields.io/github/contributors/alejontnu/am-robot?label=Number%20of%20button%20mashers&style=plastic)

# am-robot
Package for using a Franka Emika Panda robot maniplator for additive manufacturing. Done as past of two student projects:
| Project               | Commits     |
|-----------------------|-------------|
|Specialization project | 1 - 63      |
|Master's project       | 64 - latest |


## About the package



## Feature summary
Features are under development and spesifics can change as the project continues...
- Parsing and pre-processing of G-code input file
- Robot motion trajectory generation from G-code targets utilizing Frankx/Ruckig
- Automatic build area level detection and non-level compensation
- Visualization of tool-path trajectories and bed level mesh
- Generalization of tool and robot choice (Still needs work)
- Non-realtime syncronization of tool to robot (Planned for real-time)

## Getting started
```
pip install git+https://github.com/alejontnu/am-robot.git@main
pip install -r test_requirements.txt
pip install -e .
pytest
```
This package is built using Python 3, specifically tested on Python 3.8

### Dependencies
- plotly: For visualizion of trajectories and bed level mesh
- argparse: For parsing input arguments
- gcodeparser: For initial parsing of G-code into Dictionary object containing G-code lines and more
- pyserial

Franka Emika Panda robot is dependent on:
- libfranka: C++ library for controlling Franka Emika Panda robot. Build from source (link)
- frankx: Python wrapper around libfranka. Build from source (link) to include dependencies, Ruckig and Eigen


## Current status
Preliminary result show need for better control of the extrusion process. As can be seen in the test print below, motion trajectories are aborted if the robot encounters an error (such as acceleration contraint violation) and the remainder of the motion trajectory is left incomplete.

(image here)

## Known issues
- Currently up to user to determine feasable build area size, can possibly implement an automatic check for this.
- Different slicer software result in inconsistent pre-processing of the G-code generated.
- Convert to true 3D with use of additional DoF. Currently building 2.5D layer-by-layer.
- Bed probing method leaves toolhead with an offset from where it should be placed.
- Motion trajectories are sometimes aborted when an robot error is detected, need to reduce or handle such cases.
