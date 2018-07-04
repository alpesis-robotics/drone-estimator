# Drone Estimator

## Getting Started

```
    # install Qt5
    $ export Qt5Core_DIR=/usr/local/opt/qt/lib/cmake/Qt5Core
    $ export Qt5Network_DIR=/usr/local/opt/qt/lib/cmake/Qt5Network
    $ export Qt5Widgets_DIR=/usr/local/opt/qt/lib/cmake/Qt5Widgets

    $ mkdir _build && cd _build
    $ cmake ..
    $ make
    $ ./CPPSim
```

Simulator Commands

- ``right click``: choose scenario;
- ``left drag``: rotate;
- ``X + left drag``: pan;
- ``arrow keys``: apply external force;
- ``C``: clear all graphs;
- ``R``: reset simulation;
- ``Space``: pause simulation.

## Codes

```
    drone-controller/
          +---- images/              scenario images
          +---- config/              configuration files for controller and vehicle
          +---- lib/                 external libraries
          +---- project/             IDE configurations
          +---- src/                 codes
          +---- tools/               tools
          +---- CMakeLists.txt
          +---- README.rst
```

## Solution: Scenario 06_SensorNoise

By calculating the standard deviation of ``Quad.GPS.X`` and ``Quad.IMU.AX`` data located at ``config/log/``,
the charts in the scenario ``06_SensorNoise`` would be shown that the dashed lines turn green within +/- 1
sigma bound on the y-axis capturing the value of approximately 68% of the respective measurements.

The formula of standard deviation is as following:


![equation](http://latex.codecogs.com/gif.latex?\mu=\frac{1}{N}{\sum_{i=0}^N}x_i)

  

