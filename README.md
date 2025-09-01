# LAAS.RobotContactSensors

Using SOFA to simulate 200 sensors used to provide a skin to the TALOS humanoid robot in LAAS.

## Required plugins

- SofaPython3: 
    `git clone git@github.com:sofa-framework/SofaPython3.git`
- SofaGLFW (on the robotics branch): 
    `git clone git@github.com:SofaComplianceRobotics/SofaGLFW.git`
- Sofa.RigidBodyDynamics:     
    `git clone https://github.com/olivier-roussel/Sofa.RigidBodyDynamics`

## Installation 

1. Compile SOFA from source following the instructions from the SOFA website: https://www.sofa-framework.org/download/
2. Clone the plugins into common directory e.g. `plugins`:
    ```
    |-- SOFA
      |-- plugins
      |   |-- SofaGLFW
      |   |-- SofaPython3
      |   |-- Sofa.RigidBodyDynamics
      |   |-- CMakeLists.txt
      |-- build
      |-- src
    ```
3. Add the plugins to SOFA by editing the `CMakeLists.txt` file in the `plugins` directory:
    ```cmake
    cmake_minimum_required(VERSION 3.10)
    sofa_add_subdirectory(plugin SofaPython3 SofaPython3 ON)
    sofa_add_subdirectory(plugin SofaGLFW SofaGLFW ON)
    sofa_add_subdirectory(plugin Sofa.RigidBodyDynamics Sofa.RigidBodyDynamics ON)
    ```
4. Add to Cmake the path to the CMakeLists.tkt file by adding the following variable: `SOFA_EXTERNAL_DIRECTORIES=PATH_TO_/plugins` 
5. Install pinocchio (to model the rigid robot):
    1. Follow installation procedure from: https://stack-of-tasks.github.io/pinocchio/download.html
    2. In CMake: `CMAKE_PREFIX_PATH = /opt/openrobots/lib/cmake/`
5. Now that the plugins have been added you can recompile SOFA.
