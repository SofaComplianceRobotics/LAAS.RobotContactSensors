from pysdf import SDF
from pysdf import Link, State, Model, Link
import os 

def createScene(rootnode):
    import numpy as np
    from modules.header import addHeader, addSolvers
    from modules.robot import TalosHumanoidRobot
    from modules.patch import Patch
    from modules.ball import Ball
    import Sofa.ImGui as MyGui
    from math import pi
    from splib3.numerics import Quat
    from modules.robotconfigurations import talos_ctrl_joint_infos_grasp as talosInitConfiguration

    settings, modelling, simulation = addHeader(rootnode, inverse=False, withCollision=True)

    addSolvers(simulation, rayleighStiffness=0.001)
    rootnode.VisualStyle.displayFlags = ["showVisual"]

    # Units are in m, kg, s
    # Robot
    simulation.addChild(TalosHumanoidRobot("data/talos.urdf"))
    robot = simulation.TalosHumanoidRobot.Robot
    # robot.init()

    # Direct problem
    names = robot.Joints.children
    positions = np.copy(robot.getMechanicalState().position.value)
    for i in range(len(positions)):
        jointName = names[i+1].name.value
        value = 0 if jointName not in talosInitConfiguration else talosInitConfiguration[jointName].pos_desired
        positions[i] = value
        joint = robot.addObject('JointConstraint', template='Vec1', name='joint' + str(i), index=i, 
                                valueType="angle", 
                                value=value
                                )
        MyGui.MyRobotWindow.addSetting(jointName, joint.value, -pi, pi)
    # This does not work I don't know why
    # Thus we have to hard code the initial configuration in the first call of URDFModelLoader
    robot.getMechanicalState().position.value = positions

    # Load the Talos robot model from an SDF file
    filePath = os.path.dirname(os.path.realpath(__file__))+"/data/talos.sdf"
    talos = SDF.from_file(filePath)

    # Read the cells position from the sdf file
    patchs = {}
    # There is only one model in the file
    for link in talos.model.links:
        # The cells position are in the visual part of the link
        for visual in link.visuals:
            if "cell" in visual.name:
                name = visual.name.split('_')

                # The joint name is the two or three first elements of the visual name
                jointName = "_".join(name[0:3]) if name[0] == "arm" else "_".join(name[0:2])
                if jointName not in patchs:
                    patchs[jointName] = {}

                # Add the cell position and orientation to the patch
                patch = patchs[jointName]
                cell = {
                    'position': visual.pose.position,
                    'orientation': visual.pose.orientation
                }
                patch["cell_" +name[-1]] = cell
                patchs[jointName] = patch

    # Hard coded positions 
    patchOrigins = [[0.,     0.,     0.0722, 0.,     0.,     0.,     1.    ],
                    [0.00493, 0.294,   0.35093, 0.,      0.,      0.,      1.     ],
                    [0.00493, 0.294,   0.35093, 0.,      0.,      0.,      1.     ],
                    [0.02493, 0.294,   0.07273, 0.,      0.,      0.,      1.     ],
                    [ 0.00493,  0.294,   -0.19157,  0.,       0.,       0.,       1.     ],
                    [ 0.00493, -0.294,    0.35093,  0.,       0.,       0.,       1.     ],
                    [ 0.00493, -0.294,    0.35093,  0.,       0.,       0.,       1.     ],
                    [ 0.02493, -0.294,    0.07273,  0.,       0.,       0.,       1.     ],
                    [ 0.00493, -0.294,   -0.19157,  0.,       0.,       0.,       1.     ]]


    # Create the patches
    patchIndex = 0
    for patch in patchs:
        cellsPositions = []
        for cell in patchs[patch]:
            c = patchs[patch][cell]
            cellsPositions.append(list(c['position'] + c['orientation'])) 

        # patchOrigin = None
        index = None
        for link in robot.Joints.children:
            if patch in link.name.value:
                index = link.jointMapping.index.value
                # patchOrigin = np.copy(link.getMechanicalState().position.value[0])
                # print("Patch", patch, "index", index, "origin", patchOrigin)
                break
        
        if index is not None:
            Patch(simulationNode=simulation, attachNode=robot.Model, 
                  attachIndex=index, name="Patch"+patch, 
                  cellsPositions=cellsPositions, 
                  origin=patchOrigins[patchIndex])
            patchIndex += 1
    
    Ball(simulation, position=[0.3, 0, 0.3])

    return