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
    simulation.addChild(TalosHumanoidRobot("data/talos_torso.urdf"))
    robot = simulation.TalosHumanoidRobot.Robot

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

    # Add a patch
    Patch(simulationNode=simulation, attachNode=robot.Model, attachIndex=13, name="PatchRightArm", cellGrid=[4, 4], 
          origin=[0.00487 + 0.01, -0.297262 + 0.06, -0.111945 + 0.08, 0.5233419, -0.5233419, -0.4753564, -0.4753564])
    
    Patch(simulationNode=simulation, attachNode=robot.Model, attachIndex=7, name="PatchLeftArm", cellGrid=[4, 4], 
          origin=[-0.00487, 0.297262 - 0.06, 0.111945 - 0.145, 0.5233419, 0.5233419, -0.4753564, 0.4753564])

    Patch(simulationNode=simulation, attachNode=robot.Model, attachIndex=2, name="PatchTorso", cellGrid=[3, 24], 
          origin=[0.08, -0.1, 0.2, 0.0, 0.707, 0.0, 0.707])
    
    Ball(simulation, position=[0.3, 0, 0.3])

    return