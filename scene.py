def createScene(rootnode):
    import numpy as np
    from modules.header import addHeader, addSolvers
    from robot import TalosHumanoidRobot
    from patch import Patch
    import Sofa.ImGui as MyGui
    from math import pi
    from modules.robotconfigurations import talos_ctrl_joint_infos_grasp as talosInitConfiguration

    settings, modelling, simulation = addHeader(rootnode, inverse=False, withCollision=False, friction=0)

    addSolvers(simulation, rayleighStiffness=0.001)
    rootnode.VisualStyle.displayFlags = ["showVisual"]

    # Units are in m, kg, s
    rootnode.dt = 0.01
    rootnode.gravity = [0., -9.81, 0.]

    # Robot
    simulation.addChild(TalosHumanoidRobot())
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

    Patch(simulationNode=simulation, attachNode=robot.Model, attachIndex=30, name="Patch", cellGrid=[5, 5])

    return