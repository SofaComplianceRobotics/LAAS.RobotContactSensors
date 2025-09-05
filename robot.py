import Sofa
import numpy as np

class TalosHumanoidRobot(Sofa.Prefab):

    def __init__(self):
        Sofa.Prefab.__init__(self)
        self.name = 'TalosHumanoidRobot'

        # Add the robot model to the scene graph
        self.__addRobot()

    def __addRobot(self):

        # Robot node
        settings = self.addChild('Settings')
        settings.addObject("RequiredPlugin", name="Sofa.RigidBodyDynamics") # Needed to use components [URDFModelLoader]
        settings.addObject('RequiredPlugin', name='Sofa.Component.Mapping.NonLinear') # Needed to use components [RigidMapping]  
        settings.addObject('RequiredPlugin', name='Sofa.Component.Mass') # Needed to use components [UniformMass]
        settings.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant') # Needed to use components [MeshTopology]  
        settings.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D') # Needed to use components [OglModel]  

        self.addObject('URDFModelLoader', 
                        filename="data/talos.urdf", 
                        modelDirectory="data/meshes/", 
                        useFreeFlyerRootJoint=False, 
                        printLog=False, 
                        addCollision=False, 
                        addJointsActuators=False,
                        qInit = [ 0., 0., -0.448041, 0.896082, -0.448041, 0., 0., 0., 
                                 -0.448041, 0.896082, -0.448041, 0., 0., 0., -0.75847, 
                                 0.173046, 0.2502, -1.725366, 0.6, 0.9, 0., 0., 0., 0., 
                                 0., 0.75847, -0.173046, -0.2502, -1.725366, -0.6, -0.9, 0., 0., 0., 0., 0., 0., 0.]
                                 )
        robot = self.getChild("Robot")
        mechanical = robot.Model.getMechanicalState()
        mechanical.showObject = True
        mechanical.showObjectScale = 0.01
        mechanical.drawMode = 0
    

# Test/example scene
def createScene(rootnode):

    from modules.header import addHeader, addSolvers
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
    # simulation.TalosHumanoidRobot.URDFModelLoader.qInit.value = positions

    return
