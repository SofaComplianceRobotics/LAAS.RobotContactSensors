import Sofa
import numpy as np
import os

from utils import projectdirpath


class TalosHumanoidRobot(Sofa.Prefab):
    '''
    Talos humanoid robot prefab loaded from URDF file.
    ️Args:
        urdf (str): Path to the URDF file of the robot.
    '''

    def __init__(self, urdf=os.path.join(projectdirpath,"data/talos.urdf")):
        Sofa.Prefab.__init__(self)
        self.name = 'TalosHumanoidRobot'
        self.urdf = urdf

        # Add the robot model to the scene graph
        self.__addRobot()

    def __addRobot(self):
        '''
        Adds the Talos humanoid robot to the scene graph using URDFModelLoader.
        '''

        # Required plugins
        settings = self.addChild('Settings')
        settings.addObject("RequiredPlugin", name="Sofa.RigidBodyDynamics") # Needed to use components [URDFModelLoader]
        settings.addObject('RequiredPlugin', name='Sofa.Component.Mapping.NonLinear') # Needed to use components [RigidMapping]  
        settings.addObject('RequiredPlugin', name='Sofa.Component.Mass') # Needed to use components [UniformMass]
        settings.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant') # Needed to use components [MeshTopology]  
        settings.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D') # Needed to use components [OglModel]  

        # Load the robot from URDF
        self.addObject('URDFModelLoader', 
                        filename=self.urdf, # Path to the URDF file
                        modelDirectory=os.path.join(projectdirpath,"data/meshes/"), # Path to the directory containing the robot meshes
                        useFreeFlyerRootJoint=False, 
                        printLog=False, 
                        addCollision=False, 
                        addJointsActuators=False,
                        # This is is supposed to set the initial configuration of the robot
                        # so that the simulation starts in the correct pose
                        # It does work, but the robot comes back to a default pose after the first time step
                        # qInit = [ 0., 0., -0.448041, 0.896082, -0.448041, 0., 0., 0., 
                        #          -0.448041, 0.896082, -0.448041, 0., 0., 0., -0.75847, 
                        #          0.173046, 0.2502, -1.725366, 0.6, 0.9, 0., 0., 0., 0., 
                        #          0., 0.75847, -0.173046, -0.2502, -1.725366, -0.6, -0.9, 0., 0., 0., 0., 0., 0., 0.]
                                 )
        robot = self.getChild("Robot") # The URDFModelLoader creates a child node named "Robot"

        # Show / hide the robot mechanical representation using the following lines
        mechanical = robot.Model.getMechanicalState() 
        mechanical.showObject = False
        mechanical.showObjectScale = 0.01
        mechanical.drawMode = 0
    

# Example of scene using the TalosHumanoidRobot prefab
def createScene(rootnode):

    from modules.header import addHeader, addSolvers
    import Sofa.ImGui as MyGui
    from math import pi
    from modules.robotconfigurations import talos_ctrl_joint_infos_grasp as talosInitConfiguration

    # Scene setup. Add header (solvers, visual style, gravity, time step, etc.)
    settings, modelling, simulation = addHeader(rootnode, inverse=False, withCollision=False, friction=0)
    addSolvers(simulation, rayleighStiffness=0.001)
    rootnode.VisualStyle.displayFlags = ["showVisual"]

    # Units are in m, kg, s
    rootnode.dt = 0.01 # time step of the simulation
    rootnode.gravity = [0., -9.81, 0.]

    # Robot
    simulation.addChild(TalosHumanoidRobot()) # This is how we add the Talos robot prefab to the scene
    robot = simulation.TalosHumanoidRobot.Robot

    # Direct problem
    jointNames = robot.Joints.children
    positions = np.copy(robot.getMechanicalState().position.value)
    for i in range(len(positions)):
        jointName = jointNames[i+1].name.value # Get the name of each joint (loaded from URDF)
        value = 0 if jointName not in talosInitConfiguration else talosInitConfiguration[jointName].pos_desired
        positions[i] = value
        # We add a JointConstraint to each joint to fix its value
        joint = robot.addObject('JointConstraint', template='Vec1', name='joint' + str(i), index=i, 
                                valueType="angle", 
                                value=value # Set the initial value of the joint (will be applied after the first time step)
                                )
        # We add the joint to the MyRobotWindow GUI for easy manipulation
        # This will create a slider for each joint in the GUI (with the name of the joint from the URDF)
        # We also set a range for the slider between -pi and pi
        MyGui.MyRobotWindow.addSetting(jointName, joint.value, -pi, pi)

    # The following is suppose to set the initial values of the joints
    # So that the simulation starts in the correct configuration, but it does not work either I don't know why
    # robot.getMechanicalState().position.value = positions

    return
