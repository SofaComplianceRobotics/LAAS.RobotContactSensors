import Sofa

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
                            addJointsActuators=False)
        robot = self.getChild("Robot")
        mechanical = robot.Model.getMechanicalState()
        mechanical.showObject = True
        mechanical.showObjectScale = 0.05
        mechanical.drawMode = 0
    

# Test/example scene
def createScene(rootnode):

    from modules.header import addHeader, addSolvers
    import Sofa.ImGui as MyGui
    from math import pi

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
    for i in range(len(robot.getMechanicalState().position.value)):
        joint = robot.addObject('JointConstraint', template='Vec1', name='joint' + str(i), index=i, valueType="angle", value=0)
        MyGui.MyRobotWindow.addSetting("Joint" + str(i), joint.value, -pi, pi)

    return
