class TalosHumanoidRobot:

    def __init__(self, rootnode):
        self.rootnode = rootnode

        # Add the robot model to the scene graph
        self.node = rootnode.addChild('TalosHumanoidRobot')
        self.__addRobot()

    def __addRobot(self):

        # Robot node
        settings = self.node.addChild('Settings')
        settings.addObject("RequiredPlugin", name="Sofa.RigidBodyDynamics") # Needed to use components [URDFModelLoader]
        settings.addObject('RequiredPlugin', name='Sofa.Component.Mapping.NonLinear') # Needed to use components [RigidMapping]  
        settings.addObject('RequiredPlugin', name='Sofa.Component.Mass') # Needed to use components [UniformMass]
        settings.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant') # Needed to use components [MeshTopology]  
        settings.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D') # Needed to use components [OglModel]  

        self.node.addObject('URDFModelLoader', 
                            filename="data/talos.urdf", 
                            modelDirectory="data/meshes/", 
                            useFreeFlyerRootJoint=False, 
                            printLog=False, 
                            addCollision=False, 
                            addJointsActuators=False)
        robot = self.node.getChild("Robot")
        mechanical = robot.Model.getMechanicalState()
        mechanical.showObject = True
        mechanical.showObjectScale = 30
        mechanical.drawMode = 0
    

# Test/example scene
def createScene(rootnode):

    from modules.header import addHeader, addSolvers
    import Sofa.ImGui as MyGui
    from math import pi

    settings, modelling, simulation = addHeader(rootnode, inverse=False, withCollision=False, friction=0)

    addSolvers(simulation, rayleighStiffness=0.001)
    rootnode.VisualStyle.displayFlags = ["showVisual"]

    rootnode.dt = 0.001
    rootnode.gravity = [0., -9810., 0.]

    # Robot
    robot = TalosHumanoidRobot(simulation).node.Robot

    # Direct problem
    for i in range(6):
        joint = robot.addObject('JointConstraint', template='Vec1', name='joint' + str(i), index=i, valueType="angle", value=0)
        MyGui.MyRobotWindow.addSetting("Joint" + str(i), joint.value, -pi, pi)
        robot.addObject("RestShapeSpringsForceField", template='Vec1', points=[3], stiffness=1e12)

    return
