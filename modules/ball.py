import Sofa

class Ball(Sofa.Prefab):

    def __init__(self, attachNode, position=[0, 0, 0]):
        Sofa.Prefab.__init__(self)
        attachNode.addChild(self)
        self.translation = position

        self.__addSettings()
        self.__addMechanical()
        self.__addVisual()
        self.__addCollision()

    def __addSettings(self):
        settings = self.addChild("Settings")
        settings.addObject("RequiredPlugin", name="Shell")
        settings.addObject('RequiredPlugin', name='Sofa.Component.IO.Mesh') # Needed to use components [MeshOBJLoader]  
        settings.addObject('RequiredPlugin', name='Sofa.Component.Mass') # Needed to use components [UniformMass]  
        settings.addObject('RequiredPlugin', name='Sofa.Component.StateContainer') # Needed to use components [MechanicalObject]  
        settings.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant') # Needed to use components [MeshTopology] 
        settings.addObject('RequiredPlugin', name='Sofa.Component.Collision.Geometry') # Needed to use components [LineCollisionModel,PointCollisionModel]  
        settings.addObject('RequiredPlugin', name='Sofa.Component.Mapping.Linear') # Needed to use components [IdentityMapping]  
        settings.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D') # Needed to use components [OglModel]  

    def __addMechanical(self):
        self.addObject("MeshOBJLoader", filename="mesh/ball.obj", triangulate=True, scale=0.17, translation=self.translation)
        self.addObject("MeshTopology", src=self.MeshOBJLoader.linkpath)
        self.addObject("MechanicalObject", template="Rigid3")
        self.addObject("UniformMass", totalMass=0.050) # 50g
        self.addObject("TriangularBendingFEMForceField", 
                       youngModulus=3e4,
                       poissonRatio=0.4,
                       thickness=0.002) # 2mm

    def __addVisual(self):
        visual = self.addChild("Visual")
        visual.addObject("OglModel", src=self.MeshOBJLoader.linkpath, color=[0.1, 0.1, 1, 0.3])
        visual.addObject("IdentityMapping")

    def __addCollision(self):
        collision = self.addChild("Collision")
        collision.addObject("MeshTopology", src=self.MeshTopology.linkpath)
        collision.addObject("MechanicalObject")
        collision.addObject("PointCollisionModel")
        collision.addObject("LineCollisionModel")
        collision.addObject("TriangleCollisionModel")
        collision.addObject("IdentityMapping")

def createScene(rootnode):
    Ball(rootnode)