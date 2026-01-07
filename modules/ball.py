import Sofa
import os
from utils import projectdirpath

class Ball(Sofa.Prefab):
    """
    A prefab representing a deformable ball.
    """

    def __init__(self, attachNode, position=[0, 0, 0], model="volume", color=[0.1, 0.1, 1, 0.3]):
        Sofa.Prefab.__init__(self)
        attachNode.addChild(self)
        self.translation = position
        self.scale = 0.16
        self.color = color

        self.__addSettings()
        self.__addMechanical(model)
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

    def __addMechanical(self, model):
        if model == "volume":
            s = 1 * self.scale
            self.addObject("MeshVTKLoader", name="loader", 
                          filename=os.path.join(projectdirpath,"data/meshes/ball.vtk"), scale=self.scale, translation=self.translation)
        else:
            self.addObject("MeshOBJLoader", 
                            name="loader", 
                            filename="mesh/ball.obj", 
                            triangulate=True, scale=self.scale, translation=self.translation)
        self.addObject("MeshTopology", src=self.loader.linkpath)
        self.addObject("MechanicalObject", template="Vec3" if model == "volume" else "Rigid3")
        self.addObject("UniformMass", totalMass=0.050) # 50g
        self.addObject("TetrahedronFEMForceField" if model == "volume" else "TriangularBendingFEMForceField", 
                        youngModulus=3e3,
                        poissonRatio=0.4) 
        
        if model != "volume":
            self.HexahedronFEMForceField.thickness=0.002 # 2mm

    def __addVisual(self):
        visual = self.addChild("Visual")
        visual.addObject("MeshOBJLoader", filename="mesh/ball.obj", # From Sofa data repository
                         triangulate=True, scale=self.scale, translation=self.translation)
        visual.addObject("OglModel", src=visual.MeshOBJLoader.linkpath, color=self.color)
        visual.addObject("BarycentricMapping")

    def __addCollision(self):
        collision = self.addChild("Collision")
        collision.addObject("MeshTopology", src=self.Visual.MeshOBJLoader.linkpath)
        collision.addObject("MechanicalObject")
        collision.addObject("PointCollisionModel")
        collision.addObject("LineCollisionModel")
        collision.addObject("TriangleCollisionModel")
        collision.addObject("BarycentricMapping")


# Function to create the scene with the ball
def createScene(rootnode):

    # Scene setup. Add header (solvers, visual style, gravity, time step, etc.)
    from modules.header import addHeader, addSolvers
    settings, modelling, simulation = addHeader(rootnode, inverse=False, withCollision=False, friction=0)
    addSolvers(simulation, rayleighStiffness=0.001)
    rootnode.VisualStyle.displayFlags = ["showVisual"]

    # Add a ball to the scene
    Ball(simulation) # It will just fall under gravity