import Sofa
import os
from utils import projectdirpath

class CementBag(Sofa.Prefab):
    """
    A prefab representing a deformable cement bag.
    """

    def __init__(self, attachNode, position=[0, 0, 0]):
        Sofa.Prefab.__init__(self)
        attachNode.addChild(self)
        self.translation = position
        self.rotation = [90, -10, 0]
        self.scale = 0.009

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
        s = 1 * self.scale
        self.addObject("MeshVTKLoader", name="loader", 
                        filename=os.path.join(projectdirpath,"data/meshes/cement-bag.vtk"), scale=self.scale, translation=self.translation, rotation=self.rotation)
        self.addObject("MeshTopology", src=self.loader.linkpath)
        self.addObject("MechanicalObject", template="Vec3")
        self.addObject("UniformMass", totalMass=10) # 10kg
        self.addObject("TetrahedronFEMForceField", 
                        youngModulus=5e4,
                        poissonRatio=0.4) 

    def __addVisual(self):
        visual = self.addChild("Visual")
        visual.addObject("MeshOBJLoader", filename=os.path.join(projectdirpath,"data/meshes/cement-bag.obj"), 
                         triangulate=True, scale=self.scale, translation=self.translation, rotation=self.rotation)
        visual.addObject("OglModel", src=visual.MeshOBJLoader.linkpath, color=[1,1,1,1],
                         texturename="data/meshes/cement-bag.jpg")
        visual.addObject("BarycentricMapping")

    def __addCollision(self):
        collision = self.addChild("Collision")
        collision.addObject("MeshSTLLoader", filename=os.path.join(projectdirpath,"data/meshes/cement-bag-collision.stl"), 
                            triangulate=True, scale=self.scale*0.85, translation=self.translation, rotation=self.rotation)
        collision.addObject("MeshTopology", src=self.Collision.MeshSTLLoader.linkpath)
        collision.addObject("MechanicalObject")
        collision.addObject("PointCollisionModel")
        collision.addObject("LineCollisionModel")
        collision.addObject("TriangleCollisionModel")
        collision.addObject("BarycentricMapping")


# Function to create the scene with a cement bag
def createScene(rootnode):

    # Scene setup. Add header (solvers, visual style, gravity, time step, etc.)
    from modules.header import addHeader, addSolvers
    settings, modelling, simulation = addHeader(rootnode, inverse=False, withCollision=False, friction=0)
    addSolvers(simulation, rayleighStiffness=0.001)
    rootnode.VisualStyle.displayFlags = ["showVisual"]

    # Add a cement bag to the scene
    CementBag(simulation) # It will just fall under gravity