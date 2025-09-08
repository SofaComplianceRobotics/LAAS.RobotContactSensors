import Sofa
from math import pi, cos, sin

class Cell(Sofa.Prefab):
    """
        Modeling of a cell sensor. Units are m, kg
    """
    sideSize: float=0.01
    centerThickness: float=0.002
    stiffness: float=1e2
    totalMass: float=0.001

    def __init__(self, 
                 name: str,
                 simulationNode: Sofa.Core.Node,
                 attachNode: Sofa.Core.Node,
                 attachIndex: int=0,
                 ):
        Sofa.Prefab.__init__(self)

        self.name = name
        self.simulationNode = simulationNode
        self.attachNode = attachNode
        self.attachIndex = attachIndex

        self.origin = [0., 0., 0.]

        assert simulationNode.getRoot().getChild("Settings") is not None
        self.settings = simulationNode.getRoot().Settings
        
        if self.settings.getChild("Cell") is None:
            self.settings.addChild("Cell")
            self.__addSettings()
        self.positions = self.__addTopology()
        self.__addMechanical()
        self.__addCollision()

    def __addSettings(self):
        """
        Node with the settings, like the required plugins
        """
        settings = self.settings.Cell
        settings.addObject('RequiredPlugin', name='Sofa.Component.Collision.Geometry') # Needed to use components [PointCollisionModel]  
        settings.addObject('RequiredPlugin', name='Sofa.Component.Mapping.Linear') # Needed to use components [IdentityMapping]  
        settings.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant') # Needed to use components [MeshTopology] 
        settings.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.Spring') # Needed to use components [RestShapeSpringsForceField]
        settings.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Projective') # Needed to use components [FixedProjectiveConstraint]  
        settings.addObject('RequiredPlugin', name='Sofa.Component.Mapping.NonLinear') # Needed to use components [RigidMapping]  
        settings.addObject('RequiredPlugin', name='Sofa.Component.Mass') # Needed to use components [UniformMass]  
        settings.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D') # Needed to use components [OglModel]

    def __addTopology(self):
        """
        Topology of the cell, 8 points, 6 tetras.
        """
        s = self.sideSize
        o = self.origin 

        positions = [[o[0], o[1], o[2] + self.centerThickness], # First point is the center up
                     [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0],
                      o, # Last the center down 
                     ]
        for i in range(1, len(positions) - 1):
            angle = 2. * pi / 6. * i
            positions[i] = [s*cos(angle) + o[0], s*sin(angle) + o[1], o[2]]
        self.edges = [[0, 1],[0, 2],[0, 3],[0, 4],[0, 5],[0, 6],
                        [1, 2], [2, 3], [3, 4], [4, 5], [5, 6], [6, 1],
                        [7, 1],[7, 2],[7, 3],[7, 4],[7, 5],[7, 6],
                        [0, 7]]
        self.tetras = [[0, 1, 2, 7],
                        [0, 2, 3, 7],
                        [0, 3, 4, 7],
                        [0, 4, 5, 7],
                        [0, 5, 6, 7],
                        [0, 6, 1, 7],]

        return positions
    
    def __addMechanical(self):
        """
        Adds cell mechanical, deformable and rigid parts.
        """
        all = Sofa.Core.Node("All")
        all.addObject("MeshTopology", position=self.positions)
        all.addObject("MechanicalObject", position=self.positions,
                       showObject=False, showObjectScale=5e-4, drawMode=2)
        all.addObject("UniformMass", totalMass=self.totalMass)

        self.rigidified = self.attachNode.addChild(self.name.value + "RigidPart")
        self.rigidified.addObject("MechanicalObject", position=[self.positions[1:]],
                             showObject=False, showObjectScale=5e1, drawMode=0, showColor=[255, 0, 0, 255])
        self.rigidified.addObject("RigidMapping", index=self.attachIndex, globalToLocalCoords=False)
        self.rigidified.addChild(all)

        topCenterRestPosition = self.attachNode.addChild(self.name.value + "TopCenterRestPosition")
        topCenterRestPosition.addObject("MechanicalObject", position=[self.positions[0]],
                                        showObject=False, showObjectScale=5e1, drawMode=0, showColor=[0, 255, 0, 255])
        topCenterRestPosition.addObject("RigidMapping", index=self.attachIndex, globalToLocalCoords=False)
        topCenterRestPosition.init()
        
        self.deformable = self.simulationNode.addChild(self.name.value + "DeformablePart")
        self.deformable.addObject("MechanicalObject", position=topCenterRestPosition.getMechanicalState().position.value,
                                    showObject=False, showObjectScale=5e1, drawMode=0, showColor=[0, 0, 255, 255])
        self.deformable.addChild(all)

        visual = all.addChild("Visual")
        visual.addObject("MeshTopology", position=self.positions, edges=self.edges, tetras=self.tetras)
        visual.addObject("OglModel", src=visual.MeshTopology.linkpath, color=[1, 0, 0, 1])
        visual.addObject("IdentityMapping")

        self.deformable.addObject("RestShapeSpringsForceField", points=[0], stiffness=self.stiffness,
                                  external_points=[0],
                                  external_rest_shape=topCenterRestPosition.getMechanicalState().linkpath) # Spring on the top center of the cell
        all.addObject('SubsetMultiMapping', template="Vec3,Vec3",
                       input=[self.rigidified.getMechanicalState().linkpath,
                              self.deformable.getMechanicalState().linkpath],
                       output=all.getMechanicalState().linkpath,
                       indexPairs=[[1, 0],
                                   [0, 0], [0, 1], [0, 2], [0, 3], [0, 4], [0, 5], [0, 6]])
        
    def __addCollision(self):
        """
        Adds a collision model, one point on the center top of the cell.
        """
        collision = self.deformable.addChild("Collision")
        collision.addObject("MeshTopology", position=[self.positions[0]])
        collision.addObject("MechanicalObject", position=[self.positions[0]],
                            showObject=False, showObjectScale=0.002, drawMode=2)
        collision.addObject("PointCollisionModel")
        collision.addObject("BarycentricMapping", input_topology=collision.MeshTopology.linkpath)

        
def createScene(rootnode):

    from modules.header import addHeader, addSolvers

    settings, modelling, simulation = addHeader(rootnode, inverse=False, withCollision=False, friction=0)

    addSolvers(simulation, rayleighStiffness=0.001)
    rootnode.VisualStyle.displayFlags = ["showVisual"]

    for i in range(3):
        patch = simulation.addChild("Patch"+str(i))
        patch.addObject("MechanicalObject", template="Rigid3", position=[[[0, 0, 0.01, 0, 0, 0, 1],
                                                                          [0, -0.01, 0, 0, 0.707, 0, 0.707],
                                                                          [0, 0, 0, 0, 0, 0, 1]][i]],
                        showObject=True, showObjectScale=0.005)
        patch.addObject("FixedProjectiveConstraint", indices=[0])

        Cell(simulationNode=simulation, attachNode=patch, attachIndex=0, name="Patch"+str(i)+"Cell")
