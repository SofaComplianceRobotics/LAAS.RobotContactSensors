import Sofa
from math import pi, cos, sin, floor
import numpy as np

class Cell(Sofa.Prefab):
    """
        Modeling of a cell sensor. Units are m, kg
    """
    sideSize: float=0.01
    centerThickness: float=0.002
    stiffness: float=1e2
    totalMass: float=0.001

    def __init__(self, 
                 simulationNode: Sofa.Core.Node,
                 attachNode: Sofa.Core.Node,
                 name: str="Cell",
                 attachIndex: int=0,
                 addToCell=None
                 ):
        Sofa.Prefab.__init__(self)

        self.name = name
        self.simulationNode = simulationNode
        self.attachNode = attachNode
        self.attachIndex = attachIndex

        self.colorActive = [0, 1, 0, 1]
        self.colorInactive = [1, 1, 1, 1]
        self.drawMode = 1
        self.drawScale = 0.002

        assert simulationNode.getRoot().getChild("Settings") is not None
        self.settings = simulationNode.getRoot().Settings
        if self.settings.getChild("Cell") is None:
            self.settings.addChild("Cell")
            self.__addSettings()

        self.positions = self.__addTopology()
        if addToCell is None:
            self.__addMechanical()
            self.__addVisual()
            self.__addCollision()
        else:
            self.__addToCell(addToCell)

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
        o = [0., 0., 0.]

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
        self.all = all
        all.addObject("MeshTopology", position=self.positions, edges=self.edges, tetras=self.tetras)
        all.addObject("MechanicalObject", position=self.positions)
        all.addObject("UniformMass", totalMass=self.totalMass)

        self.rigidified = self.attachNode.addChild(self.name.value + "RigidPart")
        self.rigidified.addObject("MechanicalObject", position=[self.positions[1:]])
        self.rigidified.addObject("RigidMapping", rigidIndexPerPoint=[self.attachIndex]*7, globalToLocalCoords=False)
        self.rigidified.addChild(all)

        topCenterRestPosition = self.attachNode.addChild(self.name.value + "TopCenterRestPosition")
        topCenterRestPosition.addObject("MechanicalObject", position=[self.positions[0]],
                                        showObject=True, showObjectScale=self.drawScale, drawMode=self.drawMode, showColor=self.colorActive)
        topCenterRestPosition.addObject("RigidMapping", index=self.attachIndex, globalToLocalCoords=False)
        topCenterRestPosition.init()
        
        self.deformable = self.simulationNode.addChild(self.name.value + "DeformablePart")
        self.deformable.addObject("MechanicalObject", position=topCenterRestPosition.getMechanicalState().position.value,
                                    showObject=True, showObjectScale=self.drawScale*1.1, drawMode=self.drawMode, showColor=self.colorInactive)
        self.deformable.addChild(all)

        self.deformable.addObject("VisualStyle", displayFlags=["showBehavior"])
        self.deformable.addObject("RestShapeSpringsForceField", name="rsff0",
                                  points=[0], stiffness=self.stiffness,
                                  external_points=[0],
                                  external_rest_shape=topCenterRestPosition.getMechanicalState().linkpath) # Spring on the top center of the cell
        all.addObject('SubsetMultiMapping', template="Vec3,Vec3",
                       input=[self.rigidified.getMechanicalState().linkpath,
                              self.deformable.getMechanicalState().linkpath],
                       output=all.getMechanicalState().linkpath,
                       indexPairs=[[1, 0],
                                   [0, 0], [0, 1], [0, 2], [0, 3], [0, 4], [0, 5], [0, 6]])
        
    def __addToCell(self, cell):

        all = cell.all
        offset = len(all.getMechanicalState().position.value)
        offsetD = floor(offset / 8)
        offsetR = offset - offsetD

        offset = len(all.getMechanicalState().position.value)
        for e in self.edges:
            for i in range(2):
                e[i] += offset
        for t in self.tetras:
            for i in range(4):
                t[i] += offset

        all.MeshTopology.position.value = np.append(all.MeshTopology.position.value, self.positions, axis=0)
        all.MeshTopology.edges.value = list(np.append(all.MeshTopology.edges.value, self.edges, axis=0))
        all.MeshTopology.tetras.value = list(np.append(all.MeshTopology.tetras.value, self.tetras, axis=0))
        all.getMechanicalState().position.value = np.append(all.getMechanicalState().position.value, 
                                                            self.positions, axis=0)

        cell.rigidified.getMechanicalState().position.value = np.append(cell.rigidified.getMechanicalState().position.value, 
                                                                        self.positions[1:], axis=0)
        rigidIndexPerPoint = np.append(cell.rigidified.RigidMapping.rigidIndexPerPoint.value,
                                       [self.attachIndex]*7)
        cell.rigidified.RigidMapping.rigidIndexPerPoint.value = list(rigidIndexPerPoint)

        topCenterRestPosition = self.attachNode.addChild(self.name.value + "TopCenterRestPosition" + str(offsetR))
        topCenterRestPosition.addObject("MechanicalObject", position=[self.positions[0]],
                                        showObject=True, showObjectScale=self.drawScale, drawMode=self.drawMode, showColor=self.colorActive)
        topCenterRestPosition.addObject("RigidMapping", index=self.attachIndex, globalToLocalCoords=False)
        topCenterRestPosition.init()

        cell.deformable.getMechanicalState().position.value = np.append(cell.deformable.getMechanicalState().position.value,
                                                                        topCenterRestPosition.getMechanicalState().position.value, axis=0)

        cell.deformable.addObject("RestShapeSpringsForceField", 
                                  name = "rsff" + str(offsetD),
                                  points=[offsetD], stiffness=self.stiffness,
                                  external_points=[0],
                                  external_rest_shape=topCenterRestPosition.getMechanicalState().linkpath) # Spring on the top center of the cell
        
        indexPairs = list(all.SubsetMultiMapping.indexPairs.value) + [1, offsetD,
                                                                      0, offsetR, 0, offsetR+1, 0, offsetR+2, 0, offsetR+3, 0, offsetR+4, 0, offsetR+5, 0, offsetR+6]
    
        all.SubsetMultiMapping.indexPairs.value = indexPairs
        
    def __addVisual(self):
        """
        Adds a visual model
        """
        visual = self.all.addChild("VisualWhite")
        visual.addObject("OglModel", src=self.all.MeshTopology.linkpath, color=[1, 1, 1, 0.5])
        visual.addObject("IdentityMapping")

    def __addCollision(self):
        """
        Adds a collision model, one point on the center top of the cell.
        """
        self.deformable.addObject("PointCollisionModel")
        
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
