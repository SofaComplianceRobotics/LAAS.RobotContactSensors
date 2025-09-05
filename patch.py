import Sofa 
from cell import Cell
import numpy as np
from math import sqrt

class Patch(Sofa.Prefab):

    def __init__(self,
                 name: str,
                 simulationNode: Sofa.Core.Node,
                 attachNode: Sofa.Core.Node,
                 attachIndex: int=0,
                 cellGrid: tuple[int]=[1, 1],
                 origin: list[float]=[0, 0, 0, 0, 0, 0, 1]):
        
        Sofa.Prefab.__init__(self)

        self.name = name
        self.simulationNode = simulationNode
        self.attachNode = attachNode
        self.attachIndex = attachIndex
        self.cellGrid = cellGrid
        self.origin = origin

        self.attachNode.addChild(self)

        self.__addMechanical()
        self.__addCells()

    def __addMechanical(self):
        positions = []
        stepx = Cell.sideSize * 3
        stepy = Cell.sideSize * sqrt(3)/2
        for i in range(self.cellGrid[0]):
            for j in range(self.cellGrid[1]):
                p = list(self.origin)
                p[0] += stepx * i + 1.5 * Cell.sideSize * (j % 2) 
                p[1] += stepy * j
                positions.append(p)

        self.addObject("MechanicalObject", template="Rigid3", position=positions,
                       showObject=False, showObjectScale=0.01, drawMode=2)
        self.addObject("RigidMapping", index=self.attachIndex, globalToLocalCoords=True)

    def __addCells(self):
        for i in range(self.cellGrid[0]):
            for j in range(self.cellGrid[1]):
                cell = Cell(name=self.name.value + "Cell"+str(i)+str(j),
                            simulationNode=self.simulationNode,
                            attachNode=self,
                            attachIndex=i*self.cellGrid[1]+j,
                            )

    

def createScene(rootnode):

    from modules.header import addHeader, addSolvers

    settings, modelling, simulation = addHeader(rootnode, inverse=False, withCollision=False, friction=0)

    addSolvers(simulation, rayleighStiffness=0.001)
    rootnode.VisualStyle.displayFlags = ["showVisual"]

    robot = simulation.addChild("Robot")
    robot.addObject("MechanicalObject", template="Rigid3", position=[[0, 0, 0, 0, 0, 0, 1]],
                    showObject=True, showObjectScale=0.01, drawMode=2)
    robot.addObject("FixedProjectiveConstraint", indices=[0])

    Patch(simulationNode=simulation, attachNode=robot, attachIndex=0, name="Patch1", cellGrid=[5, 5], origin=[0, 0.1, 0, 0, 0.707, 0, 0.707])
    Patch(simulationNode=simulation, attachNode=robot, attachIndex=0, name="Patch2", cellGrid=[5, 5], origin=[0, 0, 0, 0, 0, 0, 1])