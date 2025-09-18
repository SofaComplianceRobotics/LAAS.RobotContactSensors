import Sofa 
from .cell import Cell
import numpy as np
from math import sqrt, pi, cos, sin
from splib3.numerics import Quat, Vec3

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
                x = stepx * i + 1.5 * Cell.sideSize * (j % 2) 
                y = stepy * j
                z = 0
                q = Quat(self.origin[3:7])
                v = Vec3([x, y, z])
                v = v.rotateFromQuat(q)
                for k in range(3):
                    v[k] += self.origin[k]
                positions.append(list(v)+list(q))

        self.addObject("MechanicalObject", template="Rigid3", position=positions,
                       showObject=False, showObjectScale=0.01, drawMode=2)
        self.addObject("RigidMapping", index=self.attachIndex, globalToLocalCoords=True)

    def __addCells(self):
        cell = None
        for i in range(self.cellGrid[0]):
            for j in range(self.cellGrid[1]):
                index = i*self.cellGrid[1] + j
                if cell is None:
                    cell = Cell(name=self.name.value + "Cells",
                                simulationNode=self.simulationNode,
                                attachNode=self,
                                attachIndex=index,
                                )
                else:
                    Cell(simulationNode=self.simulationNode,
                         attachNode=self,
                         attachIndex=index,
                         addToCell=cell)
                    

def createScene(rootnode):

    from modules.header import addHeader, addSolvers

    settings, modelling, simulation = addHeader(rootnode, inverse=False, withCollision=False, friction=0)

    addSolvers(simulation, rayleighStiffness=0.001)
    rootnode.VisualStyle.displayFlags = ["showVisual"]

    robot = simulation.addChild("Robot")
    robot.addObject("MechanicalObject", template="Rigid3", position=[[0, 0, 0, 0, 0, 0, 1]])
    robot.addObject("FixedProjectiveConstraint", indices=[0])

    patch = Patch(simulationNode=simulation, attachNode=robot, attachIndex=0, name="Patch1", cellGrid=[5, 4], 
                  origin=[0., 0.1, 0., 0., 0.707, 0., 0.707])
    patch.getMechanicalState().showObject=False
    patch = Patch(simulationNode=simulation, attachNode=robot, attachIndex=0, name="Patch2", cellGrid=[5, 5], 
                  origin=[0., 0., 0., cos(pi/6), 0., 0., sin(pi/6)])
    patch.getMechanicalState().showObject=False
    patch = Patch(simulationNode=simulation, attachNode=robot, attachIndex=0, name="Patch3", cellGrid=[2, 5], 
                  origin=[-0.2, 0.1, 0., 0., 0., 0., 1])
    patch.getMechanicalState().showObject=False