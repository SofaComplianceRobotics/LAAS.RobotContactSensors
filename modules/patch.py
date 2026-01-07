import Sofa 
from cell import Cell
import numpy as np
from math import sqrt, pi, cos, sin
from splib3.numerics import Quat, Vec3

class Patch(Sofa.Prefab):
    """
    A Patch is a collection of cells arranged in a grid or at specified positions.
    """

    def __init__(self,
                 name: str,
                 simulationNode: Sofa.Core.Node,
                 attachNode: Sofa.Core.Node, # the node to which the patch is attached (the robot)
                 attachIndex: int=0, # the index of the joint where the patch is attached
                 cellsGrid: tuple[int]=[1, 1], # option1: number of cells in (x,y) directions
                 cellsPositions: list[list[float]] = None, # option2: list of cell positions and orientations
                 origin: list[float]=[0, 0, 0, 0, 0, 0, 1]): # origin position and orientation of the patch
        
        Sofa.Prefab.__init__(self)

        self.name = name
        self.simulationNode = simulationNode
        self.attachNode = attachNode
        self.attachIndex = attachIndex
        self.cellsGrid = cellsGrid
        self.cellsPositions = cellsPositions
        self.origin = origin

        self.attachNode.addChild(self)

        # Add mechanical object and cells
        self.__addMechanical()
        self.__addCells()

    def __addMechanical(self):
        """
        Add the MechanicalObject and RigidMapping to the patch.
        """
        positions = self.cellsPositions
        if positions is None: # In case no positions are provided, create a grid
            positions = []
            stepx = Cell.sideSize * 3
            stepy = Cell.sideSize * sqrt(3)/2
            for i in range(self.cellsGrid[0]):
                for j in range(self.cellsGrid[1]):
                    x = stepx * i + 1.5 * Cell.sideSize * (j % 2) 
                    y = stepy * j
                    z = 0
                    q = Quat(self.origin[3:7])
                    v = Vec3([x, y, z])
                    v = v.rotateFromQuat(q)
                    for k in range(3):
                        v[k] += self.origin[k]
                    positions.append(list(v)+list(q))
        else: # Else, adjust positions based on origin
            for i in range(len(positions)):
                v = Vec3(positions[i][0:3])
                for k in range(3):
                    v[k] += self.origin[k]
                positions[i][0:3] = list(v)

        # This mechanical object holds the positions of the cells
        self.addObject("MechanicalObject", template="Rigid3", position=positions,
                       showObject=False, showObjectScale=0.01, drawMode=2)
        # This mapping connects the patch to the robot joint
        self.addObject("RigidMapping", index=self.attachIndex, globalToLocalCoords=True)

    def __addCells(self):
        """
        Add cells to the patch based on the provided positions or grid.
        """
        cell = None
        if self.cellsPositions is not None:
            for index in range(len(self.cellsPositions)):
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
            return
        
        for i in range(self.cellsGrid[0]):
            for j in range(self.cellsGrid[1]):
                index = i*self.cellsGrid[1] + j
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
                    

# Example of scene using Patch
def createScene(rootnode):

    # Scene setup. Add header (solvers, visual style, gravity, time step, etc.)
    from modules.header import addHeader, addSolvers
    settings, modelling, simulation = addHeader(rootnode, inverse=False, withCollision=False, friction=0)
    addSolvers(simulation, rayleighStiffness=0.001)
    rootnode.VisualStyle.displayFlags = ["showVisual"]

    # We add a frame representing the robot
    robot = simulation.addChild("Robot")
    robot.addObject("MechanicalObject", template="Rigid3", position=[[0, 0, 0, 0, 0, 0, 1]])
    robot.addObject("FixedProjectiveConstraint", indices=[0]) # Fix the robot in space

    # We add three patches at different locations on the robot
    patch = Patch(simulationNode=simulation, attachNode=robot, attachIndex=0, name="Patch1", cellsGrid=[5, 4], 
                  origin=[0., 0.1, 0., 0., 0.707, 0., 0.707])
    patch.getMechanicalState().showObject=False
    patch = Patch(simulationNode=simulation, attachNode=robot, attachIndex=0, name="Patch2", cellsGrid=[5, 5], 
                  origin=[0., 0., 0., cos(pi/6), 0., 0., sin(pi/6)])
    patch.getMechanicalState().showObject=False
    patch = Patch(simulationNode=simulation, attachNode=robot, attachIndex=0, name="Patch3", cellsGrid=[2, 5], 
                  origin=[-0.2, 0.1, 0., 0., 0., 0., 1])
    patch.getMechanicalState().showObject=False