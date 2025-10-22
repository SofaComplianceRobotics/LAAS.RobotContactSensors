def addHeader(rootnode,
              inverse=False, multithreading=False,
              friction=0.6,
              withCollision=False, withConstraint=True):

    # Units are in m, kg, s
    # RequiredPlugins
    settings = rootnode.addChild('Settings')
    settings.addObject('RequiredPlugin', name="external",
                       pluginName=['BeamAdapter', 'SoftRobots', 'MultiThreading',
                                   'SoftRobots.Inverse', 'ArticulatedSystemPlugin'
                                   ])
    settings.addObject('RequiredPlugin', name="header",
                       pluginName=['Sofa.GL.Component.Shader',  # needed for LightManager
                                   'Sofa.Component.Collision.Detection.Algorithm',
                                   'Sofa.Component.Collision.Detection.Intersection',
                                   'Sofa.Component.AnimationLoop',
                                   'Sofa.Component.Collision.Response.Contact',
                                   # Needed to use components [RuleBasedContactManager]
                                   'Sofa.Component.Constraint.Lagrangian.Solver',
                                   # Needed to use components [GenericConstraintSolver]
                                   'Sofa.Component.StateContainer',  # Needed to use components [MechanicalObject]
                                   'Sofa.Component.Visual',  # Needed to use components [VisualStyle]
                                   'Sofa.GUI.Component',  # Needed to use components [AttachBodyButtonSetting]
                                   "Sofa.Component.IO.Mesh"
                                   ])

    # Utilities
    rootnode.addObject("DefaultVisualManagerLoop")
    rootnode.addObject('VisualStyle')
    rootnode.gravity = [0, 0, -9.81]
    rootnode.dt = 0.01

    modelling = rootnode.addChild('Modelling')
    modelling.addChild('Topology')
    simulation = rootnode.addChild('Simulation')

    # Collision header
    if withCollision:
        rootnode.addObject('CollisionPipeline')
        rootnode.addObject('RuleBasedContactManager', responseParams='mu=' + str(friction),
                           response='FrictionContactConstraint')
        rootnode.addObject('ParallelBruteForceBroadPhase')
        rootnode.addObject('ParallelBVHNarrowPhase')
        rootnode.addObject('LocalMinDistance', alarmDistance=0.005, contactDistance=0.001)

    # Constraint header
    if withConstraint:
        rootnode.addObject('FreeMotionAnimationLoop',
                           parallelODESolving=multithreading)
        if inverse:
            rootnode.addObject('QPInverseProblemSolver', name='ConstraintSolver', tolerance=1e-8, maxIterations=500,
                               multithreading=multithreading, responseFriction=friction, allowSliding=False, epsilon=0.01)
        else:
            rootnode.addObject('ProjectedGaussSeidelConstraintSolver', name='ConstraintSolver', tolerance=1e-8, maxIterations=500,
                               multithreading=multithreading)
    else:
        rootnode.addObject('DefaultAnimationLoop')

    return settings, modelling, simulation


def addSolvers(node, rayleighMass=0., rayleighStiffness=0.01, firstOrder=False,
               multithreading=False, iterativeSolver=False):

    # Required plugins
    settings = node.getRoot().Settings
    if settings.getObject("solvers") is None:
        settings.addObject('RequiredPlugin', name="solvers",
                           pluginName=['Sofa.Component.ODESolver.Backward',
                                       'Sofa.Component.LinearSolver.Direct',
                                       'Sofa.Component.Constraint.Lagrangian.Correction',
                                       'Sofa.Component.LinearSolver.Iterative'
                                       ])

    # Solvers
    node.addObject('EulerImplicitSolver', firstOrder=firstOrder, rayleighStiffness=rayleighStiffness,
                   rayleighMass=rayleighMass, printLog=False)
    if multithreading:
        node.addObject('RequiredPlugin', name='SofaCUDASolvers')
        node.addObject('CudaSparseLDLSolver', name='solver', template='AsyncCompressedRowSparseMatrixMat3x3f',
                       useMultiThread=True)
    else:
        if iterativeSolver:
            node.addObject('CGLinearSolver', name='solver', iterations=500, tolerance=1e-10, threshold=1e-10)
        else:
            node.addObject('SparseLDLSolver', name='solver', template="CompressedRowSparseMatrixd")

    if not iterativeSolver:
        node.addObject('GenericConstraintCorrection', linearSolver=node.solver.getLinkPath())


# Test
def createScene(rootnode):
    addHeader(rootnode)
