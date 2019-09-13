from pydrake.solvers.solver_interface import SolverInterface

import unittest

class DummySolverInterface(SolverInterface):
    def __init__(self):
        pass

    def available(self):
        return True

    def solver_id(self):
        return 0

    def Solve(prog, initial_guess, solver_options):
        pass

    def AreProgramAttributesSatisfied(self, prog):
        return True


class DummySolverInterfaceTest(unittest.TestCase):
    def test_dummy_solver_interface(self):
        solver = DummySolverInterface()
        self.assertTrue(solver.available())
