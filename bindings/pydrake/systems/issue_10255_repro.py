from pydrake.systems.framework import LeafSystem
from pydrake.systems.analysis import Simulator

class DemoSystem(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        self._DeclarePeriodicDiscreteUpdate(period_sec=1, offset_sec=0)
        self.num_updates = 0

    def _DoCalcDiscreteVariableUpdates(self, context, events, discrete_state):
        self.num_updates += 1
        # Call base method to ensure we do not get recursion.
        LeafSystem._DoCalcDiscreteVariableUpdates(
            self, context, events, discrete_state)

system = DemoSystem()
simulator = Simulator(system)
simulator.StepTo(0)
assert system.num_updates == 1, system.num_updates
