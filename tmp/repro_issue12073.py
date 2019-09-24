# From: https://github.com/RobotLocomotion/drake/issues/12073
import sys

assert __name__ == "__main__"
torch_first = "--torch_first" in sys.argv

def repro():
    if torch_first:
        print("Torch first")
        import torch
        import nice_type_name
    else:
        print("Torch last")
        import nice_type_name
        import torch
    print(nice_type_name.ArbitraryName().name())
    print("[ Done ]")

import sys, trace
sys.stdout = sys.stderr
tracer = trace.Trace(trace=1, count=0, ignoredirs=["/usr", sys.prefix])
tracer.runfunc(repro)
# repro()
exit(0)
