# From: https://github.com/RobotLocomotion/drake/issues/12073
import sys

assert __name__ == "__main__"
torch_first = "--torch_first" in sys.argv

def repro():
    if torch_first:
        print("Torch first")
        import torch
        import cc_regex
    else:
        print("Torch last")
        import cc_regex
        import torch
    print(cc_regex.get_name())
    print("[ Done ]")

import sys, trace
sys.stdout = sys.stderr
tracer = trace.Trace(trace=1, count=0, ignoredirs=["/usr", sys.prefix])
tracer.runfunc(repro)
# repro()
exit(0)
