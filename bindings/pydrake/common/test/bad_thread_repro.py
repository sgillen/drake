from functools import partial
from threading import Thread

from pydrake.common import hack_callback


def traced(f):
    import sys, trace
    sys.stdout = sys.stderr
    tracer = trace.Trace(trace=1, count=0, ignoredirs=["/usr", sys.prefix])
    return partial(tracer.runfunc, f)

was_called = False

def my_func():
    global was_called
    print("CALLED")
    was_called = True

def stuff():
    hack_callback(my_func)
    assert was_called == True

def main():
    t = Thread(target=traced(stuff))
    t.start()
    t.join()

assert __name__ == "__main__"
traced(main)()
