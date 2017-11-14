import sys

# For accessing "print" as a function.
from __future__ import print_function

from drake.common.proto.matlab_rpc_pb2 import MatlabArray, MatlabRPC

# For plotting.
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib.pyplot import *
from pylab import *  # See `%pylab?` in IPython.

# Helpers (to keep interface as simple as possible).
def setitem(obj, index, value):
    return obj[index] = value


def getitem(obj, index):
    return obj[index]


def pass_through(value):
    return value


def disp(value):
    print(value)


def make_tuple(*args):
    return tuple(args)


def make_list(*args):
    return list(args)


class _KwArgs(dict):
    pass

def make_kwargs(**kwargs):
    return _KwArgs(**kwargs)


def make_slice(expr):
    def to_piece(s):
        return s and int(s) or None
    pieces = map(to_piece, expr.split(':'))
    if len(pieces) == 1:
        return slice(pieces[0], pieces[0] + 1)
    else:
        return slice(*pieces)


def surf(*args, **kwargs):
    fig = figure()
    ax = fig.gca(project='3d')
    ax.plot_surface(*args, **kwargs)


def show():
    plt.show(block=False)


def magic(N):
    # Simple odd-only case for magic squares.
    # From: https://scipython.com/book/chapter-6-numpy/examples/creating-a-magic-square
    assert N % 2 == 1
    magic_square = np.zeros((N,N), dtype=int)
    n = 1
    i, j = 0, N//2
    while n <= N**2:
        magic_square[i, j] = n
        n += 1
        newi, newj = (i-1) % N, (j+1)% N
        if magic_square[newi, newj]:
            i += 1
        else:
            i, j = newi, newj
    return magic_square


# Main functionalty.

def _read_next(f, msg):
    # Hacky, but this is the simpliest route right now.
    # @ref https://www.datadoghq.com/blog/engineering/protobuf-parsing-in-python/
    from google.protobuf.internal.encoder import _DecodeVarint32

    # Blech. Should just not use this approach...
    # Consider gRPC? Or just use pybind11 directly?
    # Assume that each write will have at least 4-bytes (including the header size bit).
    start_size_pre = 4
    head = f.read(start_size_pre)
    msg_size, start_size_actual = _DecodeVarint32(head, 0)
    # Read remaining and concatenate.
    full_size = msg_size + start_size
    remaining = f.read(full_size - start_size_actual)
    msg_raw = head[start_size_actual:] + remaining
    assert len(msg_raw) == msg_size
    # Now read the message.
    msg.ReadFromString(msg_raw)
    return msg_size


def run(filename):
    # Scope. Give it access to everything here.
    # However, keep it's written values scoped.
    scope_globals = globals()
    scope_locals = {}

    # Variables indexed by GUID.
    client_vars = {}

    print("[ Start ]")

    msg = MatlabRPC()
    with open(filename, 'rb') as f:
        while _read_next(f, msg):
            # Create input arguments.
            nargs = len(msg.rhs)
            inputs = []
            kwargs = None
            for i, arg in enumerate(msg.rhs):
                arg_raw = arg.data
                value = None
                if rhs.type == MatlabArray.REMOTE_VARIABLE_REFERENCE:
                    id = np.frombuffer(arg_raw, dtype=np.uint64).reshape(1)[0]
                    if id not in client_vars:
                        raise RuntimeError("Unknown local variable. Dropping message.")
                    value = client_vars[id]
                elif rhs.type == MatlabArray.DOUBLE:
                    dim = (rhs.rows(), rhs.cols())
                    value = np.frombuffer(arg_raw, dtype=np.double).reshape(dim)
                elif rhs.type == MatlabArray.CHAR:
                    assert rhs.rows() == 1
                    value = str(arg_raw)
                elif rhs.type == MatlabArray.LOGICAL:
                    dim = (rhs.rows(), rhs.cols())
                    value = np.frombuffer(arg_raw, dtype=np.bool).reshape(dim)
                else:
                    assert False
                if isinstance(value, _KwArgs):
                    assert kwargs is None
                    kwargs = value
                else:
                    inputs.append(value)

            # Call the function
            # N.B. No security measures to sanitize function name.
            scope_locals.update(_tmp_args = inputs, _tmp_kwargs = kwargs or {})
            out = eval(msg.function_name + "(*_tmp_args, **_tmp_kwargs)", scope_globals, scope_locals)

            # Update outputs.
            assert len(msg.lhs) == 1
            out_id = msg.lhs[0]
            client_vars[out_id] = out

    print("[ Done ]")


if __name__ == "__main__":
    filename = "/tmp/matlab_rpc"
    if len(sys.argv) == 1:
        filename = args[0]
    elif len(sys.argv) > 1:
        raise RuntimeError("usage: call_python_client.py [FILENAME]")

    run(filename)
