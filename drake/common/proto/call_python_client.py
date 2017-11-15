from __future__ import print_function
import sys

from drake.common.proto.matlab_rpc_pb2 import MatlabArray, MatlabRPC

# For plotting.
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib.pyplot import *
from pylab import *  # See `%pylab?` in IPython.

# Helpers (to keep interface as simple as possible).
def setitem(obj, index, value):
    print(obj)
    print(index)
    print(value)
    obj[index] = value
    return obj[index]


def getitem(obj, index):
    print(obj)
    print(index)
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
    from google.protobuf.internal.decoder import _DecodeVarint32

    # Blech. Should just not use this approach...
    # Consider gRPC? Or just use pybind11 directly?
    # Assume that each write will have at least 4-bytes (including the header size bit).
    peek_size = 4
    peek = f.read(peek_size)
    if len(peek) == 0:
        # We have reached the end.
        return 0
    msg_size, peek_end = _DecodeVarint32(peek, 0)
    peek_left = peek_size - peek_end
    # Read remaining and concatenate.
    remaining = f.read(msg_size - peek_left)
    msg_raw = peek[peek_end:] + remaining
    assert len(msg_raw) == msg_size
    # Now read the message.
    msg.ParseFromString(msg_raw)
    return msg_size


def run(filename):
    # Scope. Give it access to everything here.
    # However, keep it's written values scoped.
    scope_globals = globals()
    scope_locals = {}

    # Variables indexed by GUID.
    client_vars = {}

    def _client_var_del(id):
        # If a variable is created but not used, then it will not be registered.
        if id in client_vars:
            del client_vars[id]
    scope_locals.update(_client_var_del=_client_var_del)

    def get_dim(arg):
        if arg.is_vector:
            assert arg.cols == 1
            return (arg.rows,)
        else:
            return (arg.rows, arg.cols)

    print("[ Start ]")

    msg = MatlabRPC()
    with open(filename, 'rb') as f:
        while _read_next(f, msg):
            # # Handle special-cases without overhead.
            # if msg.function_name == "_client_var_del":
            #     _client_var_del(msg)
            #     continue

            print(msg)

            # Create input arguments.
            args = msg.rhs
            nargs = len(args)
            inputs = []
            kwargs = None
            for i, arg in enumerate(args):
                arg_raw = arg.data
                value = None
                if arg.type == MatlabArray.REMOTE_VARIABLE_REFERENCE:
                    id = np.frombuffer(arg_raw, dtype=np.uint64).reshape(1)[0]
                    if id not in client_vars:
                        raise RuntimeError("Unknown local variable. Dropping message.")
                    value = client_vars[id]
                elif arg.type == MatlabArray.DOUBLE:
                    dim = get_dim(arg)
                    value = np.frombuffer(arg_raw, dtype=np.double).reshape(dim)
                elif arg.type == MatlabArray.CHAR:
                    assert arg.rows == 1
                    value = str(arg_raw)
                elif arg.type == MatlabArray.LOGICAL:
                    dim = get_dim(arg)
                    value = np.frombuffer(arg_raw, dtype=np.bool).reshape(dim)
                elif arg.type == MatlabArray.INT:
                    dim = get_dim(arg)
                    value = np.frombuffer(arg_raw, dtype=np.int32).reshape(dim)
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
            if len(msg.lhs) > 0:
                assert len(msg.lhs) == 1
                out_id = msg.lhs[0]
                client_vars[out_id] = out

    print("[ Done ]")


if __name__ == "__main__":
    filename = "/tmp/matlab_rpc"
    if len(sys.argv) == 2:
        filename = sys.argv[1]
    elif len(sys.argv) > 2:
        raise RuntimeError("usage: call_python_client.py [FILENAME]")

    run(filename)
