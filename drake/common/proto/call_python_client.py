from __future__ import print_function
import sys
import time
from threading import Thread, Lock

from drake.common.proto.matlab_rpc_pb2 import MatlabArray, MatlabRPC

# For plotting.
from mpl_toolkits.mplot3d import Axes3D
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.pyplot import *
from pylab import *  # See `%pylab?` in IPython.

# Helpers (to keep interface as simple as possible).
def setitem(obj, index, value):
    obj[index] = value
    return obj[index]


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


def make_kwargs(*args):
    assert len(args) % 2 == 0
    keys = args[0::2]
    values = args[1::2]
    kwargs = dict(zip(keys, values))
    return _KwArgs(**kwargs)


def make_slice(expr):
    def to_piece(s):
        return s and int(s) or None
    pieces = map(to_piece, expr.split(':'))
    if len(pieces) == 1:
        return slice(pieces[0], pieces[0] + 1)
    else:
        return slice(*pieces)


def surf(x, y, Z, **kwargs):
    fig = plt.gcf()
    ax = fig.gca(projection='3d')
    X, Y = np.meshgrid(x, y)
    ax.plot_surface(X, Y, Z, **kwargs)


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


class CallPythonClient(object):
    def __init__(self, filename, threaded = True):
        self.filename = filename
        # Scope. Give it access to everything here.
        # However, keep it's written values scoped.
        self.scope_globals = globals()
        self.scope_locals = {}

        # def _client_var_del(id):
        #     # If a variable is created but not used, then it will not be registered.
        #     if id in client_vars:
        #         del client_vars[id]
        # scope_locals.update(_client_var_del=_client_var_del)

        # Variables indexed by GUID.
        self.client_vars = {}

        self.threaded = threaded

    def _to_array(self, arg, dtype):
        np_raw = np.frombuffer(arg.data, dtype=dtype)
        if arg.shape_type == MatlabArray.SCALAR:
            assert arg.cols == 1 and arg.rows == 1
            return np_raw[0]
        elif arg.shape_type == MatlabArray.VECTOR:
            assert arg.cols == 1
            return np_raw.reshape(arg.rows)
        elif arg.shape_type is None or arg.shape_type == MatlabArray.MATRIX:
            return np_raw.reshape(arg.rows, arg.cols)

    def _handle_message(self, msg):
        # # Handle special-cases without overhead.
        # if msg.function_name == "_client_var_del":
        #     _client_var_del(msg)
        #     continue

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
                if id not in self.client_vars:
                    raise RuntimeError("Unknown local variable. Dropping message.")
                value = self.client_vars[id]
            elif arg.type == MatlabArray.DOUBLE:
                value = self._to_array(arg, np.double)
            elif arg.type == MatlabArray.CHAR:
                assert arg.rows == 1
                value = str(arg_raw)
            elif arg.type == MatlabArray.LOGICAL:
                value = self._to_array(arg, np.bool)
            elif arg.type == MatlabArray.INT:
                value = self._to_array(arg, np.int32)
            else:
                assert False
            if isinstance(value, _KwArgs):
                assert kwargs is None
                kwargs = value
            else:
                inputs.append(value)

        # Call the function
        # N.B. No security measures to sanitize function name.
        function_name = msg.function_name
        out_id = None
        if len(msg.lhs) > 0:
            assert len(msg.lhs) == 1
            out_id = msg.lhs[0]

        self.scope_locals.update(_tmp_args = inputs, _tmp_kwargs = kwargs or {})
        out = eval(function_name + "(*_tmp_args, **_tmp_kwargs)", self.scope_globals, self.scope_locals)
        # Update outputs.
        self.client_vars[out_id] = out

    def run(self):
        if self.threaded:
            # Main thread is consumer
            queue = []
            lock = Lock()
            def producer_loop():
                import copy
                for msg in self._generate_messages():
                    msg_copy = copy.deepcopy(msg)
                    with lock:
                        queue.append(msg_copy)
            producer = Thread(target = producer_loop)
            producer.start()

            # Consume.
            while True:
                with lock:
                    # Process all messages.
                    for msg in queue:
                        self._handle_message(msg)
                    del queue[:]
                # Spin busy for a bit.
                plt.pause(0.001)
        else:
            for msg in self._generate_messages():
                self._handle_message(msg)

    def _generate_messages(self):
        # Return a new incoming message.
        # Not guaranteed to be a unique instance. Should copy if needed.
        msg = MatlabRPC()
        with open(self.filename, 'rb') as f:
            while _read_next(f, msg):
                yield msg
 

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--wait", action='store_true', help="Use to interact with plots.")
    # TODO: This does not work at present. Need to ensure that plotting happens in background thread.
    parser.add_argument("--loop", action='store_true', help="Poll for commands, even after a C++ session closes. (Use to interact with plots.)")
    parser.add_argument("-f", "--file", type=str, default="/tmp/matlab_rpc")
    args = parser.parse_args(sys.argv[1:])

    print(matplotlib.get_backend())
    matplotlib.interactive(True)
    assert matplotlib.is_interactive()

    client = CallPythonClient(args.file)
    client.run()
    if args.loop:
        # raise RuntimeError("Will not function as expected")
        while True:
            client.run()
    if args.wait:
        print("waiting...")
        # Block.
        plt.show(block=True)
