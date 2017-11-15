from __future__ import print_function
import sys
import time
from threading import Thread, Lock

from drake.common.proto.matlab_rpc_pb2 import MatlabArray, MatlabRPC

import numpy as np

class _KwArgs(dict):
    pass


# Helpers (to keep interface as simple as possible).
def _get_required_helpers(scope_locals):
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

    def set_var(var, value):
        scope_locals[var] = value

    return locals()

def _merge_dicts(*args):
    out = {}
    for arg in args:
        out.update(arg)
    return out

# Main functionalty.
def default_globals():
    # For plotting.
    import numpy as np
    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib
    import matplotlib.pyplot as plt
    import pylab  # See `%pylab?` in IPython.
    # Enable interactivity for default globals...
    # Where better to put this?
    matplotlib.interactive(True)

    def surf(x, y, Z, **kwargs):
        fig = plt.gcf()
        ax = fig.gca(projection='3d')
        X, Y = np.meshgrid(x, y)
        ax.plot_surface(X, Y, Z, **kwargs)


    def show():
        plt.show(block=False)


    def pause(interval):
        plt.pause(interval)


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

    # Use <module>.__dict__ to simulate `from <module> import *`, since that is invalid
    # in this scope.
    return _merge_dicts(
        globals(),
        plt.__dict__,
        pylab.__dict__,
        locals())


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
        # TODO(eric.cousineau): If running threaded, without looping, this will not correctly
        # determine the end condition for the client... Why?
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
    def __init__(self, filename = None, threaded = True, loop = False,
                 scope_globals = None, scope_locals = None):
        if filename is None:
            self.filename = "/tmp/matlab_rpc"
        else:
            self.filename = filename
        # Scope. Give it access to everything here.
        # However, keep it's written values scoped.
        if scope_locals is None:
            self.scope_locals = {}
        else:
            self.scope_locals = scope_locals
        required_helpers = _get_required_helpers(self.scope_locals)
        if scope_globals is None:
            scope_globals = default_globals()
        self.scope_globals = _merge_dicts(required_helpers, scope_globals)

        # def _client_var_del(id):
        #     # If a variable is created but not used, then it will not be registered.
        #     if id in client_vars:
        #         del client_vars[id]
        # scope_locals.update(_client_var_del=_client_var_del)

        # Variables indexed by GUID.
        self.client_vars = {}

        self.threaded = threaded
        self.loop = loop

        self.done = False

        self._file = None

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
        self.scope_locals.update(_tmp_out = out)
        # Update outputs.
        self.client_vars[out_id] = out

    def run(self):
        """ Run the client code.
        @note This must be run in the main thread if used for plotting.
        @note If you wish to maintain interactivity, enable the `threaded` option.
        """
        if self.threaded:
            self._handle_messages_async()
        else:
            self.handle_messages()

    def _handle_messages_async(self):
        # Main thread is consumer
        queue = []
        lock = Lock()
        def producer_loop():
            import copy
            for msg in self._generate_messages():
                msg_copy = copy.deepcopy(msg)
                with lock:
                    queue.append(msg_copy)
            self.done = True
        producer = Thread(target = producer_loop)
        producer.start()

        # Consume.
        # TODO(eric.cousineau): Trying to quit via Ctrl+C is awkward.
        try:
            pause = self.scope_globals['pause']
            while not self.done:
                with lock:
                    # Process all messages.
                    for msg in queue:
                        self._handle_message(msg)
                    del queue[:]
                # Spin busy for a bit.
                pause(0.001)
        except KeyboardInterrupt:
            print("Quitting")
            self.done = True

    def handle_messages(self, max_count=None, record=True, handle=True):
        """ Handle all messages sent (e.g., through IPython).
        @param max_count Maximum number of messages to handle.
        @param record Record all messages and return them.
        @param handle Play the given message.
        @return (count, msgs) where `count` is how many messages were processed (e.g. 0 if no more messages left)
        and `msgs` are either the messages themselves for playback.
        and (b) the messages themselves for playback (if record==True), otherwise an empty list. """
        count = 0
        msgs = []
        for msg in self._generate_messages():
            if handle:
                self._handle_message(msg)
            count += 1
            if record:
                msgs.append(msg)
            if max_count is not None and count >= max_count:
                break
        return (count, msgs)

    def play_messages(self, msgs):
        """ Play a set of recorded messages. """
        for msg in msgs:
            self._handle_message(msg)

    def _get_file(self):
        # TODO(eric.cousineau): For IPython, the file pointer may linger, and may cause C++
        # clients to *not* block on initial execution.
        # Consider a more explicit cleanup mechanism?
        if self._file is None:
            self._file = open(self.filename, 'rb')
        return self._file

    def _close_file(self):
        if self._file is not None:
            self._file.close()
            self._file = None

    def _generate_messages(self):
        # Return a new incoming message.
        # Not guaranteed to be a unique instance. Should copy if needed.
        msg = MatlabRPC()
        while not self.done:
            f = self._get_file()
            while _read_next(f, msg) and not self.done:
                yield msg
            # Close the file if we reach the end;
            # If we don't reach the end, keep the file open (e.g. if reading a few messages).
            self._close_file()
            if not self.loop:
                break


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--no_threading", action='store_true', help="Disable threaded dispatch.")
    parser.add_argument("--no_loop", action='store_true', help="Stop client after a C++ session closes.")
    parser.add_argument("-f", "--file", type=str, default=None)
    args = parser.parse_args(sys.argv[1:])

    client = CallPythonClient(args.file, loop = not args.no_loop, threaded = not args.no_threading)
    client.run()
