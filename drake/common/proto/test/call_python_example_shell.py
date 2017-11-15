#!/usr/bin/env python

from call_python_client import CallPythonClient

client = CallPythonClient()
count, msgs = client.handle_messages(5)
print("Handled: {}".format(count))

count, msgs = client.handle_messages(5, execute=False)
print("Handled: {}".format(count))

# Execute messages.
client.execute_messages(msgs)
