#!/usr/bin/env python

from call_python_client import CallPythonClient

client = CallPythonClient()
count = client.handle_messages(5)
print("Handled: {}".format(count))

count = client.handle_messages(5)
print("Handled: {}".format(count))
