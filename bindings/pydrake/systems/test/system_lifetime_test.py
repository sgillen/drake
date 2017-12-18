#!/usr/bin/env python
# -*- coding: utf8 -*-

"""
@file
Captures limitations for the present state of the Python bindings for the
lifetime of objects, eventually lock down capabilities as they are introduced.
"""

from __future__ import print_function

import unittest
import numpy as np

from pydrake.systems.analysis import (
    Simulator,
    )
from pydrake.systems.framework import (
    DiagramBuilder,
    )
from pydrake.systems.test.system_lifetime_test_util import (
    DeleteListenerSystem,
    )


class Info(object):
    def __init__(self):
        self.deleted = False

    def record_deletion(self):
        assert not self.deleted
        self.deleted = True


def _create_system():
    info = Info()
    system = DeleteListenerSystem(info.record_deletion)
    return system, info


class TestSystemLifetime(unittest.TestCase):
    def test_basic(self):
        system, info = _create_system()
        self.assertFalse(info.deleted)
        del system
        self.assertTrue(info.deleted)

    def test_ownership_diagram(self):
        system, info = _create_system()
        builder = DiagramBuilder()
        # `system` is now owned by `builder`.
        builder.AddSystem(system)
        # `system` is now owned by `diagram`.
        diagram = builder.Build()
        # Delete the builder. Should still be alive.
        del builder
        self.assertFalse(info.deleted)
        # Delete the diagram. Should be dead.
        del diagram
        # WARNING
        self.assertTrue(info.deleted)
        self.assertTrue(system is not None)

    def test_ownership_simulator(self):
        system, info = _create_system()
        simulator = Simulator(system)
        self.assertFalse(info.deleted)
        del simulator
        # Simulator does not own the system.
        self.assertFalse(info.deleted)
        self.assertTrue(system is not None)


if __name__ == '__main__':
    unittest.main()
