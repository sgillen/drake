import unittest

import numpy as np

import numpy_dtypes_user as mut


class TestNumpyDtypesUser(unittest.TestCase):
    def test_scalar_meta(self):
        """Tests basic metadata."""
        self.assertIsSubClass(mut.Symbol, np.generic)
        self.assertIsInstance(np.dtype(mut.Symbol), np.dtype)
