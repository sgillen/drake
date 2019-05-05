import pydrake.common.eigen_geometry as mut

import copy
import unittest

import numpy as np
import six

from pydrake.autodiffutils import AutoDiffXd
from pydrake.symbolic import Expression
import pydrake.common.test.eigen_geometry_test_util as test_util


def normalize(x):
    return x / np.linalg.norm(x)


@np.vectorize
def to_value(x):
    if isinstance(x, float) or isinstance(x, int):
        return x
    elif isinstance(x, AutoDiffXd):
        return x.value()
    elif isinstance(x, Expression):
        return x.Evaluate()
    assert False


def allclose(a, b, **kwargs):
    return np.allclose(to_value(a), to_value(b), **kwargs)


class TestEigenGeometry(unittest.TestCase):
    def check_types(self, check_func):
        print(check_func)
        check_func(float)
        check_func(AutoDiffXd)
        check_func(Expression)

    def test_quaternion(self):
        self.check_types(self.check_quaternion)

    def check_quaternion(self, T):
        # Simple API.
        Quaternion = mut.Quaternion_[T]
        cast = np.vectorize(T)
        q_identity = Quaternion()
        self.assertTrue(allclose(q_identity.wxyz(), [1, 0, 0, 0]))
        self.assertTrue(allclose(
            copy.copy(q_identity).wxyz(), [1, 0, 0, 0]))
        self.assertTrue(allclose(
            q_identity.wxyz(), Quaternion.Identity().wxyz()))
        if T == float:
            self.assertEqual(
                str(q_identity), "Quaternion_[float](w=1.0, x=0.0, y=0.0, z=0.0)")
        # Test ordering.
        q_wxyz = normalize([0.1, 0.3, 0.7, 0.9])
        q = Quaternion(w=q_wxyz[0], x=q_wxyz[1], y=q_wxyz[2], z=q_wxyz[3])
        # - Accessors.
        self.assertEqual(to_value(q.w()), q_wxyz[0])
        self.assertEqual(to_value(q.x()), q_wxyz[1])
        self.assertEqual(to_value(q.y()), q_wxyz[2])
        self.assertEqual(to_value(q.z()), q_wxyz[3])
        self.assertTrue(allclose(q.xyz(), q_wxyz[1:]))
        self.assertTrue(allclose(q.wxyz(), q_wxyz))
        # - Mutators.
        q_wxyz_new = q_wxyz[::-1]
        self.assertFalse(allclose(q_wxyz, q_wxyz_new))
        q.set_wxyz(wxyz=q_wxyz_new)
        self.assertTrue(allclose(q.wxyz(), q_wxyz_new))
        q.set_wxyz(
            w=q_wxyz_new[0], x=q_wxyz_new[1], y=q_wxyz_new[2], z=q_wxyz_new[3])
        self.assertTrue(allclose(q.wxyz(), q_wxyz_new))
        # Alternative constructors.
        q_other = Quaternion(wxyz=q_wxyz)
        self.assertTrue(allclose(q_other.wxyz(), q_wxyz))
        R = np.array([
            [0, 0, 1],
            [1, 0, 0],
            [0, 1, 0]])
        q_wxyz_expected = np.array([0.5, 0.5, 0.5, 0.5])
        q_other = Quaternion(q_wxyz_expected)
        self.assertTrue(allclose(q_other.rotation(), R))
        R_I = np.eye(3, 3)
        q_other.set_rotation(R_I)
        self.assertTrue(allclose(q_other.wxyz(), q_identity.wxyz()))
        # - Copy constructor.
        cp = Quaternion(other=q)
        self.assertTrue(allclose(q.wxyz(), cp.wxyz()))
        # Bad values.
        if T != Expression:
            q = Quaternion.Identity()
            # - wxyz
            q_wxyz_bad = [1., 2, 3, 4]
            with self.assertRaises(RuntimeError):
                q.set_wxyz(q_wxyz_bad)
            self.assertTrue(allclose(q.wxyz(), [1, 0, 0, 0]))
            # - Rotation.
            R_bad = np.copy(R)
            R_bad[0, 0] = 10
            with self.assertRaises(RuntimeError):
                q_other.set_rotation(R_bad)
            self.assertTrue(allclose(q_other.rotation(), R_I))

        # Operations.
        q = Quaternion(wxyz=[0.5, 0.5, 0.5, 0.5])
        self.assertTrue(allclose(q.multiply(position=[1, 2, 3]), [3, 1, 2]))
        q_I = q.inverse().multiply(q)
        self.assertTrue(allclose(q_I.wxyz(), [1, 0, 0, 0]))
        if six.PY3:
            self.assertTrue(allclose(
                eval("q.inverse() @ q").wxyz(), [1, 0, 0, 0]))
        q_conj = q.conjugate()
        self.assertTrue(allclose(q_conj.wxyz(), [0.5, -0.5, -0.5, -0.5]))

        # Test `type_caster`s.
        if T == float:
            value = test_util.create_quaternion()
            self.assertTrue(isinstance(value, mut.Quaternion))
            test_util.check_quaternion(value)

    def test_isometry3(self):
        self.check_types(self.check_isometry3)

    def check_isometry3(self, T):
        Isometry3 = mut.Isometry3_[T]
        # - Default constructor
        transform = Isometry3()
        X = np.eye(4, 4)
        self.assertTrue(allclose(transform.matrix(), X))
        self.assertTrue(allclose(copy.copy(transform).matrix(), X))
        if T == float:
            self.assertEqual(str(transform), str(X))
        # - Constructor with (X)
        transform = Isometry3(matrix=X)
        self.assertTrue(allclose(transform.matrix(), X))
        # - Copy constructor.
        cp = Isometry3(other=transform)
        self.assertTrue(allclose(transform.matrix(), cp.matrix()))
        # - Identity
        transform = Isometry3.Identity()
        self.assertTrue(allclose(transform.matrix(), X))
        # - Constructor with (R, p)
        R = np.array([
            [0., 1, 0],
            [-1, 0, 0],
            [0, 0, 1]])
        p = np.array([1., 2, 3])
        X = np.vstack((np.hstack((R, p.reshape((-1, 1)))), [0, 0, 0, 1]))
        transform = Isometry3(rotation=R, translation=p)
        self.assertTrue(allclose(transform.matrix(), X))
        self.assertTrue(allclose(transform.translation(), p))
        transform.set_translation(-p)
        self.assertTrue(allclose(transform.translation(), -p))
        self.assertTrue(allclose(transform.rotation(), R))
        transform.set_rotation(R.T)
        self.assertTrue(allclose(transform.rotation(), R.T))
        # - Check transactions for bad values.
        if T != Expression:
            transform = Isometry3(rotation=R, translation=p)
            R_bad = np.copy(R)
            R_bad[0, 0] = 10.
            with self.assertRaises(RuntimeError):
                transform.set_rotation(R_bad)
            self.assertTrue(allclose(R, transform.rotation()))
            X_bad = np.copy(X)
            X_bad[:3, :3] = R_bad
            with self.assertRaises(RuntimeError):
                transform.set_matrix(X_bad)
            self.assertTrue(allclose(X, transform.matrix()))
        # Test `type_caster`s.
        if T == float:
            value = test_util.create_isometry()
            self.assertTrue(isinstance(value, mut.Isometry3))
            test_util.check_isometry(value)
        # Operations.
        transform = Isometry3(rotation=R, translation=p)
        transform_I = transform.inverse().multiply(transform)
        self.assertTrue(allclose(transform_I.matrix(), np.eye(4)))
        self.assertTrue(allclose(
            transform.multiply(position=[10, 20, 30]), [21, -8, 33]))
        if six.PY3:
            self.assertTrue(allclose(
                eval("transform.inverse() @ transform").matrix(), np.eye(4)))
            self.assertTrue(allclose(
                eval("transform @ [10, 20, 30]"), [21, -8, 33]))

    def test_translation(self):
        # Test `type_caster`s.
        value = test_util.create_translation()
        self.assertEqual(value.shape, (3,))
        test_util.check_translation(value)

    def test_angle_axis(self):
        value_identity = mut.AngleAxis.Identity()
        self.assertEqual(value_identity.angle(), 0)
        self.assertTrue((value_identity.axis() == [1, 0, 0]).all())

        # Construct with rotation matrix.
        R = np.array([
            [0., 1, 0],
            [-1, 0, 0],
            [0, 0, 1]])
        value = mut.AngleAxis(rotation=R)
        self.assertTrue(np.allclose(value.rotation(), R, atol=1e-15, rtol=0))
        self.assertTrue(np.allclose(
            copy.copy(value).rotation(), R, atol=1e-15, rtol=0))
        self.assertTrue(np.allclose(
            value.inverse().rotation(), R.T, atol=1e-15, rtol=0))
        self.assertTrue(np.allclose(
            value.multiply(value.inverse()).rotation(), np.eye(3),
            atol=1e-15, rtol=0))
        if six.PY3:
            self.assertTrue(np.allclose(
                eval("value @ value.inverse()").rotation(), np.eye(3),
                atol=1e-15, rtol=0))
        value.set_rotation(np.eye(3))
        self.assertTrue((value.rotation() == np.eye(3)).all())

        # Construct with quaternion.
        q = mut.Quaternion(R)
        value = mut.AngleAxis(quaternion=q)
        self.assertTrue(np.allclose(
            value.quaternion().wxyz(), q.wxyz(), atol=1e-15, rtol=0))
        value.set_quaternion(mut.Quaternion.Identity())
        self.assertTrue((value.quaternion().wxyz() == [1, 0, 0, 0]).all())

        # Test setters.
        value = mut.AngleAxis(value_identity)
        value.set_angle(np.pi / 4)
        v = normalize(np.array([0.1, 0.2, 0.3]))
        with self.assertRaises(RuntimeError):
            value.set_axis([0.1, 0.2, 0.3])
        value.set_axis(v)
        self.assertEqual(value.angle(), np.pi / 4)
        self.assertTrue((value.axis() == v).all())

        # Test symmetry based on accessors.
        # N.B. `Eigen::AngleAxis` does not disambiguate by restricting internal
        # angles and axes to a half-plane.
        angle = np.pi / 6
        axis = normalize([0.1, 0.2, 0.3])
        value = mut.AngleAxis(angle=angle, axis=axis)
        value_sym = mut.AngleAxis(angle=-angle, axis=-axis)
        self.assertTrue(np.allclose(
            value.rotation(), value_sym.rotation(), atol=1e-15, rtol=0))
        self.assertTrue(np.allclose(
            value.angle(), -value_sym.angle(), atol=1e-15, rtol=0))
        self.assertTrue(np.allclose(
            value.axis(), -value_sym.axis(), atol=1e-15, rtol=0))
