import os
from os.path import isdir, isfile, join
from shutil import rmtree
import unittest
from subprocess import STDOUT, call, check_call, check_output
import sys


class TestInstallMeta(unittest.TestCase):
    BINARY = "tools/install/dummy/dummy_install"

    def setUp(self):
        assert "TEST_TMPDIR" in os.environ, (
            "Must only be run within `bazel test`.")

    def get_python_site_packages_dir(self):
        major, minor = sys.version_info.major, sys.version_info.minor
        return join("lib", "python{}.{}".format(major, minor), "site-packages")

    def get_install_dir(self, case):
        # Do not use `install_test_helper`, as its behavior more constrained
        # than what is useful for these tests.
        install_dir = join(os.environ["TEST_TMPDIR"], "installation", case)
        # Ensure this is only called once per case.
        self.assertFalse(isdir(install_dir), case)
        return install_dir

    def test_nominal(self):
        """Test nominal behavior of install."""
        install_dir = self.get_install_dir("test_nominal")
        check_call([self.BINARY, install_dir])
        py_dir = self.get_python_site_packages_dir()
        expected_manifest = [
            "share/README.md",
            "lib/libdummy.so",
            join(py_dir, "dummy.py"),
        ]
        for expected_file in expected_manifest:
            file_path = join(install_dir, expected_file)
            self.assertTrue(isfile(file_path), expected_file)

    def test_strip_args(self):
        """Test behavior of `--no_strip` and related arguments."""
        install_dir = self.get_install_dir("test_strip_args")
        # Negative test: Ensure `--disable_nostrip_warning` is not used in a
        # blanket fashion.
        returncode = call(
            [self.BINARY, install_dir, "--disable_nostrip_warning"])
        self.assertEqual(returncode, 1)
        # Test for warnings with no stripping:
        warning_substr = "WARNING: Symbols are not stripped"
        # - Nominal.
        text_with_warning = check_output([
            self.BINARY, install_dir, "--no_strip"], stderr=STDOUT)
        self.assertIn(warning_substr, text_with_warning, text_with_warning)
        # - Warning disabled.
        text_without_warning = check_output([
            self.BINARY, install_dir, "--no_strip",
            "--disable_nostrip_warning"], stderr=STDOUT)
        self.assertNotIn(warning_substr, text_without_warning)
