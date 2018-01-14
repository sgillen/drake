load("@drake//tools/external_data:test.bzl", "external_data_test_workspaces")  # noqa

def test_repositories(workspace_dir):
    external_data_test_workspaces(workspace_dir)
