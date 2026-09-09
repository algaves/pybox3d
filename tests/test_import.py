import pybox3d


def test_import_succeeds():
    assert pybox3d is not None


def test_version_present():
    assert isinstance(pybox3d.__version__, str)
    assert pybox3d.__version__


def test_all_public_names_exported():
    for name in pybox3d.__all__:
        assert hasattr(pybox3d, name), f"pybox3d.{name} missing"


def test_box_kind_constants():
    assert pybox3d.BOX_KIND_AABB != pybox3d.BOX_KIND_OBB
