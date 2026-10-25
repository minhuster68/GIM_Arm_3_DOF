"""The old nominal LQR audit must not silently ignore the new ESO dynamics."""
import importlib.util
from pathlib import Path

import pytest


def test_existing_lqr_audit_refuses_to_certify_an_eso_profile():
    root = Path(__file__).resolve().parents[3]
    spec = importlib.util.spec_from_file_location('lqr_audit_scope', root/'tools/check_lqr_circle.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with pytest.raises(ValueError, match='ESO'):
        module.build_controller({'eso_enabled': True}, None)
