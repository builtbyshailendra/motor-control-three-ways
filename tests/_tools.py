"""Skip markers for the CI-only tools.

Locally (Windows, no ngspice or OpenModelica) these tests skip. CI sets
MCT_REQUIRE_TOOLS=1, which turns a missing tool into a failure instead of a skip,
so a broken install can't hide behind a green run.
"""
import os
import shutil

import pytest

REQUIRE = os.environ.get("MCT_REQUIRE_TOOLS") == "1"


def _needs(tool: str):
    return pytest.mark.skipif(shutil.which(tool) is None and not REQUIRE,
                              reason=f"{tool} not installed here (runs in CI)")


needs_ngspice = _needs("ngspice")
needs_omc = _needs("omc")
