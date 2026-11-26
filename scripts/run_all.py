"""Every measurement in order; step 2 needs ngspice (CI).

    python scripts/run_all.py
"""
import _setup  # noqa: F401
import step1_design
import step2_spice

from mct import spice

if __name__ == "__main__":
    step1_design.main()
    if spice.have_ngspice():
        step2_spice.main()
    else:
        print("ngspice not installed: step 2 skipped")
