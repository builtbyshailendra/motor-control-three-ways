"""Every measurement in order; steps 2 and 3 need ngspice and omc (CI).

    python scripts/run_all.py
"""
import _setup  # noqa: F401
import step1_design
import step2_spice
import step3_modelica
import step4_windup

from mct import modelica, spice

if __name__ == "__main__":
    step1_design.main()
    if spice.have_ngspice():
        step2_spice.main()
    else:
        print("ngspice not installed: step 2 skipped")
    if modelica.have_omc():
        step3_modelica.main()
    else:
        print("omc not installed: step 3 skipped")
    step4_windup.main()
