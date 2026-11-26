# motor-control-three-ways

[![ci](https://github.com/builtbyshailendra/motor-control-three-ways/actions/workflows/ci.yml/badge.svg)](https://github.com/builtbyshailendra/motor-control-three-ways/actions/workflows/ci.yml)

> Personal learning project, built pairing with Claude Code.

One DC-motor speed loop, simulated three independent ways (python-control, ngspice through PySpice, OpenModelica) and checked against each other. Work in progress: step 1 (python-control design) step 2 (the op-amp circuit in ngspice) and step 3 (the Modelica model in OpenModelica); steps 2 and 3 run in CI.

```bash
pip install -r requirements.txt
python -m pytest
python scripts/step1_design.py
```
