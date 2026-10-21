# HEX calibration runs

`hex_calibration_node.py` writes one folder per run here, `<date_time>/`:

| file | |
|---|---|
| `s1.npz`, `s2.npz`, … | the recordings of both sensors (not in git, see `.gitignore`) |
| `hex_calibration.yaml` | the calibration fitted in that run |
| `hex_calibration_report.txt` | errors before/after, identified mount, notes |

The active calibration, which `launch/hex_calibrated.launch` loads, is `../cfg/hex_calibration.yaml`.
Every usable fit replaces it. To go back to an older run:
`cp calibration/<date_time>/hex_calibration.yaml cfg/hex_calibration.yaml`.
