# PID control with Kalman filter sensor fusion

Simulation code for **Performance Evaluation of PID Control with Kalman
Filter-Based Sensor Fusion for Robotic Joints**.

The model compares encoder-only PID, 10-Hz low-pass-filtered PID and KF-PID
for a single robotic joint. It also includes a 30-degree step test and
measurement-noise, inertia and damping sensitivity tests.

## Run

Download the repository, open its folder in MATLAB, and enter:

```matlab
main
```

All model parameters and helper functions are included in `main.m`. No
additional MATLAB toolboxes, Simulink models or private files are required.
The code is intended for MATLAB R2018b or later and has been tested with
GNU Octave 6.4.0. Execution under MATLAB itself has not yet been verified.
To run without generating figures:

```matlab
main('results', false)
```

## Data and outputs

This is a numerical simulation study; no empirical or third-party research
data are used. Running `main` generates and saves the numerical outputs:

| Location | Contents |
|---|---|
| `results/data/tracking_*.csv` | Time series underlying Figures 2–11, with units in column headers |
| `results/data/step_*.csv` | Separate step-test trajectories and metrics |
| `results/data/robustness_*.csv` | Sensitivity-test trajectories, settings and metrics |
| `results/data/configuration.csv` | Model, sensor, controller and filter parameters |
| `results/data/table2_controller_comparison.csv` | Controller comparison calculated from the trajectories |
| `results/data/table3_robustness.csv` | Sensitivity comparison calculated from the trajectories |
| `results/data/phase_*.csv` | Phase estimates and fitting coefficients |
| `results/data/nominal_*.csv` | Nominal characteristic polynomial and poles |
| `results/data/estimator_eigenvalues.csv` | Steady estimator error-update eigenvalues |
| `results/figures/` | Generated figures; Figure 1 is a conceptual diagram |
| `results/simulation_results.mat` | Complete simulation data, configuration and metrics |

Figures 2–5 use the encoder-only run; Figures 6–11 use the KF-PID run.
Tracking errors compare the reference with the true simulated state;
estimation errors compare sensor/filter outputs with that true state.
Primary metrics use 1–5 s inclusive. Full-record estimation metrics are
also saved. Step settling uses a 2% band through the 5 s simulation horizon;
`NaN` means not settled, or not applicable to a sinusoidal run.

The LPF uses backward Euler. Phase differences are calculated by fitting a
1-Hz sinusoid and an intercept over 1–5 s, comparing the KF and an offline
LPF against the same true trajectory. Controller and KF tuning remain fixed
in the sensitivity tests. Nominal poles are a local linearized check, not
a proof of nonlinear closed-loop stability.

## Reproducibility

Keep `data/noise_standard.csv` with the code. It contains the common
normalized encoder and gyroscope noise used by all runs. It was generated
in GNU Octave 6.4.0 using `randn('state',42)`, with interleaved encoder/gyro
draws and a zero initialization row. Its SHA-256 is
`2598fbf544dedfb815950e3cd08d16854ecebc5015148c1a12f5f2780937522b`.

The saved realization avoids differences between MATLAB and Octave random
number generators. Without this file, the script generates and saves new
seed-42 noise, whose values may differ between software environments.
Repeated execution with the included noise reproduced all 47 CSV files
exactly in the tested environment.

This implementation follows the revised manuscript's stated parameters;
it is not a recovered copy of the program that originally generated the
manuscript. Some recomputed numbers differ from the reported values.
`results/data/manuscript_comparison.csv` lists those differences, which
must be reconciled before claiming reproduction of the unchanged manuscript.

## License

MIT. See `LICENSE`.
