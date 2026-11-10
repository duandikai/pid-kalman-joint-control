# Verification record

Execution environment: GNU Octave 6.4.0 on Linux. MATLAB was not available,
so execution under MATLAB itself has not been checked. Figures were rendered
using gnuplot 5.4. The optional independent verifier ran under Python 3 with
NumPy 1.26.4. The reference computation was prepared on 2026-11-10.

Commands:

```matlab
main('results', true)
```

```sh
python verify_results.py results
```

Passed checks:

- All 18 trajectories finish with finite states and preserve the specified
  voltage/current limits, motor current relation and discrete torque balance.
- Correct reference angular-velocity units, previous-sample control feedback,
  and identical normalized sensor noise across all cases.
- Positive covariance diagonals and nonnegative covariance determinants.
- Every exported scalar metric independently recomputed from CSV values;
  Table 2/3 entries traced to the appropriate sinusoidal, step or sensitivity run.
- Phase differences independently checked by sinusoidal least-squares fits.
- Every audit difference and rounding flag checked.
- 47 CSV output files are byte-identical between initial generation and
  subsequent execution using the deposited input noise.
- The standalone fallback, before depositing the noise file, also completed.

The exported figures were visually inspected. This verifies output rendering,
not correspondence to the original Word document's embedded images.

The manuscript comparison remains unresolved: 22/60 values agree at the
displayed precision. See MANUSCRIPT_AUDIT.md and summary/manuscript_comparison.csv.
