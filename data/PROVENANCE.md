# Deposited sensor noise

`noise_standard.csv` was generated for this reconstruction in GNU Octave
6.4.0 on Linux on 2026-11-10. It is not claimed to be the noise sequence used
to obtain the original manuscript's numerical results.

The generating commands were:

```matlab
randn('state',42);
noise = [0,0; reshape(randn(2*5000,1),2,[])'];
t = (0:0.001:5)';
```

The columns are time in seconds, normalized encoder noise, and normalized
gyroscope noise. The two channels are drawn in interleaved order, encoder
then gyro. The first row is zero because the model starts from a known zero
state before the first simulated sensor update. The script scales the
remaining samples by the channel's specified noise standard deviation.

The exact CSV bytes have SHA-256:

`2598fbf544dedfb815950e3cd08d16854ecebc5015148c1a12f5f2780937522b`

The same file is used for all runs, including step tests and sensitivity
cases. Saving the realization lets MATLAB and Octave use exactly the same
input values even when their normal-random-number sequences differ. This
does not imply bitwise-identical floating-point calculations across all
software versions or processor platforms.

References for the native generator interfaces:
[GNU Octave 6.4](https://docs.octave.org/v6.4.0/Special-Utility-Matrices.html)
and [MATLAB rng](https://www.mathworks.com/help/matlab/ref/rng.html).
