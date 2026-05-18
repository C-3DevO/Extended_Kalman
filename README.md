# Extended Kalman Filter (EKF) for Nonlinear Target Tracking

This project implements an Extended Kalman Filter (EKF) in MATLAB for nonlinear vehicle tracking using noisy range and bearing measurements.

The EKF is used to estimate the position and velocity of a moving target in Cartesian coordinates while handling nonlinear radar measurements.

---

## Overview

In many radar and navigation systems, the target motion evolves linearly, but the sensor measurements are nonlinear.

The vehicle state vector is defined as:

```math
s[n] =
\begin{bmatrix}
r_x[n] \\
r_y[n] \\
v_x[n] \\
v_y[n]
\end{bmatrix}
```

where:

- `r_x[n], r_y[n]` → target position
- `v_x[n], v_y[n]` → target velocity

The EKF estimates the target trajectory recursively from noisy range and bearing observations.

---

## State-Space Model

The target follows a constant-velocity motion model.

### State Evolution

```math
s[n] = As[n-1] + u[n]
```

with transition matrix:

```math
A =
\begin{bmatrix}
1 & 0 & \Delta & 0 \\
0 & 1 & 0 & \Delta \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
```

where:

- `Δ` is the sampling interval
- `u[n]` is Gaussian process noise

---

## Nonlinear Measurement Model

The radar sensor provides:

- Range
- Bearing angle

The observation model is:

```math
x[n] =
\begin{bmatrix}
\sqrt{r_x^2[n] + r_y^2[n]} \\
\tan^{-1}\left(\frac{r_y[n]}{r_x[n]}\right)
\end{bmatrix}
+
w[n]
```

where:

- `w[n]` is Gaussian measurement noise

This nonlinear measurement equation motivates the use of the Extended Kalman Filter.

---

## EKF Algorithm

The EKF linearizes the nonlinear observation model using a Jacobian matrix.

---

## Prediction Step

### Predicted State

```math
\hat{s}[n|n-1] = A\hat{s}[n-1|n-1]
```

### Predicted Covariance

```math
M[n|n-1] =
AM[n-1|n-1]A^T + Q
```

---

## Linearization

The nonlinear observation function is linearized around the predicted state.

Define:

```math
\rho = \sqrt{r_x^2 + r_y^2}
```

The Jacobian becomes:

```math
H[n] =
\begin{bmatrix}
\frac{r_x}{\rho} & \frac{r_y}{\rho} & 0 & 0 \\
-\frac{r_y}{\rho^2} & \frac{r_x}{\rho^2} & 0 & 0
\end{bmatrix}
```

---

## Update Step

### Innovation

```math
e[n] = x[n] - h(\hat{s}[n|n-1])
```

### Kalman Gain

```math
K[n] =
M[n|n-1]H^T[n]
S^{-1}[n]
```

### Corrected State

```math
\hat{s}[n|n] =
\hat{s}[n|n-1]
+
K[n]e[n]
```

### Covariance Update

```math
M[n|n] =
(I-K[n]H[n])M[n|n-1]
```

---

## MATLAB Features

The implementation includes:

- Nonlinear radar observation modeling
- Range and bearing measurements
- EKF prediction and correction
- Jacobian linearization
- State covariance tracking
- Cartesian trajectory reconstruction
- Minimum MSE analysis

---

## Simulations Performed

### Task 1 — Baseline EKF Tracking

Parameters:

```matlab
sigma_u^2 = 0.0001
sigma_r^2 = 0.1
sigma_b^2 = 0.01
```

Results:

- EKF successfully tracks the vehicle trajectory
- Initial transient due to poor initialization
- Covariance decreases during convergence
- EKF estimate closely follows the true trajectory

---

## Task 2 — Poor Initialization

Initial state changed to:

```math
\hat{s}[-1|-1] =
[50,-50,2,-2]^T
```

Observations:

- EKF diverges significantly
- Linearization becomes inaccurate
- Covariance shrinks despite incorrect estimates
- Demonstrates EKF sensitivity to initialization

This highlights one of the major limitations of EKF:
linearization around incorrect operating points can cause filter inconsistency.

---

## Task 3 — Increased Process Noise

Parameters:

```matlab
sigma_u^2 = 0.01
```

Effects observed:

- Vehicle trajectory becomes more random
- EKF uncertainty grows over time
- MSE increases significantly
- Tracking becomes less stable

Higher process noise injects uncertainty into velocity states, which accumulates into position estimation error.

---

## Task 4 — Increased Measurement Noise

Parameters:

```matlab
sigma_r^2 = 1
sigma_b^2 = 0.5
```

Observations:

- Observed trajectory becomes highly scattered
- EKF relies more on motion model than measurements
- Estimates remain smoother
- Tracking precision decreases

This demonstrates the EKF tradeoff between:

- model trust
- measurement trust

---

## Key Observations

### EKF Strengths

- Handles nonlinear measurement systems
- Recursive estimation with low complexity
- Good tracking performance under moderate noise
- Converges successfully with reasonable initialization

### EKF Limitations

- Sensitive to initialization
- Linearization errors can cause divergence
- Performance degrades under strong nonlinearities
- Covariance may underestimate actual error

---

## Main Findings

- EKF performs well when the initial estimate is close to the true state
- Large initialization errors lead to divergence
- Increased process noise causes growing estimation uncertainty
- Increased measurement noise reduces tracking precision
- EKF uncertainty evolution is reflected in covariance terms:

```math
M_{11}[n|n], \quad M_{22}[n|n]
```

which represent the minimum MSE estimates for position states.

---

## Files

- `ekf_tracking.m` — MATLAB implementation
- `EKF_Report.pdf` — Full report with derivations, plots, experiments, and discussion :contentReference[oaicite:0]{index=0}

---

## References

1. S. M. Kay, *Fundamentals of Statistical Signal Processing: Estimation Theory*, Prentice Hall, 1993.

2. K. P. Murphy, *Machine Learning: A Probabilistic Perspective*, MIT Press, 2012.
