# Extended-Kalman-Filter-EKF-Sim
Approximate solutions to EKF type problems in code. The attempt is to validate manual solutions and display results in a plot, not anything revolutionary!

## 1. Problem Setup

(Combination of problems from various texts online) A cart rolls along a straight, one-dimensional track. A sensor sits a height $h$ above the track at $x=0$ and measures only the straight-line "**range**" to the cart. The goal is to estimate the cart's position and velocity from those range measurements alone.

The state vector holds everything the filter is trying to estimate:

$$
\mathbf{x} = \begin{bmatrix} p \\ v \end{bmatrix}
$$

where $p$ is position along the track and $v$ is velocity.

#### 1a. Dynamics

The cart follows a **constant-velocity** model. Over one time step $\Delta t$, position advances by $v\Delta t$ and velocity stays put:

$$
\mathbf{x}_{k+1} = F\mathbf{x}_k + \mathbf{w}_k, \qquad F = \begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix}
$$

No real cart moves at a perfectly constant velocity, so $\mathbf{w}_k$ is **process noise**: a small random acceleration that kicks the cart each step. It's drawn from a zero-mean Gaussian with covariance $Q$. If the random acceleration is white noise with intensity $q$, integrating it over one step gives:

$$
\boxed{Q = q\begin{bmatrix} \Delta t^3/3 & \Delta t^2/2 \\ \Delta t^2/2 & \Delta t \end{bmatrix}}
$$

The off-diagonal terms are nonzero because a random acceleration disturbs position *and* velocity together, so those errors are correlated.

#### 1b. Measurement

The sensor reports the hypotenuse between itself and the cart, plus noise:

$$
z_k = \sqrt{p_k^2 + h^2} + n_k, \qquad n_k \sim \mathcal{N}(0, \sigma_r^2)
$$

so the measurement noise covariance is simply $R = \sigma_r^2$. The square root makes this measurement **nonlinear** in the state. A standard Kalman filter needs linear models, which is why this problem calls for the *Extended* Kalman filter.

## 2. Truth Model

The truth model plays the role of the real world. It propagates the true state forward with real process noise and generates noisy measurements from it. The filter never sees the true state, only the measurements $z_k$.

This separation is what makes the test honest. If the filter and the "world" share the same code, a bug in the model is invisible because both sides make the same mistake. In a full study the truth model is deliberately *higher fidelity* than the filter's model (for orbit determination, the truth might include J2 and drag while the filter uses two-body motion), and the question becomes whether the filter still holds up.

## 3. Filter Models and Linearization

The filter keeps its own copy of the physics: a dynamics function $f(\mathbf{x})$ and a measurement function $h(\mathbf{x})$. The EKF handles nonlinearity by **linearizing** these functions around the current best estimate, using their Jacobians:

$$
F_k = \frac{\partial f}{\partial \mathbf{x}}, \qquad H_k = \frac{\partial h}{\partial \mathbf{x}}
$$

The dynamics here are already linear, so the Jacobian is just $F$. For the range measurement, with $r = \sqrt{p^2 + h^2}$:

$$
\frac{\partial r}{\partial p} = \frac{p}{r}, \qquad \frac{\partial r}{\partial v} = 0
$$

$$
\boxed{H_k = \begin{bmatrix} p/r & 0 \end{bmatrix}}
$$

Physically, $p/r$ is how sensitive the range is to a small change in position. Far from the sensor, $p/r \to 1$ and range tracks position almost one-to-one. Directly beneath the sensor, $p/r \to 0$ and a range measurement says almost nothing about where the cart is.

## 4. The Extended Kalman Filter

Each time step is two moves: **predict** where the cart should be, then **update** that guess with the new measurement. A superscript $-$ marks the predicted, pre-measurement quantities.

#### 4a. Predict

Push the estimate and its covariance forward one step:

$$
\hat{\mathbf{x}}_k^- = f(\hat{\mathbf{x}}_{k-1})
$$

$$
\boxed{P_k^- = F_k P_{k-1} F_k^T + Q}
$$

$P$ is the filter's own statement of how uncertain it is. Prediction always *grows* $P$, since adding $Q$ accounts for the random kicks the filter can't see.

#### 4b. Update

Compare the actual measurement against the measurement predicted from $\hat{\mathbf{x}}_k^-$. The difference is the **innovation**, the part of the measurement the filter didn't expect:

$$
\boldsymbol{\nu}_k = z_k - h(\hat{\mathbf{x}}_k^-)
$$

The innovation covariance $S$ is how large the innovation *should* be. It combines the predicted state uncertainty, mapped into measurement space, with the sensor noise:

$$
\boxed{S_k = H_k P_k^- H_k^T + R}
$$

The **Kalman gain** decides how much to trust the measurement versus the prediction:

$$
\boxed{K_k = P_k^- H_k^T S_k^{-1}}
$$

A large $P^-$ (uncertain prediction) relative to $R$ (precise sensor) makes $K$ large and the filter leans on the measurement. The reverse makes it lean on the prediction. The corrected state is then:

$$
\boxed{\hat{\mathbf{x}}_k = \hat{\mathbf{x}}_k^- + K_k\boldsymbol{\nu}_k}
$$

The covariance update uses the **Joseph form**:

$$
P_k = (I - K_k H_k)P_k^-(I - K_k H_k)^T + K_k R K_k^T
$$

Algebraically this equals the shorter $(I - K_k H_k)P_k^-$. Numerically, though, the Joseph form keeps $P$ symmetric and positive definite even with rounding error, which the short form does not guarantee.

## 5. Consistency Checks

A filter can produce a smooth, reasonable-looking estimate and still be wrong about its own uncertainty. Consistency checks test whether $P$ and $S$ actually describe the errors the filter is making.

#### 5a. The 3σ Bounds

The diagonal of $P$ holds the variance of each state. If the filter is consistent, the true error $\mathbf{e}_k = \mathbf{x}_k - \hat{\mathbf{x}}_k$ should stay inside $\pm 3\sqrt{P_{ii}}$ about 99.7% of the time. Errors that repeatedly leave the bounds mean the filter is **overconfident**. Errors that hug zero inside wide bounds mean it is **underconfident**.

#### 5b. NEES

The **Normalized Estimation Error Squared** scales the full error vector by the covariance:

$$
\boxed{\epsilon_k = \mathbf{e}_k^T P_k^{-1}\mathbf{e}_k}
$$

For a consistent filter, $\epsilon_k$ follows a chi-square distribution with $n$ degrees of freedom, where $n$ is the state dimension ($n=2$ here), so its expected value is $n$. NEES requires the true state, so it can only be computed in simulation.

#### 5c. NIS

The **Normalized Innovation Squared** does the same for the innovation:

$$
\boxed{\epsilon_{\nu,k} = \boldsymbol{\nu}_k^T S_k^{-1}\boldsymbol{\nu}_k}
$$

This follows a chi-square distribution with $m$ degrees of freedom, where $m$ is the measurement dimension ($m=1$ here). NIS needs no truth, so unlike NEES it can also be run on real data in operation.

#### 5d. Monte Carlo Averaging and Acceptance Bounds

A single run's NEES is too noisy to judge. Averaging over $N$ independent Monte Carlo runs at each time step gives:

$$
\bar{\epsilon}_k = \frac{1}{N}\sum_{i=1}^{N}\epsilon_k^{(i)}
$$

Since $N\bar{\epsilon}_k$ is chi-square with $Nn$ degrees of freedom, the two-sided 95% acceptance region is:

$$
\boxed{\frac{\chi^2_{0.025}(Nn)}{N} < \bar{\epsilon}_k < \frac{\chi^2_{0.975}(Nn)}{N}}
$$

With $N=50$ and $n=2$, this works out to roughly $1.49 < \bar{\epsilon}_k < 2.59$. For NIS with $m=1$, it's roughly $0.65 < \bar{\epsilon}_{\nu,k} < 1.43$. A consistent filter keeps both averages inside their bounds about 95% of the time. Sitting above the bounds means overconfident; sitting below means underconfident.

## References

- Y. Bar-Shalom, X. R. Li, and T. Kirubarajan, *Estimation with Applications to Tracking and Navigation*, Wiley, 2001.
- D. Simon, *Optimal State Estimation: Kalman, H∞, and Nonlinear Approaches*, Wiley, 2006.
