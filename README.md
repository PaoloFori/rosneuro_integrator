# ROS-Neuro Integrator Package (MI / CVSA / Hybrid BCI)

This package provides a highly modular interface for a Brain-Computer Interface (BCI) project, designed to **synchronize, integrate, and normalize multiple data streams** coming from distinct neuroprediction pipelines (Motor Imagery and Covert Visuospatial Attention).

The node dynamically adapts its behavior based on the selected `paradigm` parameter, managing the synchronization of the following input streams:
1.  `/cvsa/neuroprediction/raw` (type `rosneuro_msgs::NeuroOutput`): The raw probabilities from the CVSA classifier.
2.  `/mi/neuroprediction/raw` (type `rosneuro_msgs::NeuroOutput`): The raw probabilities from the MI classifier.
3.  `/artifact_presence` (type `artifacts_bci::artifact_presence`): A custom message acting as a gatekeeper, indicating whether the current sample contains artifacts (e.g., EOG).
4.  `/events/bus` (type `rosneuro_msgs::NeuroEvent`): Used to listen for specific events (e.g., the start of the Continuous Feedback) to reset the integrator and the timeline.

---

## ⚙️ Parameters
The node requires several parameters to be set in the ROS parameter server:
* `plugin`: The underlying rosneuro integration plugin to apply (e.g., `rosneuro::Buffer`).
* `paradigm`: The operating mode. Accepted values are `cvsa`, `mi`, or `hybrid`.
* `classes`: A list of the classes used by the decoders.
* `thresholds`: The probability thresholds required to trigger a "Hit" for each class. Used for real-time normalization.
* `reset_event`: The event code used to reset the integrator and the temporal baseline for the Bayesian fusion.
* `cvsa_influence`: Total cosine decay duration in seconds. α goes from 1 at t=0 (full CVSA) to 0.5 at t=T/2 (equal MI/CVSA) to 0 at t=T (pure MI). Default 3.0 s.

---

## 🔄 Synchronization Logic
The node implements a robust, thread-safe asynchronous buffer to synchronize the topics.

1.  **Sequence Matching:** Integration strictly relies on the sequence number (`seq`) present in the headers of all incoming messages.
2.  **Dynamic Buffering:** The node creates a `Sync_Set` for each incoming `seq`. Depending on the `paradigm`, it waits for the required combination of messages (e.g., `mi` + `cvsa` + `artifacts` for the hybrid mode).
3.  **Timeout Pruning:** A dedicated timer runs at 2 Hz to check the buffer. If a `Sync_Set` is older than `max_age_` (1.0 second) and is still incomplete, it is dropped to prevent memory leaks and keep the system operating in strict real-time.
4.  **Execution:** Once a `Sync_Set` collects all required messages for a specific `seq`, it is immediately sent to the integration pipeline.

---

## 🧠 Integration Logic & Hybrid Paradigm

The `integrateSyncData` function applies a three-step processing pipeline:

### 1. Artifact Gatekeeper
If the artifact topic flags `has_artifact: true`, the node immediately freezes the output. It bypasses the classifiers and outputs the last known stable probability from the underlying plugin (`getData()`), preventing erratic cursor movements in VR due to eye blinks.

### 2. Paradigm Routing
* **Single Paradigms (`cvsa` or `mi`):** The node acts as a pass-through, feeding the raw probabilities directly into the generic `rosneuro` integrator plugin.
* **Hybrid Paradigm (`hybrid`):** The node performs **Cosine-Annealed Bayesian Fusion (LOP)**:

  The temperature $\alpha(t)$ controls how strongly CVSA acts as a prior:

  $$\alpha(t) = \begin{cases} \tfrac{1}{2}\!\left(1 + \cos\!\left(\dfrac{\pi\,t}{T}\right)\right) & 0 \leq t < T \\ 0 & t \geq T \end{cases}$$

  where $T$ = `cvsa_influence` (default 3.0 s).

  | t | α (T=3 s) | MI weight | Note |
  |---|-----------|-----------|------|
  | 0.0 s | 1.000 | 0% | CVSA fully active |
  | 0.5 s | 0.933 | 7% | slow start |
  | 1.0 s | 0.750 | 25% | CVSA still dominant |
  | 1.5 s | 0.500 | 50% | crossover |
  | 2.0 s | 0.250 | 75% | MI dominant |
  | 3.0 s | 0.000 | 100% | pure MI |

  The fused output is the Logarithmic Opinion Pool (LOP):
  * $P_\text{prior}(c) \propto P_\text{CVSA}(c)^\alpha$
  * $P_\text{out}(c) \propto P_\text{MI}(c) \times P_\text{prior}(c)$

  **Overall behaviour:**
  | Scenario | Output |
  |----------|--------|
  | Both agree on class A | LOP boosts P_out above either input alone |
  | Symmetric disagreement | Products cancel → P_out ≈ [0.5, 0.5] → buffer stalls |
  | CVSA uncertain ($P_\text{CVSA} \approx 1/n$) | Prior ≈ uniform → P_out ≈ $P_\text{MI}$ |
  | $t \geq T$ ($\alpha = 0$) | Prior = uniform → pure $P_\text{MI}$ |

### 3. Buffer Integration and Normalization
Regardless of the paradigm, the fused probabilities are passed through the loaded `rosneuro` integrator plugin (e.g., `rosneuro::integrator::Buffer` — winner-take-all leaky integrator with HARD/SOFT step modes).

The integrator publishes only `integrated/raw`. Normalization is performed downstream by `feedback_bci_vr/training_node`, which subscribes to `raw`, applies a per-class linear stretch, and publishes `integrated/normalized`:
```
slope_i = (1 - p_rest) / (threshold_i - p_rest)   where p_rest = 1/n_classes
normalized_i = clamp(p_rest + (raw_i - p_rest) * slope_i, 0, 1)
```
This maps `raw_i = threshold_i → normalized_i = 1.0` independently for each class. The two outputs serve different consumers:
* `integrated/raw` → consumed by `training_node` (evaluation modality) for hit detection: `raw[i] >= threshold[i]`.
* `integrated/normalized` → consumed by Unity (`BCIUniversalController`) for visual/audio feedback: `offset = max(0, (normalized − 0.5) × 2)` maps `[0.5, 1.0] → [0, 1]`.

The visual goal (cube at max position, audio at max volume) and the hit detection fire at the same physical instant because both are gated by the same `threshold` value.

---

## 📤 Output
Depending on the `paradigm` parameter, the node publishes:
* **Raw Topic:** `/[paradigm]/neuroprediction/integrated/raw` (`rosneuro_msgs::NeuroOutput`)

The `integrated/normalized` topic is published by `feedback_bci_vr/training_node`, which subscribes to `raw` and applies the per-class linear-stretch normalization.

---

## 🚀 Usage
The package acquires raw `NeuroOutput` messages, applies the synchronization/fusion logic, and publishes the resulting data. The following command runs the node:
```bash
rosrun rosneuro_integrator integrator _plugin:=[INTEGRATORPLUGIN] [OPTIONAL PLUGIN-RELATED PARAMETERS]
```

### Example launch
```bash
rosrun rosneuro_integrator integrator _plugin:=rosneuro::Buffer _paradigm:=hybrid
```

---

## 🔢 Worked numerical examples (T = 3 s, binary BCI)

All examples use `cvsa_influence=3.0 s`, `buffer_size=40`, `k_gain=2`, `framerate=20 Hz`, `threshold_c1=0.95`.

**SOFT step**: `step = min(|P_out − 0.5| × 2 × 2, 1) / 40` → max step = 0.025/frame = 0.5 buffer/s.

---

### Case 1 — Agreement (both classifiers say class 1)

```
P_MI   = [0.75, 0.25]
P_CVSA = [0.70, 0.30]

LOP at selected time points:
  t=0.0s  α=1.000  P_prior=[0.700,0.300]  P_out=[0.875,0.125]  step=0.025  Δbuf/s=0.50
  t=1.0s  α=0.750  P_prior=[0.654,0.346]  P_out=[0.851,0.149]  step=0.025  Δbuf/s=0.50
  t=1.5s  α=0.500  P_prior=[0.604,0.396]  P_out=[0.821,0.179]  step=0.025  Δbuf/s=0.50
  t=2.0s  α=0.250  P_prior=[0.553,0.447]  P_out=[0.787,0.213]  step=0.025  Δbuf/s=0.50
  t=3.0s  α=0.000  P_prior=[0.500,0.500]  P_out=[0.750,0.250]  step=0.025  Δbuf/s=0.50
```

Step is capped throughout because LOP keeps P_out > 0.75 at all times.
Buffer fills at 0.5/s → 0.5 to 0.95 in **18 frames = 0.9 s → HIT**.

---

### Case 2 — Disagreement (MI says class 1, CVSA says class 2)

```
P_MI   = [0.70, 0.30]
P_CVSA = [0.30, 0.70]   ← opposing attention

LOP at selected time points:
  t=0.0s  α=1.000  P_prior=[0.300,0.700]  P_out=[0.500,0.500]  step=0.000  Δbuf/s=0.00
  t=0.5s  α=0.933  P_prior=[0.317,0.683]  P_out=[0.507,0.493]  step=0.001  Δbuf/s=0.01
  t=1.0s  α=0.750  P_prior=[0.346,0.654]  P_out=[0.553,0.447]  step=0.005  Δbuf/s=0.10
  t=1.5s  α=0.500  P_prior=[0.396,0.604]  P_out=[0.605,0.395]  step=0.011  Δbuf/s=0.21
  t=2.0s  α=0.250  P_prior=[0.447,0.553]  P_out=[0.653,0.347]  step=0.016  Δbuf/s=0.31
  t=3.0s  α=0.000  P_prior=[0.500,0.500]  P_out=[0.700,0.300]  step=0.020  Δbuf/s=0.40
```

Integration trajectory (buffer starting from 0.5 at reset):
```
Frame   t(s)    buffer(c1)   Note
  1     0.00     0.500       reset
  2     0.05     0.500       frozen (LOP cancels at α≈1)
 20     0.95     0.540       barely moving
 30     1.45     0.622       α past midpoint, MI gradually takes over
 40     1.95     0.761       step growing
 51     2.50     0.950       HIT — but 2.5 s later vs 0.9 s with agreement
```

The disagreement freezes the buffer for ~1 s, then it slowly recovers as α→0 and P_out→P_MI.
A shorter CF window would produce a timeout instead.

---

### Case 3 — CVSA uncertain

```
P_MI   = [0.78, 0.22]   (MI confident, mid-trial)
P_CVSA = [0.52, 0.48]   (CVSA near chance)
α = 0.5

P_prior ∝ [0.52^0.5, 0.48^0.5] = [0.721, 0.693] → normalised [0.510, 0.490]
P_out   ∝ [0.78×0.510, 0.22×0.490] = [0.398, 0.108] → [0.787, 0.213]
```

Uncertain CVSA → near-uniform prior → P_out ≈ P_MI regardless of α. Fusion degrades gracefully to pure MI when spatial attention is not established.

---

### Key takeaway

| CVSA state | P_out vs P_MI | Integration speed |
|------------|---------------|-------------------|
| Strong agreement | P_out > P_MI (LOP boost) | Fast (max step) |
| Mild agreement | P_out slightly > P_MI | Moderate |
| Uncertain CVSA | P_out ≈ P_MI | Same as MI alone |
| Disagreement at t=0 | P_out ≈ 0.5 (cancels) | Near zero (frozen) |
| Disagreement at t=T | P_out = P_MI | Same as MI alone |