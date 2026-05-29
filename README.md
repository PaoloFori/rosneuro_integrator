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
* `cvsa_hold`: Plateau duration in seconds — CVSA stays at full influence (α=1) for this long after the reset event (default 1.0 s).
* `cvsa_influence`: Cosine decay duration in seconds — after the plateau, α decays from 1 to 0 over this window (default 3.0 s).

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
* **Hybrid Paradigm (`hybrid`):** The node performs a **Dynamic Bayesian Fusion with Plateau + Cosine-Annealed CVSA Prior (LOP)**:

  The temperature $\alpha(t)$ controls how strongly CVSA acts as a prior:

  $$\alpha(t) = \begin{cases} 1 & t \leq T_\text{hold} \\ \tfrac{1}{2}\!\left(1 + \cos\!\left(\dfrac{\pi\,(t - T_\text{hold})}{T_\text{decay}}\right)\right) & T_\text{hold} < t \leq T_\text{hold} + T_\text{decay} \\ 0 & t > T_\text{hold} + T_\text{decay} \end{cases}$$

  where $T_\text{hold}$ = `cvsa_hold` (default 1.0 s) and $T_\text{decay}$ = `cvsa_influence` (default 3.0 s).

  The fused output is the Logarithmic Opinion Pool (LOP):
  * $P_\text{prior}(c) \propto P_\text{CVSA}(c)^\alpha$
  * $P_\text{out}(c) \propto P_\text{MI}(c) \times P_\text{prior}(c)$

  **Overall behaviour:**
  | Scenario | Output |
  |----------|--------|
  | Both agree on class A | LOP boosts above both inputs |
  | Symmetric disagreement | Products cancel → uniform naturally |
  | Asymmetric disagreement at $t \leq T_\text{hold}$ | CVSA redirects (reliable at trial onset) |
  | CVSA uncertain ($P_\text{CVSA} \approx 1/n$) | Prior ≈ uniform → pure $P_\text{MI}$ |
  | $t > T_\text{hold} + T_\text{decay}$ ($\alpha = 0$) | Pure $P_\text{MI}$ |

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

### Example usage
```bash
rosrun rosneuro_integrator integrator _plugin:=rosneuro::Buffer _paradigm:=hybrid
```