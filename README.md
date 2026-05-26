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
* **Hybrid Paradigm (`hybrid`):** The node performs a **Dynamic Bayesian Fusion with Tempered Priors and Agreement Gate**, in two steps:

  **Step 1 — LOP (Logarithmic Opinion Pool):**
  * The time $t$ elapsed since the start of the Continuous Feedback (CF) is calculated from `start_cf_` (reset on event 781).
  * A temperature $\alpha(t) = 0.5 \times (1 + \cos(\pi t / 2.5))$ decays from 1.0 to 0.0 over the first 2.5 seconds.
  * The CVSA probability is tempered: $P_\text{prior}(c) \propto P_\text{CVSA}(c)^\alpha$.
  * The MI probability is updated with this prior: $\text{lop}(c) \propto P_\text{MI}(c) \times P_\text{prior}(c)$.

  **Step 2 — Agreement Gate:**
  * The dot product $\text{agree} = \sum_c P_\text{CVSA}(c) \times P_\text{MI}(c)$ measures classifier agreement (0 = perfect disagreement, 1 = perfect agreement).
  * A neutral weight $\text{neutral\_w} = (1 - \text{agree\_w}) \times \alpha$ (where $\text{agree\_w}$ scales the dot product to $[0,1]$) determines how strongly to pull toward the uniform distribution.
  * Final output: $P_\text{out}(c) = (1 - \text{neutral\_w}) \times \text{lop}(c) + \text{neutral\_w} \times \frac{1}{n}$.

  **Effect:** When MI and CVSA agree, the gate is inactive and the LOP amplifies their shared prediction. When they disagree early in the trial (high $\alpha$), the output is pulled toward $[0.5, 0.5]$, preventing erroneous buffer movement. As the trial progresses ($\alpha \to 0$), the gate fades and MI takes full control regardless of CVSA.

### 3. Smoothing and Normalization
Regardless of the paradigm, the resulting data is passed through the loaded `rosneuro` integrator plugin (e.g., exponential smoothing) to eliminate micro-jitters. Finally, the probabilities are mathematically normalized based on the user-defined `thresholds` to ensure consistent control dynamics in the VR application.

---

## 📤 Output
Depending on the `paradigm` parameter, the node publishes on two separate topics:
* **Raw Topic:** `/[paradigm]/neuroprediction/integrated/raw`
* **Normalized Topic:** `/[paradigm]/neuroprediction/integrated/normalized`

Both topics publish the integrated probability as a `rosneuro_msgs::NeuroOutput` message.

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