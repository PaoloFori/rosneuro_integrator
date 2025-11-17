# ROS-Neuro Integrator Package (CVSA)

This package provides a specific interface for the CVSA project, designed to **synchronize and integrate three different data streams** from the neuroprediction pipeline.

The node manages and synchronizes the following three input messages:
1.  `/cvsa/neuroprediction/icnic` (type `rosneuro_msgs/NeuroOutput`): Indicates the probability of the IC (Impaired Consciousness) vs. NIC (Not Impaired Consciousness) state.
2.  `/cvsa/neuroprediction/raw` (type `rosneuro_msgs/NeuroOutput`): Contains the "raw" probability from the classifier (e.g., right vs. left).
3.  `/cvsa/artifact_presence` (type `artifact_presence`): A custom message indicating whether the current sample contains artifacts (`has_artifact: true`) or not.

The node publishes the resulting integrated data as a single **NeuroOutput** message.

---

## Dependencies
This package depends on the following message types:
* `rosneuro_msgs/NeuroOutput`
* `cvsa_msgs/artifact_presence` (or the name of the package where it is defined)

---

## Synchronization Logic
The node uses an internal buffer to synchronize the three input topics.

1.  **Synchronization via `seq`:** Integration relies on the sequence number (`seq`) present in all messages.
2.  **Buffering:** The node waits to receive all three messages (`icnic`, `raw`, `artifact_presence`) with the **same sequence number**.
3.  **Dropping:** If the messages are not complete for a given `seq` (e.g., `icnic` and `raw` arrive, but `artifact_presence` is missing before a newer `seq` arrives), that data packet is considered pending and is dropped to keep the system aligned and in real-time.
4.  **Integration:** Only when a complete data set (with matching `seq`) is available are the data sent to the integration algorithm.

---

## Output
* **Topic:** `/cvsa/neuroprediction/integrated` (or similar)
* **Data:** Publishes the integrated probability (based on the inputs and artifact status) as a `rosneuro_msgs/NeuroOutput` message.

---


## Usage
The package acquires **NeuroOutput** messages, applies the loaded integrator and publishes the resulting integrated data as **NeuroOutput** message. The following command can be used to run the node:
```
rosrun rosneuro_integrator integrator _plugin:=[INTEGRATORPLUGIN] [OPTIONAL PLUGIN-RELATED PARAMETERS]
```
**Example with ExponentialSmoothing plugin**
```
rosrun rosneuro_integrator integrator _plugin:=rosneuro::Exponential _alpha:=0.97
```
