# ROS-Neuro integrator package
The package provides a generic interface for integrating [NeuroOutput](https://github.com/rosneuro/rosneuro_msgs) messages. The interace accepts plugins that can be independently developed and dynamically loaded.

## Usage
The package acquires **NeuroOutput** messages, applies the loaded integrator and publishes the resulting integrated data as **NeuroOutput** message. The following command can be used to run the node:
```
rosrun rosneuro_integrator integrator _plugin:=[INTEGRATORPLUGIN] [OPTIONAL PLUGIN-RELATED PARAMETERS]
```
**Example with ExponentialSmoothing plugin**
```
rosrun rosneuro_integrator integrator _plugin:=rosneuro::Exponential _alpha:=0.97
```
