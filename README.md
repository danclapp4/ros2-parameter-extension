# ROS2 Parameters Extension

## _A Foxglove Studio Extension_

The **ROS2 Parameters Extension** provides parameter functionality for all your ROS2 nodes including: 

- **View** a node's parameter names, types, and values in a table format
- **Set** new parameter values for all types of parameters on a node
- **Save** a node's paramter configuration to a .yml file 
- **Load** a previous configuration from a .yml file stored on your computer

## Installation

- Download NodeJs 14+ [here](https://nodejs.org/en/download/)
- from the bin folder, run:
```
npm install --global yarn
```

- from inside your folder where you cloned this repo, run: 
```
yarn install
``` 
This will install the missing node modules and dependencies 

- To build and add extension to foxglove, run:
```
yarn local-install
```

- To install and run Rosbridge Websocket, run:

```
sudo apt install ros-galactic-rosbridge-suite
```
```
ros2 launch rosbridge_server rosbridge_websocket_launch.xml


```
