# view_motion_planner

Full motion planner for optimizing fruit coverage

## Setup

You can use the workspace provided [here](https://github.com/Eruvae/vmp_ws). Follow the instructions in the readme to clone all required packages, install their dependencies and set up the workspace.

## Run view motion planner (example)

Launch simulation:
```
roslaunch ur_with_cam_gazebo ur_with_cam.launch base:=trolley world_name:=world23
```

Run view motion planner:
```
rosrun view_motion_planner planner_node
```

Launch rviz:
```
roslaunch ur_with_cam_gazebo rviz.launch
```

Run rqt:
```
rqt
```

Select the ROI Viewpoint Planner plugin: Plugins -> Configuration -> ROI Viewpoint Planner

It should automatically switch to the view motion planner tab.

To run the complete pipeline, change "mode" to "PLAN_WITH_TROLLEY".

## Paper

If you use this work in your research, you can cite the following paper:

[Graph-based View Motion Planning for Fruit Detection](https://arxiv.org/pdf/2303.03048.pdf), accepted at IROS 2023

```bibtex
@inproceedings{zaenker2023graph,
	title={Graph-based View Motion Planning for Fruit Detection},
	author={Zaenker, Tobias and R{\"u}ckin, Julius and Menon, Rohit and Popovi{\'c}, Marija and Bennewitz, Maren},
	booktitle={Proc.~of the IEEE/RSJ Intl.~Conf.~on Intelligent Robots and Systems (IROS)},
	year={2023}
}
```

## Related repositories

[roi_viewpoint_planner](https://github.com/Eruvae/roi_viewpoint_planner): Our older Next-Best-View planner

[capsicum_superellipsoid_detector](https://github.com/salihmarangoz/capsicum_superellipsoid_detector.git): Capsicum shape completion, used for evaluation
