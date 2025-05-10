# tiago_spnav_teleop

TIAGo (and TIAGo++) arm teleoperation with 3Dconnexion's SpaceMouse.

## Dependencies

This project depends on the [spacenav_node](https://wiki.ros.org/spacenav_node) ROS package and daemon service. Make sure you have installed them prior to launching the tiago_spnav_teleop controller:

```
sudo apt install ros-${ROS_DISTRO}-spacenav-node spacenavd
```

Once the daemon is loaded, a blue light should be seen on the SpaceMouse when plugged in.

## Running in simulation

We recommend Docker. It is assumed that you have cloned this repository inside your current working directory.

### For single-arm TIAGo

```
docker run -it --privileged --network host -e DISPLAY \
           -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
           -v $XAUTHORITY:/root/.Xauthority:ro \
           -v /dev/bus/usb:/dev/bus/usb \
           -v $PWD/tiago_spnav_teleop:/tiago_public_ws/src/tiago_spnav_teleop \
       palroboticssl/tiago_tutorials:noetic
```

Then (assuming you have run `catkin build tiago_spnav_teleop`):

```
roslaunch tiago_spnav_teleop tiago_gazebo.launch
```

### For dual-arm TIAGo++

```
docker run -it --privileged --network host -e DISPLAY \
           -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
           -v $XAUTHORITY:/root/.Xauthority:ro \
           -v /dev/bus/usb:/dev/bus/usb \
           -v $PWD/tiago_spnav_teleop:/tiago_dual_public_ws/src/tiago_spnav_teleop \
       palroboticssl/tiago_dual_tutorials:noetic
```

Then (assuming you have run `catkin build tiago_spnav_teleop`):

```
roslaunch tiago_spnav_teleop tiago_dual_gazebo.launch arm:=right
```

Alternatively, use `arm:=left` or `arm:=both` to achieve the obvious result.

## Running on the real robot

Upload the project to your active workspace and build it using the `deploy.py` script (but without deploying it yet):

```
rosrun pal_deploy deploy.py -u pal -p tiago_spnav_teleop $(hostname)
```

Press `n` when asked to rsync, then copy the command displayed at the bottom, replace `pal` with `root`, and run it. Make sure to reboot the robot at least once in order to let ROS find the plugin library at the next start.

### Instantiating the 3D mouse controller

On an external PC connected to the same WiFi (check that `ROS_IP` points at your IP inside said network):

```
export ROS_MASTER_URI=http://10.68.0.1:11311
export ROS_IP=$(hostname -I | awk '{print $1}')
roslaunch tiago_spnav_teleop spnav_mouse.launch
```

Append `__ns:=/spnav_controller_right` or `__ns:=/spnav_controller_right` to the latter command if using TIAGo++. Launch it twice (using different `left`/`right` namespaces) in case you have two devices and want to use both arms.

### For single-arm TIAGo

After you `ssh` into the robot:

```
roslaunch tiago_spnav_teleop tiago_real.launch
```

### For dual-arm TIAGo++

After you `ssh` into the robot:

```
roslaunch tiago_spnav_teleop tiago_dual_real.launch arm:=right
```

Alternatively, use `arm:=left` or `arm:=both` to achieve the obvious result.

## Important notes and post-demo actions

- Upon initialization, the robot will extend the requested arm. Make sure its path is free of obstacles.
- The FreeSpacenav implementation that powers ROS 3D mouse controllers doesn't allow communicating with more than a single device on the same machine. This means that, if you wish to bimanipulate TIAGo++ with two devices, you will need to issue `spnav_mouse.launch` on different machines (one of them could be the robot itself as long as you `apt install spacenavd ros-$(echo $ROS_DISTRO)-spacenav-node` and plug the device in one of its external USB ports), assign different ROS namespaces (e.g. `__ns:==/spnav_controller_right`) and check communications (`rostopic echo /spnav_controller_right/spacenav/joy`).
- If you want to restore the default controllers after closing this application, run `rosrun tiago_spnav_teleop stop_spnav.py`. Append `--arm left`, `--arm right` or `--arm both` correspondingly if using TIAGo++.

## Citation

If you found this project useful, please consider citing the following work:

Calzada García, A., Łukawski, B., Victores, J., & Balaguer, C. (2024). Teleoperation of the robot TIAGo with a 3D mouse controller. In Actas del Simposio de Robótica, Bioingeniería y Visión por Computador (pp. 133–138). Universidad de Extremadura.

```bibtex
@inproceedings{calzada2024srbv,
    author={Calzada García, Ana and Łukawski, Bartek and Victores, Juan G. and Balaguer, Carlos},
    title={{Teleoperation of the robot TIAGo with a 3D mouse controller}},
    booktitle={Actas del Simposio de Robótica, Bioingeniería y Visión por Computador},
    year={2024},
    pages={133--138},
    publisher={Universidad de Extremadura},
    url={http://hdl.handle.net/10662/21260},
}
```

## See also

- [demo application (YouTube)](https://youtu.be/3DYZG5At-Gw)
- [roboticslab-uc3m/tiago_telepresence_controllers](https://github.com/roboticslab-uc3m/tiago_telepresence_controllers)
