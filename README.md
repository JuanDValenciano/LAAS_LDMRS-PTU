# LAAS_LDMRS-PTU
A ROS driver for 3D scanning based on SICK LD-MRS and FLIR PTU-46

## Prerequisites
* Python
* C
* ROS

## Getting Started
* Install [RobotoPKG](http://robotpkg.openrobots.org/install.html)
* After run ./bootstrap:
```
% cd robotpkg/architecture/genom3
% make update
% make
% make install
```
```
% cd robotpkg/interfaces/openrobots-idl
% make update
% make
% make install
```

* Setup (bash version) with $ROS_VERSION, your ros version :

```
% source /opt/ros/$ROS_VERSION/setup.bash
% export INSTALL_PATH=/{YOUR_INSTALL_PATH}
% export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$INSTALL_PATH/lib/pkgconfig
% export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$INSTALL_PATH/src/ros-nodes:$INSTALL_PATH/share
% export PYTHONPATH=/opt/ros/groovy/lib/python2.7/dist-packages:$INSTALL_PATH/lib/python2.7/site-packages
% echo "setup_ros ok"
```

* Clone the project and install manually the modules in your workspace: platine_light and LiDARldmrs:
```
% autoreconf -vi
% mkdir build && cd build
% ../configure --prefix=${YOUR_INSTALL_PATH} --with-templates=ros/server,ros/client/c,ros/client/ros
% make
% make install
```
* Edit the file main.py to change the path_data and path_bin

## Running the tests
* Launch roscore
* In bin folder run the modules:
```
% ./platine_light
% ./LiDARldmrs
```
* In 3D_sensor folder:
```
% python main.py
```
* Configure the scan parameters
The pointCloud data is saved in .pkl file

## Built With
[GenoM3](https://www.openrobots.org/wiki/genom3)

## Authors
* Harold F Murcia - *Initial work from:*  Christophe Reymann and  Matthieu Herrb; LAAS-CNRS, [RIS-TEAM](https://www.laas.fr/public/en)
