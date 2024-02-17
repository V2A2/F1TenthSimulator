# F1TenthSimulator
Gazebo based simulation for F1Tenth

Simulation requres ros noetic and gazebo 11

http://wiki.ros.org/noetic/Installation/Ubuntu

https://classic.gazebosim.org/tutorials?tut=install_ubuntu

Download the latest LibTorch version

https://pytorch.org/get-started/locally/

Build OpenCV 4 from source (Ideally with cuda support)

https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html

Additions to .bashrc

export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/home/[Path to libtorch install]/share/cmake/Torch:/home/[Path to Custom OpenCV Build]/build

export GAZEBO_MODEL_PATH=/home/[Location of Catkin Workspace]/src/new_f1tenth_simulator

Additionally make sure your path for cuda is set if you are having build issues

export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}

export LD_LIBRARY_PATH="/usr/local/cuda/lib64:/usr/local/cuda-11.7/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
