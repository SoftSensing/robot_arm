### Data Acquisition using the Franka Emika Robot Arm

The usage of the code provided is based on the [Franka Emika Robot Arm](https://robodk.com/robot/Franka/Emika-Panda) and libfranka, which can be installed by following the instructions [here](https://frankaemika.github.io/docs/libfranka.html).

After installing libfranka, clone this repo directly by running:
```
git clone https://github.com/SoftSensing/robot_arm.git
cd robot_arm
```

Alternatibely, one can directly download the `multisine_acquisition.cpp` in the directory `/path/to/libfranka/examples` of the libfranka installation.

Buildindg and running the code for moving the Robot Arm according to a trajectory descried by a multisine wave can be achieved by:
```
cd /path/to/libfranka/examples
cmake -DCMAKE_BULD_TYPE=Release .
cd examples
make ..
./multisine_acquisition <robot_arm_ip>
```
