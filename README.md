ashesh-hri1-controller
==

This package contains the skeleton of a new controller implementation for HRP2-kai Hand Trajectories (at present only RArm). 
several human trajectroies were recorded using Motion Analysis tool and others were computed in Matlab. This controller is 
designed for upcoming HRI experiment.  

## What does the controller do?

This controller plays several trajectories at different speeds, fastest trajectory takes 0.4sec and slowest takes 1sec.
there are total of 4 different types of trajectories,
28 are human hand moment- recorded using motion capture with 4 subjects (2 each male/female)
7 are min_jerk velocity profile
14 are trapezoidal velocity profile 


## How to use 

run VREP simulation 
use combination of suffix and prefix
/*suffix; f= fast; m= medium; s= slow, m2f= med2fast, m12= med1med2, sm1= slowmed1*/
/*prefix; mj= min-jerk; s1, s2 s3, s4= subjects; i= industrial-trap; t= trap velocity*/
send_msg  mj_s
send_msg mj_ts
send_msg s1_s

Setup
--

Assuming `mc_rtc` is already setup on your system then simply follow the usual method:

```
git submodule update --init
mkdir build
cd build
cmake ../
make
sudo make install # sudo optionnal depending on mc_rtc installation
```

Using this project as a starting point
--

*Mandatory*

A few mandatory things have to be changed in order to use this project as a
starting point:

1. Project name in `CMakeLists.txt`
2. Controller name in the `SIMPLE_CONTROLLER_CONSTRUCTOR` macro in `src/mc_sample_controller.h`

*Optional (but recommended)*

1. The class name (may create unwanted collusions otherwise)
2. The file names

--
more information will be added soon 