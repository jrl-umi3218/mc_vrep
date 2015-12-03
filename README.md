What is this?
==

mc_vrep is a stand-alone executable used to control JRL/IDH robots (HRP-n) in a VREP simulation. The functionalities mimic that of the equivalent program used to control the actual robots so that going from the simulation to the robot requires a minimal effort (at least on the software side).

How does it work?
==

The infrastructure for this program is as follows:
Task <--> mc_rtc <--> mc_vrep <====> VREP

The communication with VREP is done through the remote API and specifically using the [synchronous operation mode] (http://www.coppeliarobotics.com/helpFiles/en/remoteApiModusOperandi.htm#synchronous)

It requires to [extend the remote API](http://www.coppeliarobotics.com/helpFiles/en/remoteApiExtension.htm) but information about this process will be given later.

The application launched by the user is mc_vrep with an optional configuration file for the controller (which controller to use, environment to load, etc...).

The simulation is entirely driven by mc_vrep, with the following timeline for each time step of the simulation:
1. Get joints/sensors data from VREP (2 remote API calls)
2. Compute the control (no remote API calls)
3. Send the control result to VREP (1 custom remote API call)

ATM (03/12/2015), it is possible to communicate with mc_vrep via a CLI but not more.

Current limitations (last update: 03/12/2015)
==

- only HRP2-kai is supported, some (minor) work has to be done in mc_rtc and mc_vrep to support all robots
- the scene has to be loaded manually in VREP and mc_vrep assumes two things, warnings are emitted if these assumptions are not respected.
..1. The robot loaded in VREP is the one controlled by mc_vrep and the joints' names in VREP matches those in the URDF loaded by mc_rtc
..2. Idem for force sensors (can be fatal for mc_rtc if some assumptions are made about the force sensors' data)

How to build/run it? (WIP)
==

**Note for Windows/MacOS build**

This has not been well tested yet. For MacOS it is recommended to install the dependencies through [Homebrew](http://brew.sh/).

mc_non_ros/Tasks
--

The first step is to install the `mc_non_ros` package as explained [here](https://gite.lirmm.fr/multi-contact/installation_wiki/wikis/home)

Note that the python bindings are optional as well as anything ROS-related.

Once you have installed `mc_non_ros` you will need an up-to-date version of the Tasks library that can be found [here](https://github.com/gergondet/Tasks)

mc_rbdyn_urdf_cpp
--

`mc_rbdyn_urdf_cpp` has the following dependencies:
* [tinyxml2](http://www.grinninglizard.com/tinyxml2/) or `libtinyxml2-dev` on debian-like systems

Then clone from https://gite.lirmm.fr/multi-contact/mc_rbdyn_urdf_cpp and build (cmake/make/make install)

mc_rtc
--

`mc_rtc` has the following dependencies:
* [jsoncpp](https://github.com/open-source-parsers/jsoncpp) or `libjsoncpp-dev` on debian-like systems
* [libgeos](https://github.com/libgeos/libgeos) or `libgeos++-dev` on debian-like systems

Then clone from https://gite.lirmm.fr/multi-contact/mc_rtc/ and build (cmake/make/make install)

**Building without ROS support**

If you are building `mc_rtc` without ROS support then you need to tell him where it can fetch the models for HRP2-kai and the environment. This is done by defining:
* `-DHRP2_DESCRIPTION_PATH=$PATH` where `$PATH` is the location of the `hrp2_drc_description` folder from the [hrp2_drc repository](https://gite.lirmm.fr/mc-hrp2/hrp2_drc)
* `-DMC_ENV_DESCRIPTION_PATH=$PATH` where `$PATH` is the location of the `mc_env_description` folder from the [mc_ros repository](https://gite.lirmm.fr/multi-contact/mc_ros)

mc_vrep
--

We will assume that you installed VREP in the `$VREP_INSTALL_FOLDER`.

Clone the [mc_vrep repository](https://gite.lirmm.fr/multi-contact/mc_vrep) and build it (cmake/make)

Put the `simxCustomCmd.cpp` file located in the repository in `$VREP_INSTALL_FOLDER/programming/v_repExtRemoteApi`, then `cd` to this folder, rebuild the `v_repExtRemoteApi` plugin (`make` on Linux/MacOS or using the provided Visual Studio solution for Windows) and copy the resulting library (`lib/libv_repExtRemoteApi.so`) in `$VREP_INSTALL_FOLDER` (overwriting the existing one).

The content of `$VREP_INSTALL_FOLDER/remoteApiCOnnections.txt` should look like this (any port is ok of course as long as you reflect the change in `mc_vrep.cpp`):
```portIndex1_port = 4242
portIndex1_debug = false
portIndex1_syncSimTrigger = true
```

Open an environment containing the HRP-n robot you wish to simulate.

Finally, run mc_vrep:
* `./src/mc_vrep ../etc/mc_vrep.conf` from the build folder for example

By default this will start the simulation with a very simple posture controller, you can check that it is working properly by typing commands into the CLI, for example with HRP2-kai:
* `set_joint_pos HEAD_JOINT_1 0.5` will move the head downwards

Finally, to stop the simulation, you should type `stop` into the CLI.
