What is this?
==

mc_vrep is a stand-alone executable used to control JRL/IDH robots (HRP-n) in a VREP simulation. The functionalities mimic that of the equivalent program used to control the actual robots so that going from the simulation to the robot requires a minimal effort (at least on the software side).

This only works with VREP >= 3.3.0. For older version please check the VREP<3.3.0 tag. However, this version is no longer maintained.

How does it work?
==

The infrastructure for this program is as follows:
Task <--> mc_rtc <--> mc_vrep <== vrep-api-wrapper ==> VREP

The communication with VREP is handled by [vrep-api-wrapper](https://gite.lirmm.fr/vrep-utils/vrep-api-wrapper) which is required by this project.

The application launched by the user is mc_vrep with an optional configuration file for the controller (which controller to use, environment to load, etc...).

The simulation is entirely driven by mc_vrep, with the following timeline for each time step of the simulation:
1. Get joints/sensors data from VREP
2. Compute the control
3. Send the control result to VREP

ATM (2016/08/29), it is possible to communicate with mc_vrep via a CLI but not more.

Current limitations (last update: 2016/08/29)
==

- the scene has to be loaded manually in VREP and mc_vrep assumes three things, warnings are emitted if these assumptions are not respected.
..1. The robot loaded in VREP is the one controlled by mc_vrep and the joints' names in VREP matches those in the URDF loaded by mc_rtc
..2. The robot's model in VREP has a base in VREP
..3. Idem for force sensors (can be fatal for mc_rtc if some assumptions are made about the force sensors' data)

How to build/run it?
==

**Note for Windows/MacOS build**

This has not been well tested yet. For MacOS it is recommended to install the dependencies through [Homebrew](http://brew.sh/).

mc_rtc
--

You need a working installation of mc_rtc to use mc_vrep. See the [COMANOID software release page](http://comanoid.cnrs.fr/packages/) for details on installation. This page provides instructions to install mc_rtc from binary packages (Ubuntu only) as well as from source.

Installation from source (Linux/MacOS)
----

Once you have cloned the `mc_rtc` repository. Go to the `utils` folder
located inside and find the file named `build_and_install.sh`. You can
then edit it to your liking (typically to change `INSTALL_PREFIX`,
`WITH_ROS_SUPPORT` and `ROS_DISTRO` variables) and simply run:

```
./build_and_install.sh
```

The script will take care of installing the required dependencies
(using `apt` on Linux and `brew` on MacOSX), clone all necessary source
codes, build and install them. This may take a while.

vrep-api-wrapper
--

You need to install vrep-api-wrapper by hand as VREP is not cleanly packaged on any platform. Please refer to the [project's page](https://gite.lirmm.fr/vrep-utils/vrep-api-wrapper) for detailed instructions on the installation.

mc_vrep
--

Once you installed both mc_rtc and vrep-api-wrapper you should be able to build and install mc_vrep easily.

The content of `$VREP_INSTALL_FOLDER/remoteApiCOnnections.txt` should look like this (any port is ok of course as long as you reflect the change in `mc_vrep.cpp`):
```
portIndex1_port = 19997
portIndex1_debug = false
portIndex1_syncSimTrigger = true
```

Open an environment containing the HRP-n robot you wish to simulate.

Finally, run mc_vrep:
* `./src/mc_vrep ../etc/mc_vrep.conf` from the build folder for example

By default this will start the simulation with a very simple posture controller, you can check that it is working properly by typing commands into the CLI, for example with HRP4:
* `set_joint_pos NECK_P 0.5` will move the head downwards

To change the robot you are controlling, simply change the `MainRobot` entry in
the configuration file you provide to the `mc_vrep` executable (e.g. HRP2DRC or
HRP4).

Finally, to stop the simulation, you should type `stop` into the CLI.
