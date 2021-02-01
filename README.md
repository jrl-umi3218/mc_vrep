mc_vrep is an interface to use [mc_rtc] controllers in [CoppeliaSim].

This works with all versions of [CoppeliaSim] >= 3.3.0 as of this writing.

Overview
==

The communication with [CoppeliaSim] is handled by [vrep-api-wrapper](https://gite.lirmm.fr/vrep-utils/vrep-api-wrapper) which is required by this project.

The application launched by the user is mc_vrep with an optional configuration file for the controller (which controller to use, environment to load, etc...).

The simulation is entirely driven by mc_vrep, with the following timeline for each time step of the simulation:
1. Get joints/sensors data from [CoppeliaSim]
2. Compute the control
3. Send the control result to [CoppeliaSim]

ATM (2016/08/29), it is possible to communicate with mc_vrep via a CLI but not more.

Current limitations (last update: 2016/08/29)
==

The scene has to be loaded manually in [CoppeliaSim] and mc_vrep assumes three things, warnings are emitted if these assumptions are not respected:

1. The robot controlled by mc_vrep is loaded in the scene and the joints' names in [CoppeliaSim] match those in the URDF loaded by [mc_rtc];
2. The robot's model in [CoppeliaSim] has a base;
3. The force sensors' names in [CoppeliaSim] match the force sensors' names in [mc_rtc];

Installation guide
==

mc_rtc
--

Follow [mc_rtc installation guide](https://jrl-umi3218.github.io/mc_rtc/tutorials/introduction/installation-guide.html) for details on installation.

vrep-api-wrapper
--

Please refer to the [project's page](https://gite.lirmm.fr/vrep-utils/vrep-api-wrapper) for detailed instructions on the installation. You will have to install [CoppeliaSim] by yourself.

mc_vrep
--

Once you have mc_rtc and vrep-api-wrapper on your system you can build and install mc_vrep easily, using cmake:

```bash
mkdir build
cd build
cmake ../ -DCMAKE_BUILD_TYPE=RelWithDebInfo
make
sudo make install
```

Usage
==

1. Load a [CoppeliaSim] simulation with the robot you wish to control
2. Configure mc_rtc accordingly (controller and robot selection)
3. Run the `mc_vrep` command

Inside mc_rtc GUI you can interact with the simulation:
- stop it;
- enable/disable step by step mode;
- when step by step mode is enabled you can also step multiple times into the simulation;

Running with multiple robots
==

mc_vrep supports multiple robots, to enable it add the following entry to your `mc_rtc.yaml` configuration:

```yaml
VREP:
  Extras:
    - index: 1
      suffix: "#0"
    - index: 2
      suffix: "#1"
```

This will tell mc_vrep that robots with index 1 and 2 in your controller are also present in the [CoppeliaSim] scene and can be controlled. Furthermore, the suffix is used by [CoppeliaSim] when multiple robots of the same type are inserted into the scene. In such cases, [CoppeliaSim] will append this suffix to every joints' and links' name in the simulated model so we need to tell mc_vrep what the suffix is.

Simulation settings
==

The `VREP` section accepts the following parameters related to the simulation:

- `SimulationTimestep`: timestep of the physics engine in the V-REP simulation; by default, `mc_vrep` assumes this is equal to the controller timestep;
- `StepByStep`: starts the simulation in step by step mode;
- `VelocityControl`: use the velocity output of mc_rtc to control the robots (default: `false`);
- `TorqueControl`: use the torque output of mc_rtc to control the robots (default: `false`);

When both `VelocityControl` and `TorqueControl` are false (i.e. in default mode) the robots are controlled via the position output of mc_rtc.


Connection options
==

The `VREP` section accepts the following parameters, they correspond to the default remote API setting for [CoppeliaSim]:

- `Host`: which host to connect to for [CoppeliaSim] remote API (default: `localhost`)
- `Port`: which port to connect to for [CoppeliaSim] remote API (default: `19997`)
- `Timeout`: timeout when attempting the connection in milliseconds (default: `3000`)
- `WaitUntilConnected`: wait until the API is connected (default: `true`)
- `DoNotReconnect`: do not attempt to reconnect if the connection is lost (default: `true`)
- `CommThreadCycleInMs`: Remote API thread cycle duration in ms (default: `1`, must be `> 0`)

[mc_rtc]: https://jrl-umi3218.github.io/mc_rtc/
[CoppeliaSim]: https://www.coppeliarobotics.com/
[V-REP]: https://www.coppeliarobotics.com/
