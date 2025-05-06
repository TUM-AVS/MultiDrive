# MultiDrive: A Co-Simulation Framework Bridging 2D and 3D Driving Simulation for AV Software Validation

This repository contains the implementation of our approach, **MultiDrive**, to integrate low- and high-fidelity
simulators for validating AV software.

Our approach extends the [Multi-agent Scenario Handler](https://github.com/TUM-AVS/Frenetix-Motion-Planner),
which uses [CommonRoad](https://commonroad.in.tum.de/) as simulation framework and [FrenetiX](https://github.com/TUM-AVS/Frenetix-Motion-Planner) as motion planner, to allow simulations in the 
[BeamNG.tech](https://beamng.tech/) high-fidelity simulator.

Thanks to BeamNG.tech's ability to procedurally generate virtual worlds, MultiDrive can automatically
generate arbitrary (CommonRoad) scenarios and run them seamlessly in low- and high-fidelity.
In turn, this ability enables developers using MultiDrive to assess the robustness of their design
time assumptions.

## Installation

### Requirements

MultiDrive has the following main requirements:

- Windows (at least version 10). This is required for running BeamNG.tech.
- Ubuntu running on WLS2 (version 22.04.5 LTS.). This is required to install FrenetiX and Make.
- Python (version 3.10). This is required for our implementation of MultiDrive.
- BeamNG.tech (at least version 0.32.4.0). This is required for running high-level simulations.
- GNU Make (version 4.3). This is required to implement the automation pipeline
- [GifSki](https://gif.ski/) (version 1.32.0). This is required to create high-quality gifs.
- FrenetiX. This is required for the motion planner under test and the basic simulation setup.

Additional python libraries are listed in the `requirements.txt` file.

### BeamNG.tech Installation

BeamNG does not require to be installed. It is enough to download the distribution and unzip it on your disk.

#### Register and Download
To download BeamNG.tech, you must apply for a license at [https://register.beamng.tech/](https://register.beamng.tech/).
License is free for non-commercial usage of the simulator and will be delivered to you by email with download instructions.

Assuming you got the registration email, download the distribution (see Requirements) and unztip it under a path that 
does **NOT** contain any blank space or special character.
For example, a path like `C://BeamNG//BeamNG.tech.v0.32.4.0` is a suitable target for installing BeamNG.tech version `0.32.4.0`.

> IMPORTANT: We will refer to the folder containing the BeamNG.tech distribution as `<BEAMNG_HOME_FOLDER>`

For reference, this folder should contain the following files and folders:

```
.
├── Bin64
├── BinLinux
├── EULA.pdf
├── PrivacyPolicy-tech.pdf
├── PrivacyPolicy.pdf
├── beamng.log
├── campaigns
├── content
├── flowgraphEditor
├── gameengine.zip
├── gameplay
├── icon-beamng.ico
├── integrity.json
├── licenses.txt
├── locales
├── lua
├── renderer
├── replays
├── settings
├── shaders
├── startup.default.ini
├── startup.ini
├── support.exe
├── tech
├── tech.key
├── temp
├── thirdpartyFilter.ini
├── thirdpartyFilter_cef.ini
├── trackEditor
└── ui
```

> NOTE: BeamNG is a large download at ~24gb. For failures due to timeout or spotty connections, consider a download manager to resume failed FTP file transfers.

#### Install the licence key

Copy the `tech.key` file that you received with the registration email inside `<BEAMNG_HOME_FOLDER>`
Otherwise, BeamNG will complain this file is missing and not start.

#### Setup User Folder

BeamNG.tech uses a folder for each user to cache data, store all the generated scenarios, and so on.
To avoid using the default location (i.e., user home), create a new folder "`<BEAMNG_HOME_FOLDER>`_user".
For instance, `C://BeamNG//BeamNG.tech.v0.32.4.0_user`

> IMPORTANT: This folder will be referred as the `<BEAMNG_USER_FOLDER>`.

> NOTE: You can always delete the user folder to clean up your system.

#### Patch the LUA files
Currently, BeamNG does not provide out of the box a value for steering angle. So we need to patch it such that it can be computed using
the solution mentioned at [https://github.com/BeamNG/BeamNGpy/issues/224](https://github.com/BeamNG/BeamNGpy/issues/224).

To do so, you need to copy the two files in the folder `beamng-patches` inside the folder `<BEAMNG_HOME_FOLDER>/lua/vehicle/`.
The patched `electrics.py` simply exports a variable called `wheel_angle`. This can be retrieved from python using the Electrics sensor.
The patched `wheels.py` implements the function that computes the steering angle and stores it inside `electrics.values.wheel_angle`.

> NOTE: We will provide automated script in the future

#### Patch the materials file
Currently, the material of the roads that MultiDrive uses are not present in the defaul BeamNG.tech level (`tech_ground`).

So, we need to manually patch the level to render the roads correctly.
If you skip this step, the simulations will still run but you'll see an orange texture saying "material not found" on the screen.
This is not really a problem for FrenetiX, as it does not use vision for the moment;
however, if you plan to include vision-based sensors, you need to fix this.

Patching consists in updating a file, called `main.materials.json`, inside the `tech_ground.zip` file located inside
`<BEAMNG_HOME_FOLDER>\content\levels\` by copying some content from a similar file in the `west_coast_usa.zip` file located in 
the same folder.

Open the file `<BEAMNG_HOME_FOLDER>\content\levels\west_coast_usa.zip\levels\west_coast_usa\art\road\main.materials.json`
Copy the entries related to `"road_invisible"` and `"road_asphalt_light"` that are the names of the materials we use in our project.

Update the file `<BEAMNG_HOME_FOLDER>\content\levels\tech_ground.zip\levels\tech_ground\art\road\main.materials.json`.
To do so, you need to:

1. Copy the `main.materials.json` file outside the `tech_ground.zip` into your disk.
2. Create a backup copy of the file `main.materials.json.bkp`
2. Edit the `main.materials.json` in your local disk by adding the entries of the materials you copies before. Pay attention to the commas!
3. Copy the `main.materials.json` and `main.materials.json.bkp` files inside the `tech_ground.zip` and select "Copy & Replace" when asked, thus overwriting the original `main.materials.json` file.

> NOTE: We will provide automated script in the future.

#### BeamNG.ini

This project requires some configurations to work.
You can provide them as follows:

From the `<PROJECT_HOME>` go inside the `cr_beamng_cosimulation` folder:

```bash
cd cr_beamng_cosimulation
```

Rename the `beamng.ini-example` file to `beamng.ini`:

```bash
mv beamng.ini-example beamng.ini
```

Edit the `beamng.ini` file providing the missing options (indicated with the `!!FILL ME!!`).
Specifically, `home_folder` must be the same as `<BEAMNG_HOME_FOLDER>`, 
and `user_folder` must be the same as `<BEAMNG_USER_FOLDER>`.

The `host` must be the IP address of the WSL2 installation that can be retrieved by running
the following in the WSL2 bash terminal:

```bash
ip route | grep default | awk '{print $3}'
```

For the moment, you can leave the other parametes as they are.

> NOTE: The code assume that the `beamng.ini` file in inside `cr_beamng_cosimulation` folder. So do not move it from there!

## Python Setup (in WSL)

We assume that MultiDrive runs in a WSL2 Linux Ubuntu using Python 3.10.

We refer to the folder containing MultiDrive as `<PROJECT_HOME>`.

Inside `<PROJECT_HOME>`, checkout the Frenetix-Motion-Planner as git submodule:

```
git submodule init
```

Expect something like:

```
Submodule 'Frenetix-Motion-Planner' (https://github.com/TUM-AVS/Frenetix-Motion-Planner.git) registered for path 'Frenetix-Motion-Planner'
```

Pull the submodule:

```
git submodule update --remote
```

Expect something like:

```
Cloning into './cr-beamng-cosimulation/Frenetix-Motion-Planner'...
Submodule path 'Frenetix-Motion-Planner': checked out 'fc6e7708ba384f991c2a0a07b7300ab52276c70f'
```

Create a virtual env named `.venv`

```
python -m venv .venv
```

Activate the virtual env `.venv`

```
. .venv/bin/activate
```

All the followig commands assume that the `.venv` is active. 
> NOTE: You can see that the vend is active by looking at the shell showing `(.venv)`

Update `pip`:

```
pip install --upgrade pip
```

Install Frenetix from the Frenetix-Motion-Planner folder:

```
cd Frenetix-Motion-Planner
pip install -e .
cd ..
```

Install the python deps from the `requirements.txt` file:

```
pip install -r requirements.txt
```

Some of the components install a version of SciPy that is incompatible with FrenetiX.  So we need to force the expected version of the library:

```
pip install scipy==1.13.0
```

## Smoke test the installation

The main entry point of MultiDrive is the `cli.py` script which implements
the command line interface.

To list the available commands (with the `.venv` active) run:

```
python cli.py --help
```
Expect something like:

```
Usage: cli.py [OPTIONS] COMMAND [ARGS]...

Options:
  --output-folder PATH
  --scenario-name PATH
  --verbose / --no-verbose  Activate verbose debugging on console
  --help                    Show this message and exit.

Commands:
  animate-simulation      Combine the frames into an high-quality gif
  compare                 Compute the difference of the aligned...
  cosimulate-with-beamng  Simulate the CommonRoad scenario using...
  plot-simulation         Plot all the frames of the executed scenario
  simulate                Simulate the CommonRoad scenario using CommonRoad
```

Each command can be invoked with the `--help` option to list all the paramters and options. Our automation pipeline wraps those commands to ease their usage, but each command can be invoked manually from the command line.

The `simulate` command simulates the input CommonRoad scenario in low-fidelity in CommonRaod,
whereas the `cosimulate-with-beamng` command simulates the input CommonRoad scenario in high-fidelity in BeamNG.tech.
The `compare` command computes and plots error metrics from corresponding low- and high-fidelity simulations.
The `plot-simulation` creates a png for each time-step for low- or high-fidelity simulations,
whereas the `animate-simulation` creates a gif out of the pngs created using the previous command.

# Test Generation

We exemplify test generation by implementing a Grid search over two parameters in a turn scenario.
The scenario include a straight initial segment, a turn, and a final straight segment.
The turn is configured using `turn_angle` and `radius`.
The code to generate scenarios covering the two parameters space can be found in: `search/example.py`

# Automation Pipeline

To automate the execution, analysis, and visualization of scenarios and simulations, we rely on Make a widely used and well known utility.
Our make file searches (recursively) inside the `scenarios` folder for possible scenarios (`.xml` files).
For each scenario, it creates several targets that invoke the `cli.py` commands. Targets may depend on one another, and Make manages them consistently.

Targets follow the pattern:
- simulate-<SCENARIO_NAME>: simulates the CommonRoad scenario in CommonRoad and stores the results in `experiments-car/<SCENARIO_NAME>/0001`
- cosimulate_with_bng-<SCENARIO_NAME>: simulates the CommonRoad scenario in BeamNG and stores the results in `experiments-car/<SCENARIO_NAME>/0002`
- plot-simulated-<SCENARIO_NAME>: generates the plots visualizing the scenario low-fidelity execution at every time step and stores the plots in `experiments-car/<SCENARIO_NAME>/simulated-plots`
- plot-cosimulated_with_bng-<SCENARIO_NAME>: generates the plots visualizing the scenario high-fidelity execution at every time step and stores the plots in `experiments-car/<SCENARIO_NAME>/cosimulated_with_bng-plots`
- animate-simulated-<SCENARIO_NAME>: creates the `experiments-car/<SCENARIO_NAME>/simulated-scenario.gif` from the output of the `plot-simulated` command
- animate-cosimulated_with_bng-<SCENARIO_NAME>: creates the `experiments-car/<SCENARIO_NAME>/cosimulated_with_bng-scenario.gif` from the output of the `plot-cosimulated_with_bng` command
- animate-<SCENARIO_NAME>: merge the plots generated from the previous commands and creates the comparative `experiments-car/<SCENARIO_NAME>/scenario.gif`
- analyze-<SCENARIO_NAME>: loads the low- and high-fidelity simulation data, aligns the trajectory for each vehicle, and computes the error metrics (position, rotation, speed, etc.). Results are the `simulated_vs_cosimulated_with_bng.csv` and the plots inside the `plots` folder within `experiments-car/<SCENARIO_NAME>`

For instance, the following command causes the scenario `single-agent.xml` to be simulated, co-simulated, plotted, and animated:

```
make single-agent
```

> NOTE: Make ensures that commands already executed will not be executed again.