# Installation on Ubuntu

## Prerequisites

- Ubuntu 20.04 with compatible ROS version
- Python >= 3.7
- gcc & g++ >= 5.4
- OpenCV 3 or 4
- Anaconda for configuring python dependencies

## Clone the repository & install catkin dependencies

First create and navigate to your catkin workspace

``` shell
cd <your-working-directory>
mkdir <your-ros-ws>/src && cd <your-ros-ws>
```

Then, initialize the workspace and configure it. (Remember to replace <your-ros-version> by your ros version)

``` shell
catkin init
catkin config --extend /opt/ros/<your-ros-version> --merge-devel 
catkin config --cmake-args -DCMAKE_CXX_STANDARD=14 -DCMAKE_BUILD_TYPE=Release
```

Download this repository to your ROS workspace `src/` folder with submodules via:

``` shell
cd src
git clone https://github.com/TooSchoolForCool/Part-Mesh-Reconstruction.git
```

## Install python dependencies

We assume using [conda virtual environment](https://conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html#activating-an-environment) to configure python dependencies. This requires [Anaconda](https://www.anaconda.com/products/individual) to be installed and initialized as prerequisite. We create a conda env with `python3.7` and install the dependencies:

``` shell
conda create --name part-level-scene-recon python=3.7 -y
conda activate part-level-scene-recon
pip install pip --upgrade
# install the dependencies 
cd src/Part-Mesh-Reconstruction
pip install -r requirements.txt
```

You can deactivate the conda env using:
``` shell
conda deactivate
```


## Build packages

We first build the python package needed to convert the parts into CAD models.
``` shell
conda activate part-level-scene-recon
cd part2cad
make dev
```

Then we build the ros packages with `catkin build`.

``` shell
cd <your-ros-ws>
catkin build scene_builder
source devel/setup.bash
```

Please replace `bash` by `zsh` if `zsh` is your default shell.