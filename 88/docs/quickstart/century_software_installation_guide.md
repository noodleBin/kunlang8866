# Century Software Installation Guide

This document describes the steps required to install Century on Ubuntu 18.04.5
LTS (Bionic Beaver), the recommended Ubuntu release for Century 6.0.

## Pre-requisites

Before getting started, please make sure all the pre-requisite steps were
finished as described in the
[Pre-requisite Software Installation Guide](../specs/prerequisite_software_installation_guide.md).

Please also make sure Docker is running. Type `systemctl status docker` to check
the running status of Docker daemon, and type `systemctl start docker` to start
Docker if needed.

## Download Century Sources

Run the following commands to clone
[Century's GitHub Repo](https://github.com/CenturyAuto/century.git).

```bash
# Using SSH
git clone git@github.com:CenturyAuto/century.git

# Using HTTPS
git clone https://github.com/CenturyAuto/century.git

```

And checkout the latest branch:

```bash
cd century
git checkout master
```

For CN users, please refer to
[How to Clone Century Repository from China](../howto/how_to_clone_century_repo_from_china.md)
if your have difficulty cloning from GitHub.

(Optional) For convenience, you can set up environment variable
`CENTURY_ROOT_DIR` to refer to Century root directory by running:

```bash
echo "export CENTURY_ROOT_DIR=$(pwd)" >> ~/.bashrc  && source ~/.bashrc
```

![tip](images/tip_icon.png) In the following sections, we will refer to Century
root directory as `$CENTURY_ROOT_DIR`

## Start Century Development Docker Container

From the `${CENTURY_ROOT_DIR}` directory, type

```bash
bash docker/scripts/dev_start.sh
```

to start Century development Docker container.

If successful, you will see the following messages at the bottom of your screen:

```bash
[ OK ] Congratulations! You have successfully finished setting up Century Dev Environment.
[ OK ] To login into the newly created century_dev_michael container, please run the following command:
[ OK ]   bash docker/scripts/dev_into.sh
[ OK ] Enjoy!
```

## Enter Century Development Docker Container

Run the following command to login into the newly started container:

```bash
bash docker/scripts/dev_into.sh
```

## Build Century inside Container

From the `/century` directory inside Century Docker container, type:

```bash
./century.sh build
```

to build the whole Century project.

Or type

```bash
./century.sh build_opt
```

for an optimized build.

You can refer to
[Century Build and Test Explained](../specs/century_build_and_test_explained.md)
for a thorough understanding of Century builds and tests.

## Launch and Run Century

Please refer to the
[Run Century](../howto/how_to_launch_and_run_century.md#run-century) section of
[How to Launch And Run Century](../howto/how_to_launch_and_run_century.md).

## (Optional) Support a new Vehicle in DreamView

In order to support a new vehicle in DreamView, please follow the steps below:

1. Create a new folder for your vehicle under `modules/calibration/data`

2. There is already a sample file in the `modules/calibration/data` folder named
   `mkz_example`. Refer to this structure and include all necessary
   configuration files in the same file structure as “mkz_example”. Remember to
   update the configuration files with your own parameters if needed.

3. Restart DreamView and you will be able to see your new vehicle (name is the
   same as your newly created folder) in the selected vehicle.
