# General FAQs

## I am new to the Century project, where do I start?

You have several options:

- To build century on your computer, start by reviewing
  [Century Software Installation Guide](../quickstart/century_software_installation_guide.md)

- To run the Century demo offline, go to:
  [Century README.md](../demo_guide/README.md).

- To install and build Century on a vehicle, go to:
  [Century 1.0 quick start](../quickstart/century_1_0_quick_start.md).

- To build the Century Kernel, the Robot Operating System (ROS), and Century, go
  to:
  [century/docs/quickstart/century_1_0_quick_start_developer.md](../quickstart/century_1_0_quick_start_developer.md)
  and refer to
  [build kernel](../quickstart/century_1_0_quick_start_developer.md#build-the-century-kernel).

---

## How do I send a pull request?

Sending a pull request is simple.

1. Fork the Century Repository into your GitHub.
2. Create a Developer Branch in your Repository.
3. Commit your change in your Developer Branch.
4. Send the pull request from your GitHub Repository Webpage.

---

## Do comments need to be made in Doxygen?

Yes, currently all comments need to be made in Doxygen.

---

## How to debug build problems?

1. Carefully review the instructions in the documentation for the option that
   you selected to get started with the Century project.

2. Make sure that you follow the steps in the document exactly as they are
   written.

3. Use Ubuntu 14.04 as the build can only be implemented using Linux.

4. Verify that the Internet setting is correct on your computer.

5. Allocate more than 1GB of memory, at the recommended minimum, for your
   computer.

6. If roscore cannot start in century docker, you may need to tune the master
   start timeout value in ROS. You may want to check a related user-reported
   [issue](https://github.com/CenturyAuto/century/issues/2500) for more details.

---

## If I cannot solve my build problems, what is the most effective way to ask for help?

Many build problems are related to the environment settings.

1. Run the script to get your environment: `bash scripts/env.sh >& env.txt`

2. Post the content of env.txt to our Github issues page and someone from our
   team will get in touch with you.

---

## Which ports must be whitelisted to run Century in a public cloud instance?

Use these ports for HMI and Dreamview:

- 8888: Dreamview

---

## Why there is no ROS environment in dev docker?

The ROS package is downloaded when you start to build century:
`bash century.sh build`.

1. Run the following command inside Docker to set up the ROS environment after
   the build is complete: `source /century/scripts/century_base.sh`

2. Run ROS-related commands such as rosbag, rostopic and so on.

---

## How do I clean the existing build output?

Follow these steps:

1. Log into Docker using the command: `bash docker/scripts/dev_into.sh`

2. Run the command: `bash century.sh clean`

---

## How do I delete the downloaded third party dependent packages?

Follow these steps:

1. Log into Docker using the command: `bash docker/scripts/dev_into.sh`

2. Run the command: `bazel clean --expunge` The build command,
   `bash century.sh build`, then downloads all of the dependent packages
   according to the _WORKSPACE_ file.

**More General FAQs to follow.**
