#Python based Robot Interactive Development Environment (PyRIDE) for NAO

##Introduction
This repository contains PyRIDE for Aldebaran NAO robot. For more detailed introduction on PyRIDE, see [README](https://github.com/uts-magic-lab/pyride_pr2/blob/master/README.md) of PyRIDE for ROS/PR2. **WARNING** This version of PyRIDE is compatible with NaoQi SDK version 1.14.5 and below. It requires code updates to be compatible with later version of NaoQi SDK.

##Compile source code
###Prerequisites
You need NaoQi SDK 1.14.5 with the corresponding cross compiler toolchain installed on your system. You will also need qibuild 1.14.5 build system installed.

###Source code structure
PyRIDE for NAO is dependent on several opensourced third-party libraries. Since there is no prebuild library binaries, this repository contains modified source code of these libraries so that you can build and install them manually on your NAO robots. In addition, PyRIDE on NAO has been partitioned into two components: PyRideCore and PyNaoServer. PyRideCore contains core functionalities that are available to all supported robot platforms. PyNaoServer contains code that is specific to the NAO robot platform. Third-party libraries and PyRideCore are under ```libsrc``` directory. PyNaoServer is located under ```PyNaoServer``` directory. ```scripts``` contains default example Python scripts that run on PyRIDE for NAO.

###Compile procedures
Assume you have placed PyRIDE for NAO source code under a working qibuidl worktree and you have a working cross compiler toolchain ```cross-geode```, use the following command to build libraries using qibuild build system.

```
qibuild --package -c cross-geode --release
```

To install the compiled library package, e.g. commoncpp2, on local machine:

```
qitoolchain add-package -c cross-geode commoncpp2 ../../package/commoncpp2-cross-geode.tar.gz
```

You will need to compile code in the following sequence:
* commoncpp2-1.8.1
* celt-0.11.1
* ccrtp-1.7.2
* PyRideCore
* PyNaoServer


