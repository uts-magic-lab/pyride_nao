# Python based Robot Interactive Development Environment (PyRIDE) for NAO

## Introduction
This repository contains PyRIDE for Aldebaran NAO robots. For more detailed introduction on PyRIDE, see [README](https://github.com/uts-magic-lab/pyride_pr2/blob/master/README.md) of PyRIDE for ROS/PR2.

**WARNING:** This version of PyRIDE is compatible with NaoQi SDK version 2.4.3 and below. It requires code updates to be compatible with later version of NaoQi SDK.

## Compile source code
### Prerequisites
You need NaoQi SDK 2.4.3 with the corresponding cross compiler toolchain installed on your system. You will also need qibuild 3.11.x build system installed.

### Source code structure
PyRIDE for NAO is dependent on several open source third-party libraries. Since there is no prebuild library binaries, this repository contains modified source code of these libraries so that you can build and install them manually on your NAO robots. In addition, PyRIDE on NAO has been partitioned into two components: PyRideCore and PyNaoServer. PyRideCore contains core functionalities that are available to all supported robot platforms. PyNaoServer contains code that is specific to the NAO robot platform. Third-party libraries and PyRideCore are under ```libsrc``` directory. PyNaoServer is located under ```PyNaoServer``` directory. ```scripts``` contains default example Python scripts that run on PyRIDE for NAO.

### Compile procedures
Assume you have placed PyRIDE for NAO source code under a working qibuild worktree and you have a working cross compiler toolchain ```cross-atom```, use the following command to build libraries using qibuild build system.

```
qibuild package -c cross-atom --release
```

To install the compiled library package, e.g. commoncpp2, on local machine:

```
qitoolchain add-package -c cross-atom ../../../package/commoncpp2-cross-atom.zip
```

You will need to compile code in the following sequence:
* crypto (openssl)
* commoncpp2-1.8.1
* celt-0.11.1
* ccrtp-1.7.2
* pyridecore
* PyNaoServer

### Manual binary installation
Under a Pepper terminal (using ssh), copy the following shared library objects extracted from the packages (or copy directly from the ```build-cross-atom-release/sdk/lib``` subdirectory under the source directories) built in the previous section to ```/home/nao/naoqi/lib``` directory on the robot.

* libcrypto.so
* libccext2.so
* libccrtp.so
* libccgnu2.so
* libcelt.so
* libpyridecore.so
* libpynaoserver.so

Create an ```autoload.ini``` file under ```/home/nao/naoqi/preference``` with the following content:

```
# autoload.ini
#
# Use this file to list the cross-compiled modules that you wish to load.
# You must specify the full path to the module, python module or program.

[user]
#the/full/path/to/your/liblibraryname.so  # load liblibraryname.so
/home/nao/naoqi/lib/libpynaoserver.so

[python]
#the/full/path/to/your/python_module.py   # load python_module.py

[program]
#the/full/path/to/your/program            # load program
```

```autoload.ini``` will automatically load PyNaoServer when NaoQi starts.

**NOTE:** PyRIDE configuration file ```pyrideconfig.xml``` will be automatically generated under ```/home/nao/naoqi/preference``` when PyRIDE for NAO is successfully run and *properly* shutdown. As older NaoQi systems do not properly shutdown PyRIDE module when shutting down the robot by pressing the centre button, you may have to call ```PyNAO.saveConfiguration``` command to save important configuration, e.g. remote user access account, periodically.

### Using PyRIDE for NAO
Check PyRIDE for ROS/PR2 [README](https://github.com/uts-magic-lab/pyride_pr2/blob/master/README.md) for the details on how to access embedded Python engine, remote client access. Check [PyRIDE API documentation for NAO](http://uts-magic-lab.github.io/pyride_nao) for the details on the available Python methods for NAO.
