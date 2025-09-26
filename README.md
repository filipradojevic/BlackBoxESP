# BlackBox

**Documentation**

Detailed documentation about the MAVLink FTP implementation can be found here:


📄 [View FTP Documentation (PDF)](docs/FileTransferProtocol_Mavlink.pdf)



This project is intended to be used as a starting point. Make sure to clone
repository using ***--recursive*** flag
```
git clone --branch BlackBox https://github.com/filipradojevic/FTP.git BlackBox
```

If you already have cloned the branch use:
```
git pull origin BlackBox
```

Project relies on multiple generators during the build stage, such as mavgen and
ulog-gen, which are included as submodules. Because of these generators, which
are written in python, it is strongly recommended to create dedicated python
virtual environment. Creating python virtual environment named ***.venv*** can
be done by running:

- Linux
	```
	python3 -m venv .venv
	```

- Windows
	```
	python -m venv .venv
	```

There is no strict naming convention for naming virtual environment, but the
most often used names are ***.venv***, ***venv*** and ***python_venv***.
These often used names are added to ***.gitignore***.

Activating python virtual environment can be done using:

- Linux
	```
	source .venv/bin/activate
	```

- Windows
	```
	.venv\Scripts\activate
	```

Make sure that all the projects requirements are installed in virtual
environment (such as MAVLink and ULog generator requirements)

**Pymavlink for FTP**


First upgrade your pip install in python environment 

- Linux
	```
	python3 -m pip install --upgrade pip setuptools wheel
	```

- Windows
	```
	python -m pip install --upgrade pip setuptools wheel
	```

Then install pymavlink:

- Linux/Windows
	```
	pip install pymavlink
	```

**MAVLink Generator**:

- Linux
	```
	python3 -m pip install -r src/middleware/mav/mavlink/pymavlink/requirements.txt
	```

- Windows
	```
	python -m pip install -r src/middleware/mav/mavlink/pymavlink/requirements.txt
	```

**ULog Generator**:

- Linux
	```
	python3 -m pip install -r src/middleware/ulog/ulog-gen/requirements.txt
	```

- Windows
	```
	python -m pip install -r src/middleware/ulog/ulog-gen/requirements.txt
	```

After installing packages required by project virtual environment can be
deactivated using.
```
deactivate
```

Virtual environment is not needed to be activated again, and will be called by
CMake during the build stage.

To BUILD this project now press:
```
F7
```

To CLEAN-REBUILD press:
```
F3
```

To FLASH the program press:
```
F5
```

## Tool Configuration

In order to compile project it is needed to configure needed tools, instructions
can be found on QNAP or YouTube.

- [Windows Instructions](https://youtu.be/mzDSuTes94s?si=mTRIQjb0yGjn8cFB)

- [Linux Instructions](https://youtu.be/_yG40rGTXko?si=Ls3UtnsF5oRLxzyv)

### Cortex-Debug

Cortex-Debug VS Code plugin requires configuration of launch.json file, more
information can be found [here](https://go.microsoft.com/fwlink/?linkid=830387).

Example of launch.json for ULink2 (OpenOCD) and JLink is given below

```json
{
	"version": "0.2.0",
	"configurations": [
		{
			"name": "Debug (ULINK2)",
			"type": "cortex-debug",
			"request": "launch",
			"servertype": "openocd",
			"cwd": "${workspaceFolder}",
			"device": "LPC1768",
			"configFiles": ["${workspaceFolder}/devices/lpc1768/tools/openocd.cfg"],
			"executable": "${workspaceFolder}/build/program.elf",
			"svdFile": "${workspaceFolder}/devices/lpc1768/tools/LPC1768.svd",
			"runToEntryPoint": "main",
			"interface": "swd",
			"showDevDebugOutput": "raw",
			"liveWatch": {
				"enabled": true,
				"samplesPerSecond": 2
			}
		},
		{
			"name": "Debug (JLink)",
			"type": "cortex-debug",
			"request": "launch",
			"servertype": "jlink",
			"cwd": "${workspaceFolder}",
			"device": "LPC1768",
			"executable": "build/program.elf",
			"svdFile": "${workspaceFolder}/devices/lpc1768/tools/LPC1768.svd",
			"runToEntryPoint": "main",
			"interface": "swd",
			"showDevDebugOutput": "raw",
			"liveWatch": {
				"enabled": true,
				"samplesPerSecond": 2
			}
		}

	]
}
```

## Building

To build the project:
1. Choose preset (depending on optimization level)
2. Build using F7

In case of changes in configuration files make sure to use Clean Rebuild.

## Adding preset

Use `CMakePresets.json` as a reference.

To add user preset:
1. Create `CMakeUserPresets.json` file, must be located in project root.
2. For each preset specify:
	- ***CMAKE_BUILD_TYPE***: CMake build type.