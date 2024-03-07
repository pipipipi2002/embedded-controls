# Firmware
Capabilities
- Library support for 
    - 9-Axis IMU BNO08X
    - 6-Axis IMU ICM42688P
    - 6-Axis IMU ICM40609D
- Outputs data through UART (connected to the USB-C)

# Usage
```bash
$ make all      # to build 
$ make all -j8  # build with 8 worker cores
$ make stflash  # to flash
$ make clean    # to delete build outputs
```

## Dependencies
- Make and CMake
- GNU ARM toolchain
- Vscode
- OpenOCD [(install guide)](https://forum.pedalpcb.com/threads/setting-up-vscode-openocd-xpack-st-link-for-debugging-on-macos-big-sur.4861/)
- SVD file (already provided but you can get it from [here](https://github.com/cmsis-svd/cmsis-svd-data))

# VS-code 
## Setting Up
Download the necessaries extensions
- Cortex-Debug
- C/C++

Navigate to `.vscode` and edit the following files
- `c_cpp_properties.json`
    - Edit `macFrameworkPath` to your own environment
- `launch.json`
    - `configFiles`: based on the OpenOCD xpack install path
- `settings.json`
    - `cortex-debug.armToolchainPath`: to your GCC ARM toolchain path
    - `C_Cpp.default.compilerPath`: to your GCC ARM toolchain path

## Tasks
Under build group, there are 3 preconfigured tasks, same as in the Usage Section above.
Press `Ctrl+Shift+B` to bring up the vscode build panel and choose the desired action.

## Debug 
Under Debug, there are 2 preconfigured launches, as defined in `vscode/launch.json`. 
- Debug (with build)
- Attach
