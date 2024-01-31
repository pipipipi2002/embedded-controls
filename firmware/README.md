# Usage
```bash
$ make all      # to build 
$ make stflash  # to flash
$ make clean    # to delete build outputs
```

## Dependencies
- Make
- CMake
- GNU ARM toolchain
- Vscode

# VS-code 
## Tasks
Under build group, there are 3 preconfigured tasks, same as in the Usage Section above.
Press `Ctrl+Shift+B` to bring up the vscode build panel and choose the desired action.

## Debug 
Under Debug, there are 2 preconfigured launches, as defined in `vscode/launch.json`. 
- Debug (with build)
- Another to flash
