# Details
This folder contains the generated code from CubeMX (not the one inside STM32CubeIDE). The CMakeLists.txt defines the files to included/compiled, as an interface library, to be used by the top level CMakeLists.txt. Thus it does not act as a "portable" module. The reason is to make it more readable by decoupling the source/include files from potential future folders on the same directory level as Core (eg. Project).