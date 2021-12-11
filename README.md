# navigation

[![Build Status](https://github.com/rustyducks/navigation/actions/workflows/cpp_linux_x86.yml/badge.svg)](https://github.com/rustyducks/navigation/actions/workflows/cpp_linux_x86.yml)

## Installation

### Install dependencies

Please note that manually installing the dependencies is optional and should be automagically performed, provided with an Internet connection, via CMake *FetchContent*. However, for crosscompiling, they need to be build and installed manually.

- [Rustyducks' **geometry_tools**](https://github.com/rustyducks/geometry_tools)

### Build and install

```bash
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=</path/to/your/workspace> ..  # To build with tests: cmake -DENABLE_TESTS=ON ..
make -j4
make install
ctest -V  # To run tests
```

### Crosscompiling for Linux arm (e.g. Raspberry Pi with Raspbian)

```bash
mkdir buildarm && cd buildarm
export LINUX_ARM_TOOLCHAIN_PATH=</path/to/toolchain>  # optional: defaults to /usr/lib/ccache
cmake -DCROSSCOMPILE_ARM=ON -DCMAKE_INSTALL_PREFIX=</path/to/your/workspace/armlinux> ..  # your workspace must contain the dependencies, manually built for the host
make -j4
make install
```
