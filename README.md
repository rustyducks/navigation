# navigation

[![Build Status](https://github.com/rustyducks/navigation/actions/workflows/cpp_linux_x86.yml/badge.svg)](https://github.com/rustyducks/navigation/actions/workflows/cpp_linux_x86.yml)

## Dependencies

- Eigen3

### Install dependencies on Debian based

```bash
sudo apt update && sudo apt install libeigen3-dev ivy-c-dev
```

## Build

```bash
mkdir build && cd build
cmake ..  # To build with tests: cmake -DENABLE_TESTS=ON ..
make -j8
ctest -V  # To run tests
```

### Crosscompiling for Linux arm (e.g. Raspberry Pi with Raspbian)

```bash
mkdir buildarm && cd buildarm
export LINUX_ARM_TOOLCHAIN_PATH=</path/to/toolchain>  # defaults to /usr/lib/ccache
cmake -DCROSSCOMPILE_ARM=ON ..
make -j8
```
