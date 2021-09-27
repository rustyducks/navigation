# navigation
[![Build Status](https://github.com/rustyducks/navigation/actions/workflows/cpp_linux_x86.yml/badge.svg)](https://github.com/rustyducks/navigation/actions/workflows/cpp_linux_x86.yml)

## Build

```bash
mkdir build && cd build
cmake ..
make -j8
```

### Crosscompiling for Linux arm (e.g. Raspberry Pi with Raspbian)

```bash
mkdir buildarm && cd buildarm
export LINUX_ARM_TOOLCHAIN_PATH=</path/to/toolchain>  # defaults to /usr/lib/ccache
cmake -DCROSSCOMPILE_ARM=ON ..
make -j8
```
