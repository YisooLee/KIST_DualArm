
## Prerequisities
> This project was developed in Ubuntu 18.04.
> Realtime controller is developed on Xenomai, Linux.

## Requirements

#### Eigen3

#### RBDL
```sh
git clone --recursive https://github.com/ORB-HD/rbdl-orb
cd rbdl-orb
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -DRBDL_BUILD_ADDON_URDFREADER=ON ..
make all
make install
```

#### qpOASES