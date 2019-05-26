# TODOs:
* Figure out how to automaticaly not build estimators if their dependencies
don't exist on the system.
* perhaps use stable release of GTSAM/Figure out how to keep track of which
commit we are on.
* Clean up cmake in general
* Figure out if we can properly link to built libraries
* Create install commands in cmake
* Make cmake configuration arguments (BUILD_EKF for example)
* Create .deb packages for each individual estimator to ease installations

# State estimation

Estimators to be implemented:
* Factor Graph based on SLAM
* Particle Filter based on SLAM
* EKF based on SLAM

## Dependencies
Currently GTSAM but the goal is to work on the cmake so each individual
estimators' dependencies won't mix (An EKF estimator doesn't need to rely on
GTSAM)

#### GTSAM
we are using the develop branch of gtsam currently because it is what we got to work.
```bash
git clone
cd gtsam
git checkout 357e739
mkdir build && cd build
cmake ..
make -j8 # the -j command refers to how many CPU cores to use while compiling

```
## Build instructions
```bash
mkdir build
cd build
cmake ..
make -j8 # the -j command refers to how many CPU cores to use while compiling
```
