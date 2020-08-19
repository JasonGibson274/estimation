## Dependencies
Currently GTSAM but the goal is to work on the cmake so each individual
estimators' dependencies won't mix (An EKF estimator doesn't need to rely on
GTSAM)

#### GTSAM
we are using a [fork](https://github.gatech.edu/icarus/gtsam) of the develop branch of gtsam currently because it is what we got to work.
```bash
git clone https://github.gatech.edu/icarus/gtsam.git
cd gtsam
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
