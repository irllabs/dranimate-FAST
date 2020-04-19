# Dranimate FAST Skinning Library

Leverages this [libigl implementation](https://github.com/libigl/libigl/blob/master/include/igl/arap_dof.h) of the [FAST paper](https://igl.ethz.ch/projects/fast/fast-automatic-skinning-transformations-siggraph-2012-jacobson-et-al.pdf).

### Prerequisites 

[CMake](https://cmake.org/) (use "cmake --version" to check existing install)  
[Emscripten](https://emscripten.org/index.html) (use "emcc" to check existing install)

### Install

```sh
git clone https://batchku@dev.azure.com/batchku/Dranimate/_git/Dranimate-FAST
cd Dranimate-FAST
git clone https://github.com/libigl/libigl.git
```

### Build 

```sh
./build.sh
```

### Update Web App

```sh
cd Dranimate-FAST
git commit -a -m "new version"
git push
```

```sh
cd Dranimate
npm update dranimate-fast
```
