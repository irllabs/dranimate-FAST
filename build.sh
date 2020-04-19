# WASM build
mkdir -p build
cd build
emcmake cmake ..
emmake make
cp dranimate-fast.js ../dist/web
cp dranimate-fast.wasm ../dist/web

# Native build for reference
# mkdir -p build
# cd build
# cmake ..
# make
