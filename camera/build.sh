cp -rf ./lib ./build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make