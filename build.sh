echo Cleaning up...
rm -rf build

echo Creating build directory...
mkdir build
cd build

echo Running cmake...
#cmake .. -DCMAKE_BUILD_TYPE=Release -DPLATFORM=32 -DCMAKE_C_FLAGS=-m32 -DCMAKE_CXX_FLAGS=-m32
cmake .. -DCMAKE_BUILD_TYPE=Release
echo
echo Running make...
make -j4

echo
echo Finished! If there are no errors, you may copy the driver folder in the build directory to your steam vr drivers directory, or register the driver from the current folder.
