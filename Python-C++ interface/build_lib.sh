mkdir build
cd build
cmake ../marker
make CycloneVision -j4
cp libCycloneVision.so ../libCycloneVision.so