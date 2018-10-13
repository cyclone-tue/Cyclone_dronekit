mkdir build
cd build
cmake  --target=CycloneVision ../marker
make CycloneVision -j4
cp libCycloneVision.so ../libCycloneVision.so