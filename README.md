build:
mkdir build && cd build
cmake ..
make

run:
GLOG_logtostderr=1 ./sort/main