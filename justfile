fb: fresh-build

fresh-build:
  rm -rf build && mkdir build && cd build && cmake .. && make
