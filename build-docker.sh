docker build -t rosnoetic-focal:base -f Dockerfile.focal-base .
docker build -t rosnoetic-focal:minimal -f Dockerfile.focal-minimal .
docker build -t rosnoetic-focal:v1.0 -f Dockerfile.focal-packages .
docker save -o rosnoetic-focal-v1.0.tar rosnoetic-focal:v1.0
