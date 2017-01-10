#! /bin/sh

echo "Building KLGD-FF..."
cd ./plugin
make || exit 1

echo "Building KLGD Testing module"
cd ../testmod
make
