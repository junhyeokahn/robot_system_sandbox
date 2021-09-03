#! /bin/bash
PATH_PACKAGE="$(pwd)"

echo '# ==============================================================='
echo "# install pinocchio"
echo '# ==============================================================='

if [ "$(uname)" == "Darwin" ]; then
    echo "[error] not implemented yet"
elif [ "$(expr substr $(uname -s) 1 5)" == "Linux" ]; then
    sudo apt install -qqy lsb-release gnupg2 curl &&
    echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list &&
    curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add - &&
    sudo apt-get update &&
    sudo apt install -qqy robotpkg-pinocchio &&
    echo 'export PATH=/opt/openrobots/bin:$PATH' >> ~/.bashrc &&
    echo 'export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH' >> ~/.bashrc &&
    echo 'export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH' >> ~/.bashrc &&
    echo 'export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH' >> ~/.bashrc 
else
    echo "[error] os not detected"
fi
