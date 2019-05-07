#!/bin/bash
#
# Install development prerequisites for source distributions of Maliput

me='The Maliput source distribution prerequisite setup script'

die () {
    echo "$@" 1>&2
    trap : EXIT  # Disable line number reporting; the "$@" message is enough.
    exit 1
}

at_exit () {
    echo "${me} has experienced an error on line ${LINENO}" \
        "while running the command ${BASH_COMMAND}"
}

trap at_exit EXIT

[[ "${EUID}" -eq 0 ]] || die "${me} must run as root. Please use sudo."

if [ "$(lsb_release -c | grep -e bionic | wc -l)" -eq 1 ]; then
DISTRO=crystal
else
DISTRO=bouncy
fi

sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable bionic main" > /etc/apt/sources.list.d/gazebo-stable.list'
apt-key adv --keyserver hkp://p80.pool.sks-keyservers.net:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

apt update

echo ""
echo "***********************************************************************************"
echo "* Deb Installs"
echo "***********************************************************************************"
echo ""

apt install pybind11-dev

echo ""
echo "***********************************************************************************"
echo "* Debs (Build Tools)"
echo "***********************************************************************************"
echo ""

# python-vcstool || python3-vcstool
apt install python3-vcstool

if [ "$(dpkg -l | grep -c -E 'rosdep')" -eq 0 ]; then
    echo ">>> Installing rosdep"
    apt install --no-install-recommends -y python3-rosdep
    rosdep init
fi

apt install --no-install-recommends -y $(tr '\n' ' ' <<EOF
ros-$DISTRO-ament*
libyaml-cpp-dev
EOF
)



echo ""
echo "***********************************************************************************"
echo "* Success"
echo "***********************************************************************************"
echo ""

trap : EXIT  # Disable exit reporting.
echo "Finished installation"
echo ""
