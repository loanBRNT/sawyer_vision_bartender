#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/loan/sawyer_vision_bartender/src/intera_sdk/intera_interface"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/loan/sawyer_vision_bartender/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/loan/sawyer_vision_bartender/install/lib/python3/dist-packages:/home/loan/sawyer_vision_bartender/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/loan/sawyer_vision_bartender/build" \
    "/usr/bin/python3" \
    "/home/loan/sawyer_vision_bartender/src/intera_sdk/intera_interface/setup.py" \
    egg_info --egg-base /home/loan/sawyer_vision_bartender/build/intera_sdk/intera_interface \
    build --build-base "/home/loan/sawyer_vision_bartender/build/intera_sdk/intera_interface" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/loan/sawyer_vision_bartender/install" --install-scripts="/home/loan/sawyer_vision_bartender/install/bin"
