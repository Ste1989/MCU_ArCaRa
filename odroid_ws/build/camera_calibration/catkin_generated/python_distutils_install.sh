#!/bin/sh -x

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

cd "/home/ste/Odroid_ArCaRa/odroid_ws/src/camera_calibration"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
/usr/bin/env \
    PYTHONPATH="/home/ste/Odroid_ArCaRa/odroid_ws/install/lib/python2.7/dist-packages:/home/ste/Odroid_ArCaRa/odroid_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ste/Odroid_ArCaRa/odroid_ws/build" \
    "/usr/bin/python" \
    "/home/ste/Odroid_ArCaRa/odroid_ws/src/camera_calibration/setup.py" \
    build --build-base "/home/ste/Odroid_ArCaRa/odroid_ws/build/camera_calibration" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/ste/Odroid_ArCaRa/odroid_ws/install" --install-scripts="/home/ste/Odroid_ArCaRa/odroid_ws/install/bin"
