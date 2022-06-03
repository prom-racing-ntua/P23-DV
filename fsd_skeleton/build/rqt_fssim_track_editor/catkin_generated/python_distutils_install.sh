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

echo_and_run cd "/home/andva/PromRacing/fsd_skeleton/src/fssim/fssim_rqt_plugins/rqt_fssim_track_editor"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/andva/PromRacing/fsd_skeleton/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/andva/PromRacing/fsd_skeleton/install/lib/python3/dist-packages:/home/andva/PromRacing/fsd_skeleton/build/rqt_fssim_track_editor/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/andva/PromRacing/fsd_skeleton/build/rqt_fssim_track_editor" \
    "/home/andva/anaconda3/bin/python3" \
    "/home/andva/PromRacing/fsd_skeleton/src/fssim/fssim_rqt_plugins/rqt_fssim_track_editor/setup.py" \
     \
    build --build-base "/home/andva/PromRacing/fsd_skeleton/build/rqt_fssim_track_editor" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/andva/PromRacing/fsd_skeleton/install" --install-scripts="/home/andva/PromRacing/fsd_skeleton/install/bin"
