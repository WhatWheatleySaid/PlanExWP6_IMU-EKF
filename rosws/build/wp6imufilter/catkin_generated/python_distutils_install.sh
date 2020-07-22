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

echo_and_run cd "/home/lee/Documents/git/planex-wp6/rosws/src/wp6imufilter"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/lee/Documents/git/planex-wp6/rosws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/lee/Documents/git/planex-wp6/rosws/install/lib/python2.7/dist-packages:/home/lee/Documents/git/planex-wp6/rosws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/lee/Documents/git/planex-wp6/rosws/build" \
    "/usr/bin/python2" \
    "/home/lee/Documents/git/planex-wp6/rosws/src/wp6imufilter/setup.py" \
     \
    build --build-base "/home/lee/Documents/git/planex-wp6/rosws/build/wp6imufilter" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/lee/Documents/git/planex-wp6/rosws/install" --install-scripts="/home/lee/Documents/git/planex-wp6/rosws/install/bin"
