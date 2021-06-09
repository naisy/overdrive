#!/bin/bash
XSOCK=/tmp/.X11-unix
XAUTH_FILE=.Xauthority
HOST_USER=$(getent passwd 1000 | cut -d: -f1)
HOST_USER_GROUP=$(getent group 1000 | cut -d: -f1)
HOST_USER_HOME=/home/$HOST_USER
HOST_USER_XAUTH=$HOST_USER_HOME/$XAUTH_FILE
HOST_MOUNT_PATH=$HOST_USER_HOME/data
DOCKER_USER=ubuntu
DOCKER_USER_HOME=/home/$DOCKER_USER
DOCKER_USER_XAUTH=$DOCKER_USER_HOME/$XAUTH_FILE
DOCKER_MOUNT_PATH=$DOCKER_USER_HOME/data

if [ ! -f $HOST_USER_HOME/$XAUTH_FILE ]; then
    touch $HOST_USER_HOME/$XAUTH_FILE
    chown $HOST_USER:$HOST_USER_GROUP $HOST_USER_HOME/$XAUTH_FILE
    chmod 600 $HOST_USER_HOME/$XAUTH_FILE
    su $HOST_USER -c "xauth generate :0 . trusted"
fi

if [ ! -d "$HOST_MOUNT_PATH" ]; then
    mkdir $HOST_MOUNT_PATH
    chown $HOST_USER:$HOST_USER_GROUP $HOST_MOUNT_PATH
fi

IMG=naisy/donkeycar-pc:overdrive4

docker run \
    --runtime=nvidia \
    --shm-size 8G \
    -it \
    -v $XSOCK:$XSOCK \
    -v $HOST_USER_XAUTH:$DOCKER_USER_XAUTH \
    -v $HOST_MOUNT_PATH:$DOCKER_MOUNT_PATH \
    -e DISPLAY \
    -e QT_GRAPHICSSYSTEM=native \
    -e QT_X11_NO_MITSHM=1 \
    --mount type=bind,source=/run/user/1000/,target=/run/user/1000/,readonly \
    --mount type=bind,source=/var/run/dbus/system_bus_socket,target=/var/run/dbus/system_bus_socket,readonly \
    -v /etc/localtime:/etc/localtime:ro \
    -v /dev/:/dev/ \
    -u $DOCKER_USER \
    --privileged \
    --network=host \
$IMG
