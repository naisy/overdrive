#!/bin/bash
XSOCK=/tmp/.X11-unix
XAUTH_FILE=.Xauthority
HOST_USER=$(getent passwd 1000 | cut -d: -f1)
HOST_USER_GROUP=$(getent group 1000 | cut -d: -f1)
HOST_USER_HOME=/home/$HOST_USER
HOST_USER_XAUTH=$HOST_USER_HOME/$XAUTH_FILE
HOST_MOUNT_PATH=$HOST_USER_HOME/data
DOCKER_USER=jetson
DOCKER_USER_HOME=/home/$DOCKER_USER
DOCKER_USER_XAUTH=$DOCKER_USER_HOME/$XAUTH_FILE
DOCKER_MOUNT_PATH=/home/$DOCKER_USER/data
CSI_CAMERA=/tmp/argus_socket
CAMSOCK=/tmp/camsock
NVSCSOCK=/tmp/nvscsock

########################################
# make .Xauthority
########################################
if [ ! -f $HOST_USER_HOME/$XAUTH_FILE ]; then
    touch $HOST_USER_HOME/$XAUTH_FILE
    chown $HOST_USER:$HOST_USER_GROUP $HOST_USER_HOME/$XAUTH_FILE
    chmod 600 $HOST_USER_HOME/$XAUTH_FILE
    DISPLAYNAME=`echo $DISPLAY`
    if [ -z $DISPLAYNAME ]; then
	DISPLAYNAME=:0
    fi

    su $HOST_USER -c "xauth generate $DISPLAYNAME . trusted"
fi

########################################
# make ~/data/ directory
########################################
if [ ! -d "$HOST_MOUNT_PATH" ]; then
    mkdir $HOST_MOUNT_PATH
    chown $HOST_USER:$HOST_USER_GROUP $HOST_MOUNT_PATH
fi

########################################
# docker image
########################################
IMG=naisy/donkeycar-jetson:overdrive4

# https://docs.docker.com/storage/bind-mounts/
docker run \
    --runtime=nvidia \
    --memory=4g \
    --memory-swap=-1 \
    -it \
    --mount type=bind,source=$XSOCK,target=$XSOCK \
    --mount type=bind,source=$HOST_USER_XAUTH,target=$DOCKER_USER_XAUTH \
    --mount type=bind,source=$HOST_MOUNT_PATH,target=$DOCKER_MOUNT_PATH \
    -e DISPLAY \
    -e QT_GRAPHICSSYSTEM=native \
    -e QT_X11_NO_MITSHM=1 \
    --mount type=bind,source=/run/user/1000/,target=/run/user/1000/,readonly \
    --mount type=bind,source=/var/run/dbus/system_bus_socket,target=/var/run/dbus/system_bus_socket,readonly \
    --mount type=bind,source=/etc/localtime,target=/etc/localtime,readonly \
    --mount type=bind,source=$CSI_CAMERA,target=$CSI_CAMERA \
    --mount type=bind,source=$CAMSOCK,target=$CAMSOCK \
    --mount type=bind,source=$NVSCSOCK,target=$NVSCSOCK \
    --mount type=bind,source=/dev/,target=/dev/ \
    -u $DOCKER_USER \
    --privileged \
    --network=host \
$IMG
