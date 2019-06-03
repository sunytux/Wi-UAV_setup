IP_UP2="192.168.0.210"
USER_UP2="beams"
PWD_UP2="azer"
REMOTE_DIR="/home/beams/Desktop/main"
LOCAL_DIR="$(dirname "$(readlink -f "$0")")"

case "$1" in
    "make" )
        cd "${LOCAL_DIR}/build" && \
        make \
        ;;

    "build" )
        if [ -d "${LOCAL_DIR}/build" ]; then
            rm -r "${LOCAL_DIR}/build"
        fi
        mkdir "${LOCAL_DIR}/build"
        cd "${LOCAL_DIR}/build"
        cmake ../
        ;;

    "push")            
        sshpass -p $PWD_UP2 \
            rsync -av $LOCAL_DIR/* "$USER_UP2@$IP_UP2:$REMOTE_DIR" \
                --progress \
                --delete-after\
                --exclude 'build'\
        ;;

    "ssh-build" )
        ./$0 push # The script call itself to push the code on remote
        sshpass -p $PWD_UP2 \
            ssh $USER_UP2@$IP_UP2 "cd ${REMOTE_DIR} && ./run.sh build"
        ;;

    "ssh-make" )
        ./$0 push # The script call itself to push the code on remote
        sshpass -p $PWD_UP2 \
            ssh $USER_UP2@$IP_UP2 "cd ${REMOTE_DIR} && ./run.sh make"
        ;;


    *) echo >&2 "Invalid option: $@"; exit 1;;
esac
