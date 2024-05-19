docker run -it \
  -v /home/ben/Mira/Xavier/xavier_ws:/root/catkin_ws \
  --net=host \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  xavier-mira:latest bash
