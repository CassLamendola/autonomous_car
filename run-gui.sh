sudo docker run -it \
    -p 127.0.0.1:4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    capstone-1.1 


# Above, we made the container's processes interactive, forwarded our DISPLAY environment variable, mounted a volume for the X11 unix socket. 
# For more info, visit:  http://wiki.ros.org/docker/Tutorials/GUI

