if [ -e /dev/shm/fgscreen ]; then rm -f /dev/shm/fgscreen; fi && \
gst-launch-1.0 ximagesrc xname=FlightGear use-damage=false show-pointer=false ! \
videoscale ! video/x-raw,width=1920,height=1080,framerate=30/1 ! \
queue max-size-buffers=10 max-size-bytes=0 max-size-time=0 leaky=downstream ! \
tee name=t \
    t. ! queue ! videoconvert ! video/x-raw,format=BGR ! \
       shmsink socket-path=/dev/shm/fgscreen sync=false wait-for-connection=false \
    t. ! queue ! videoconvert ! \
       x264enc tune=zerolatency bitrate=4000 speed-preset=ultrafast ! \
       rtph264pay ! udpsink host=127.0.0.1 port=5600