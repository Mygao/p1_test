gst-launch-1.0 -v tcpclientsrc host=1.1.20.193 port=5000 ! gdpdepay ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false
