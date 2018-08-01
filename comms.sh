sudo chmod 666 /dev/ttyACM*
sudo stty -F /dev/ttyACM* 4000000 raw -clocal -echo
