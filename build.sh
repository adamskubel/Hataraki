PATH=$PATH:/usr/local/carlson-minot/crosscompilers/bin
make -j4
scp BasicMotion root@192.168.2.15:~/motion2
./copyconfig.sh

