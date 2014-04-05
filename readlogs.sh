#syslog -w -F '$((Time)(sec)) $Message' -k Host S Basicmotion > live_log.log &
#echo "Time,Roll0,xRoll0,Pitch0,xPitch0,Roll1,xRoll1,Pitch1,xPitch1,Pitch2,xPitch2,Roll2,xRoll2" > live_log.log;
syslog -w 0 -F '$Message' -k Host S Basicmotion | grep "," >> live_log.log &

