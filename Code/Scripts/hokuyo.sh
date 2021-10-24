#!/usr/bin/expect -f 
cd /home/lvuser/
spawn su -c "rmmod cdc-acm" admin 
expect "Password:" 
send -- "\r" 
expect eof