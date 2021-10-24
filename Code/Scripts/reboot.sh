#!/usr/bin/expect -f 
cd /home/lvuser/
spawn su -c "reboot" admin 
expect "Password:" 
send -- "\r" 
expect eof