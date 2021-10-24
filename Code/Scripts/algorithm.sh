#!/usr/bin/expect -f 
spawn su -c /home/lvuser/algorithm admin 
expect "Password:" 
send -- "\r" 
expect eof