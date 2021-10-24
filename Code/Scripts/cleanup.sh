#!/usr/bin/expect -f 
cd /home/lvuser/
spawn su -c "rm qr.txt route1.csv route2.csv cubes3Colors.txt cubes4Colors.txt cubes3Flag.txt labview.log" admin 
expect "Password:" 
send -- "\r" 
expect eof