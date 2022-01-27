#!/bin/ksh

ps -fu root | egrep "tank|rainbow" | egrep -v "grep|vnc|/bin/sh|sudo" | awk '{system(sprintf("sudo kill %d",$2))}'
sleep 0.2
ps -fu root | egrep "tank|rainbow" | egrep -v "grep|vnc|/bin/sh|sudo" | awk '{system(sprintf("sudo kill -9 %d",$2))}'

# Output Relay Control
PowerPin=a1
FirePin=a0

mcp23x17 -xr > /dev/null 2>&1
mcp23x17 -$FirePin  -w 1 > /dev/null
mcp23x17 -$PowerPin -w 1 > /dev/null

