#!/bin/ksh
set -a

typeset -l STATE=`echo $1 | cut -c1`

PowerPin=a1
FirePin=a0

usage() {
  echo "usage: turret.sh [on|off|charge|fire]"
}

case $STATE in
  -) usage;;
  h) usage;;
  o) echo turn on/off
      mcp23x17 -xr > /dev/null
      mcp23x17 -$FirePin  -w 1 > /dev/null
      mcp23x17 -$PowerPin -w 1 > /dev/null
      ;;
  c) echo charge
      mcp23x17 -$FirePin  -w 1 > /dev/null
      mcp23x17 -$PowerPin -w 0 > /dev/null
      ;;	  
  f) echo fire in the hole
      mcp23x17 -$PowerPin -w 1 > /dev/null
      sleep 0.1
      mcp23x17 -$FirePin -w 0 > /dev/null
      sleep 0.2
      mcp23x17 -$FirePin -w 1 > /dev/null
      ;;	  
  *) echo turn off
      mcp23x17 -$FirePin  -w 1 > /dev/null
      mcp23x17 -$PowerPin -w 1 > /dev/null
      ;;
esac
