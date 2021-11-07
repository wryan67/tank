#!/bin/ksh

REMOTE=tank1

ssh $REMOTE bin/stop.sh
scp -p tank $REMOTE:~/bin
ssh $REMOTE bin/start.sh -f
