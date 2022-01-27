#!/bin/ksh

REMOTE=localhost

ssh $REMOTE bin/stop.sh
scp -p tank $REMOTE:~/bin
ssh $REMOTE bin/start.sh -f
