#!/bin/ksh

f=$1

turretPin=0
analysisPin=3

pca9685 -p$turretPin  -f$f 
pca9685 -p$analysisPin -f$f
