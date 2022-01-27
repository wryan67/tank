
ps -fu `whoami` | awk '{if (/raspivid/ && !/awk/) system(sprintf("kill -9 %d %d",$3, $2))}'
