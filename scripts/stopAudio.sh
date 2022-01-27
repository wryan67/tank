
ps -fu `whoami` | awk '{if ((/arecord/||/play/||/sox/||/record.*sh/) && !/awk/) system(sprintf("kill -9 %d",$2))}'
ps -fu `whoami` | awk '{if ((/arecord/||/play/||/sox/||/record.*sh/) && !/awk/) system(sprintf("kill -9 %d",$2))}'

