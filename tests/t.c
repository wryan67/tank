#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//void chomp(char* s) {
//  int len=strlen(s)-1;

//  if (len>=0 && s[len]=='\n') {
//    s[len]==0;
//  }
//}

int main(int argc, char **argv) {
  int     pid;
  char    line[256];
  size_t  len=0;
  ssize_t read;

  FILE *fp=popen("ksh 'sleep 5 >/dev/null 2>&1 & echo $!'","r");

  fgets(line,sizeof(line),fp);
  printf("LINE=%s\n",line);
  pid=atoi(line);  
  fclose(fp);

  printf("PID=%d\n",pid); 
  fflush(stdout);
}
