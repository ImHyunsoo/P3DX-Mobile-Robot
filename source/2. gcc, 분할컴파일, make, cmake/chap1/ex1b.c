#include <stdio.h>

void funcB (){
  printf ("Hello B\n");
}

int main (){
  funcA();
  funcB();
  return 0;
}

