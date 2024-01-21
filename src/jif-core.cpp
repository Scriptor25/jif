#include <cstdio>
#include <jif/jif.h>
#include <jif/internal/window.h>

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  glfwInit();
  jif::Window window(800, 600, "JIF GUI", "res/image/icon.png");

  while (window.Spin())
    ;

  printf("hello world jif package\n");
  return 0;
}
