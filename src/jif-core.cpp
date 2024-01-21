/*
 * ------------------------------------------------------------
 * "THE BEERWARE LICENSE" (Revision 42):
 * Felix Schreiber wrote this code. As long as you retain this
 * notice, you can do whatever you want with this stuff. If we
 * meet someday, and you think this stuff is worth it, you can
 * buy me a beer in return.
 * ------------------------------------------------------------
 */

#include <filesystem>
#include <GL/glew.h>
#include <imgui/backends/imgui_impl_glfw.h>
#include <imgui/backends/imgui_impl_opengl3.h>
#include <imgui/imgui.h>
#include <jif/jif.h>
#include <jif/internal/window.h>

void glfw_error_callback(int error_code, const char *description)
{
  printf("[Window 0x%08X] %s\r\n", error_code, description);
}

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  jif::Resources::Init(argv[0]);

  glfwSetErrorCallback(glfw_error_callback);
  glfwInit();

  jif::Window window(800, 600, "JIF GUI", jif::Resources::GetResource("image/icon.png").c_str());
  window.MakeCurrent();

  glewInit();
  glClearColor(0.05f, 0.05f, 0.05f, 1.0f);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

  ImGui_ImplGlfw_InitForOpenGL(window.GetGLFW(), true);
  ImGui_ImplOpenGL3_Init();

  while (window.Spin())
  {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::DockSpaceOverViewport(NULL, ImGuiDockNodeFlags_PassthruCentralNode);
    ImGui::ShowDemoWindow();

    glViewport(0, 0, window.GetWidth(), window.GetHeight());
    glClear(GL_COLOR_BUFFER_BIT);

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
    {
      ImGui::UpdatePlatformWindows();
      ImGui::RenderPlatformWindowsDefault();
      window.MakeCurrent();
    }
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwTerminate();
  glfwSetErrorCallback(NULL);

  return 0;
}
