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
#include <jif/window.h>
#include <jif/jif.h>
#include <jif/layout.h>
#include <jif/resource.h>

void glfw_error_callback(int error_code, const char *description)
{
  printf("[Window 0x%08X] %s\r\n", error_code, description);
}

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  jif::ResourceManager::Init(argv[0]);
  jif::LayoutManager::Init();
  auto &mainMenuBar = jif::LayoutManager::GetMenuBar("main");
  auto &defaultLayout = jif::LayoutManager::GetLayout("default");

  glfwSetErrorCallback(glfw_error_callback);
  glfwInit();

  jif::Window window(800, 600, "JIF GUI", jif::ResourceManager::GetResource("drawable/icon.png").c_str());
  window.MakeCurrent();

  bool reload = false;
  window.Register([&reload](int key, int scancode, int action, int mods)
                  { if (key == GLFW_KEY_R && action == GLFW_RELEASE) reload = true; });

  glewInit();
  glClearColor(0.05f, 0.05f, 0.05f, 1.0f);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  // io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

  ImGui_ImplGlfw_InitForOpenGL(window.GetGLFW(), true);
  ImGui_ImplOpenGL3_Init();

  std::map<std::string, std::function<void()>> ACTIONS{

      {"file.exit", [window]()
       { window.Close(); }},
      {"view.add", []()
       { printf("TODO: show add view wizard\n"); }},
      {"view.manager", []()
       { printf("TODO: show view manager\n"); }},
      {"layout.new", []()
       { printf("TODO: show new layout wizard\n"); }},
      {"layout.save", []()
       { printf("TODO: save layout or show save as wizard if save not yet existing\n"); }},
      {"layout.saveas", []()
       { printf("TODO: show save as wizard\n"); }},

  };

  while (window.Spin())
  {
    if (reload)
    {
      reload = false;
      jif::LayoutManager::Init();
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    if (ImGui::BeginMainMenuBar())
    {
      for (auto &menu : mainMenuBar.Menus)
      {
        auto menuid = menu.Name + "##" + mainMenuBar.Id + '.' + menu.Id;
        if (ImGui::BeginMenu(menuid.c_str()))
        {
          for (auto &item : menu.Items)
          {
            auto itemid = item.Name + "##" + menuid + '.' + item.Id;
            if (ImGui::MenuItem(itemid.c_str(), item.Alt.c_str()))
              if (ACTIONS.count(item.Action))
                ACTIONS[item.Action]();
          }
          ImGui::EndMenu();
        }
      }
      ImGui::EndMainMenuBar();
    }

    ImGui::DockSpaceOverViewport(NULL, ImGuiDockNodeFlags_PassthruCentralNode);

    for (auto &view : defaultLayout.Views)
    {
      auto viewid = view.Name + "##" + defaultLayout.Id + '.' + view.Id;
      if (ImGui::Begin(viewid.c_str()))
      {
        ImGui::SetWindowSize({view.Width, view.Height}, ImGuiCond_FirstUseEver);
        ImGui::SetWindowPos({view.X, view.Y}, ImGuiCond_FirstUseEver);
      }
      ImGui::End();
    }

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
