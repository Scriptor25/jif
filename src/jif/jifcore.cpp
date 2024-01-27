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
#include <iostream>
#include <jif/jif.h>
#include <jif/manager.h>
#include <jif/resource.h>
#include <jif/window.h>

void glfw_error_callback(int error_code, const char *description)
{
  printf("[Window 0x%08X] %s\r\n", error_code, description);
}

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  jif::ResourceManager resources(argv[0]);
  jif::JIFManager manager(resources, "default");
  auto menubar = resources.GetMenuBar("main");

  glfwSetErrorCallback(glfw_error_callback);
  glfwInit();

  jif::Window window(800, 600, "JIF GUI", resources.GetResource("drawable/icon.png").c_str());
  window.MakeCurrent();

  window.Register(
      [&resources, &manager, &menubar](int key, int scancode, int action, int mods)
      {
        (void)scancode;
        (void)mods;

        if (key == GLFW_KEY_R && action == GLFW_RELEASE)
        {
          resources.ScanResources();
          manager = jif::JIFManager(resources, "default");
          menubar = resources.GetMenuBar("main");
          return true;
        }

        return false;
      });

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

      {"file.exit", [&window]()
       { window.Close(); }},
      {"view.add", [&manager]()
       { manager.OpenAddWizard(); }},
      {"view.manager", [&manager]()
       { manager.OpenViewManager(); }},
      {"layout.new", []()
       { printf("TODO: show new layout wizard\n"); }},
      {"layout.save", [&manager]()
       { manager.SaveLayout(); }},
      {"layout.saveas", [&manager]()
       { manager.OpenSaveWizard(); }},

  };

  for (auto &menustr : menubar->Menus)
  {
    auto menu = resources.GetMenu(menustr);
    for (auto &item : menu->Items)
      if (auto action = ACTIONS[item->Action])
        window.Register(item->Alt, action);
  }

  while (window.Spin())
  {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    if (ImGui::BeginMainMenuBar())
    {
      for (auto &menustr : menubar->Menus)
      {
        auto menu = resources.GetMenu(menustr);
        auto menuid = menu->Name + "##" + menubar->Id + '.' + menu->Id;
        if (ImGui::BeginMenu(menuid.c_str()))
        {
          for (auto &item : menu->Items)
          {
            auto itemid = item->Name + "##" + menuid + '.' + item->Id;
            if (ImGui::MenuItem(itemid.c_str(), item->Alt.c_str()))
              if (auto action = ACTIONS[item->Action])
                action();
          }
          ImGui::EndMenu();
        }
      }
      ImGui::EndMainMenuBar();
    }

    ImGui::DockSpaceOverViewport(NULL, ImGuiDockNodeFlags_PassthruCentralNode);

    manager.ShowSaveWizard();
    manager.ShowAddWizard();
    manager.ShowViewManager();

    for (auto &view : manager.Views())
    {
      auto viewid = view.second->ImGuiID();
      if (ImGui::Begin(viewid.c_str()))
      {
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
