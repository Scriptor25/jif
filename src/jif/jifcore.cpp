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
#include <jif/jifcore.h>
#include <jif/manager.h>
#include <jif/resource.h>
#include <jif/window.h>
#include <rclcpp/rclcpp.hpp>

void glfw_error_callback(int error_code, const char *description)
{
  printf("[GLFW 0x%08X] %s\r\n", error_code, description);
}

void gl_debug_message_callback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar *message, const void *userParam)
{
  (void)source;
  (void)type;
  (void)severity;
  (void)length;
  (void)userParam;
  printf("[GL 0x%08X] %s\r\n", id, message);
}

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  jif::ResourceManager resources(argv[0]);
  auto menubar = resources.GetMenuBar("main");

  glfwSetErrorCallback(glfw_error_callback);
  glfwInit();

  jif::Window window(800, 600, "JIF GUI", resources.GetResource("drawable/icon.png").c_str());
  window.MakeCurrent();

  glewInit();

  glDebugMessageCallback(gl_debug_message_callback, nullptr);
  glEnable(GL_DEBUG_OUTPUT);
  glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);

  glClearColor(0.05f, 0.05f, 0.05f, 1.0f);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  io.IniFilename = nullptr; // Manually Read/Write ini
  io.IniSavingRate = 0.0f;
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  // io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  // io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

  ImGui_ImplGlfw_InitForOpenGL(window.GetGLFW(), true);
  ImGui_ImplOpenGL3_Init();

  auto &actions = jif::ResourceManager::ACTIONS;
  jif::JIFManager manager(resources, "default");

  actions["file.exit"] = [&window]()
  { window.Close(); };
  actions["file.browser"] = [&manager]()
  { manager.OpenFileBrowser(); };
  actions["view.add"] = [&manager]()
  { manager.OpenAddView(); };
  actions["view.manager"] = [&manager]()
  { manager.OpenViewManager(); };
  actions["layout.new"] = [&manager]()
  { manager.OpenNewLayout(); };
  actions["layout.load"] = [&manager]()
  { manager.OpenLoadLayout(); };
  actions["layout.save"] = [&manager]()
  { manager.SaveLayout(); };
  actions["layout.saveas"] = [&manager]()
  { manager.OpenSaveLayout(); };
  actions["layout.reload"] = [&manager]()
  { manager.ReloadLayout(); };
  actions["demo.click"] = []()
  { std::cout << "Hello World!" << std::endl; };

  for (auto &menustr : menubar->Menus)
  {
    auto menu = resources.GetMenu(menustr);
    for (auto &item : menu->Items)
      window.Register(item->Alt, [action = item->Action, &manager]()
                      { jif::ResourceManager::Action(action, manager); });
  }

  rclcpp::init(argc, argv);
  auto core = std::make_shared<jif::JIFCore>();
  std::thread rosthread([&core]()
                        { rclcpp::spin(core); });

  while (window.Spin() && rclcpp::ok())
  {
    manager.NotifyPreNewFrame();

    window.SetSaved(!manager.HasChanges());
    window.SetFilename(manager.GetLayoutName());

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    manager.NotifyInNewFrame();

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
              jif::ResourceManager::Action(item->Action, manager);
          }
          ImGui::EndMenu();
        }
      }
      ImGui::EndMainMenuBar();
    }

    ImGui::DockSpaceOverViewport(NULL, ImGuiDockNodeFlags_PassthruCentralNode);

    manager.ShowFileBrowser();
    manager.ShowSaveLayout();
    manager.ShowNewLayout();
    manager.ShowLoadLayout();
    manager.ShowViewManager();
    manager.ShowEditView();
    manager.ShowAddView();

    for (auto &entry : manager.Views())
    {
      auto view = entry.second;
      if (!view->IsOpen())
        continue;
      if (ImGui::Begin(view->ImGuiID().c_str(), &view->IsOpen()))
      {
        auto type = view->Type();
        size_t i = 0;
        for (auto element : type->Elements)
        {
          jif::ShowArgs showargs{core, view->Fields(), view->Data(i++)};
          element->Show(manager, resources, window, showargs);
        }
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

  rclcpp::shutdown();

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwTerminate();
  glfwSetErrorCallback(NULL);

  return 0;
}
