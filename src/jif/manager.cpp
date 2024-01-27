#include <fstream>
#include <imgui/imgui.h>
#include <imgui/misc/cpp/imgui_stdlib.h>
#include <iostream>
#include <jif/jif.h>
#include <jif/manager.h>
#include <jif/resource.h>

jif::JIFManager::JIFManager(ResourceManager &resources, const std::string &id)
{
    auto layout = resources.GetViewLayout(id);
    if (!layout)
        return;

    m_LayoutID = layout->Id;
    m_LayoutName = layout->Name;
    for (auto &view : layout->Views)
    {
        auto id = view->Id;
        auto name = view->Name;
        auto type = view->ViewType;
        AddView(id, name, type);
    }
}

void jif::JIFManager::CreateView(const std::string &label, JIFViewType type)
{
    static size_t id = 0;
    while (m_Views.count(std::to_string(id++)))
        ;
    m_Views[std::to_string(id)] = std::make_shared<JIFView>(std::to_string(id++), label, type);
}

void jif::JIFManager::SaveLayout()
{
    if (m_Saved)
        return;

    if (m_LayoutName.empty() || m_LayoutID.empty())
    {
        OpenSaveWizard();
        return;
    }

    auto layout = std::make_shared<ViewLayout>();
    layout->Id = m_LayoutID;
    layout->Name = m_LayoutName;
    for (auto &v : m_Views)
    {
        auto view = std::make_shared<View>();
        view->Id = v.second->ImGuiID();
        view->Name = v.second->Label();
        view->ViewType = ToString(v.second->Type());
        layout->Views.push_back(view);
    }

    std::filesystem::remove_all("tmp");
    std::filesystem::create_directory("tmp");
    std::filesystem::path layoutjson("tmp/layout.json");
    std::filesystem::path imguiini("tmp/imgui.ini");

    {
        nlohmann::json json = layout;
        std::ofstream stream(layoutjson);
        stream << json;
    }

    std::filesystem::copy_file("imgui.ini", imguiini);

    PackJIF("tmp", m_LayoutName + ".jif");

    std::filesystem::remove_all("tmp"); // delete tmp afterwards
}

void jif::JIFManager::OpenSaveWizard()
{
    m_SaveWizardOpen = true;
    m_LayoutName.clear();
    m_LayoutID.clear();
}

void jif::JIFManager::OpenAddWizard()
{
    m_AddWizardOpen = true;
}

void jif::JIFManager::OpenViewManager()
{
    m_ViewManagerOpen = true;
}

void jif::JIFManager::ShowSaveWizard()
{
    if (!m_SaveWizardOpen)
        return;

    if (ImGui::Begin("Save Layout", &m_SaveWizardOpen, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::InputText("Name", &m_LayoutName);
        ImGui::InputText("ID", &m_LayoutID);
        ImGui::Separator();
        if (ImGui::Button("Save"))
        {
            m_SaveWizardOpen = false;
            m_Saved = false;
            SaveLayout();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel"))
        {
            m_SaveWizardOpen = false;
            m_LayoutName.clear();
            m_LayoutID.clear();
        }
    }
    ImGui::End();
}

void jif::JIFManager::ShowAddWizard()
{
    if (!m_AddWizardOpen)
        return;

    if (ImGui::Begin("Add View", &m_AddWizardOpen))
        ;
    ImGui::End();
}

void jif::JIFManager::ShowViewManager()
{
    if (!m_ViewManagerOpen)
        return;

    if (ImGui::Begin("View Manager", &m_ViewManagerOpen))
        ;
    ImGui::End();
}

std::string jif::JIFManager::ToString(JIFViewType type)
{
    switch (type)
    {
    case JIFViewType_Camera:
        return "camera";
    case JIFViewType_Debug:
        return "debug";
    case JIFViewType_Demo:
        return "demo";
    case JIFViewType_Graph:
        return "graph";
    case JIFViewType_Help:
        return "help";
    case JIFViewType_Joystick:
        return "joystick";
    case JIFViewType_Terminal:
        return "terminal";
    default:
        return "error";
    }
}

jif::JIFViewType jif::JIFManager::ViewTypeFromString(const std::string &type)
{
    if (type == "camera")
        return JIFViewType_Camera;
    if (type == "debug")
        return JIFViewType_Debug;
    if (type == "demo")
        return JIFViewType_Demo;
    if (type == "graph")
        return JIFViewType_Graph;
    if (type == "help")
        return JIFViewType_Help;
    if (type == "joystick")
        return JIFViewType_Joystick;
    if (type == "terminal")
        return JIFViewType_Terminal;

    return JIFViewType_Error;
}

void jif::JIFManager::AddView(const std::string &id, const std::string &name, const std::string &type)
{
    auto ttype = ViewTypeFromString(type);
    m_Views[id] = std::make_shared<JIFView>(id, name, ttype);
}
