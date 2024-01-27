#include <fstream>
#include <imgui/imgui.h>
#include <imgui/misc/cpp/imgui_stdlib.h>
#include <iostream>
#include <jif/jif.h>
#include <jif/manager.h>
#include <jif/resource.h>

void jif::JIFManager::CreateView(const std::string &label, JIFViewType type)
{
    static size_t id = 0;
    while (m_Views.count(id++))
        ;
    m_Views[id] = new JIFView(id++, label, type);
}

void jif::JIFManager::SaveLayout()
{
    if (m_LayoutName.empty() || m_LayoutID.empty())
    {
        OpenSaveWizard();
        return;
    }

    ViewLayout layout;
    layout.Id = m_LayoutID;
    layout.Name = m_LayoutName;
    for (auto &v : m_Views)
    {
        auto view = std::make_shared<View>();
        view->Id = v.second->ImGuiID();
        view->Name = v.second->Label();
        view->ViewType = ToString(v.second->Type());
        layout.Views.push_back(view);
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
}

void jif::JIFManager::OpenSaveWizard()
{
    m_SaveWizardOpen = true;
    m_LayoutName.clear();
    m_LayoutID.clear();
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

std::string jif::JIFManager::ToString(JIFViewType type)
{
    switch (type)
    {
    case JIFViewType_Camera:
        return "camera";
    case JIFViewType_Debug:
        return "debug";
    case JIFViewType_Graph:
        return "graph";
    case JIFViewType_Help:
        return "help";
    case JIFViewType_Joystick:
        return "joystick";
    case JIFViewType_Terminal:
        return "terminal";
    default:
        return "none";
    }
}
