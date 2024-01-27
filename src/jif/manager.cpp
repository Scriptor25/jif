/*
 * ------------------------------------------------------------
 * "THE BEERWARE LICENSE" (Revision 42):
 * Felix Schreiber wrote this code. As long as you retain this
 * notice, you can do whatever you want with this stuff. If we
 * meet someday, and you think this stuff is worth it, you can
 * buy me a beer in return.
 * ------------------------------------------------------------
 */

#include <fstream>
#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>
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

    m_Saved = true;
}

void jif::JIFManager::CreateView(const std::string &label, const std::string &type)
{
    static size_t id = 0;
    while (m_Views.count(std::to_string(id++)))
        ;
    m_Views[std::to_string(id)] = std::make_shared<JIFView>(std::to_string(id), label, type);
    id++;

    m_Saved = false;
}

void jif::JIFManager::SaveLayout()
{
    if (m_Saved)
        return;

    if (m_LayoutName.empty() || m_LayoutID.empty())
    {
        OpenSaveLayoutWizard();
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
        view->ViewType = v.second->Type();
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

    ImGui::SaveIniSettingsToDisk(imguiini.c_str());

    PackJIF("tmp", m_LayoutName + ".jif");

    std::filesystem::remove_all("tmp"); // delete tmp afterwards

    m_Saved = true;
}

void jif::JIFManager::LoadLayout(const std::string &filename)
{
    Reset();

    std::filesystem::remove_all("tmp");
    std::filesystem::create_directory("tmp");
    std::filesystem::path layoutjson("tmp/layout.json");
    std::filesystem::path imguiini("tmp/imgui.ini");

    UnpackJIF(filename, "tmp");

    std::ifstream stream(layoutjson);

    nlohmann::json json;
    stream >> json;

    auto layout = json.get<ViewLayoutPtr>();
    m_LayoutID = layout->Id;
    m_LayoutName = layout->Name;
    for (auto &view : layout->Views)
    {
        auto id = view->Id;
        auto name = view->Name;
        auto type = view->ViewType;
        AddView(id, name, type);
    }
    m_Saved = true;

    ImGui::LoadIniSettingsFromDisk(imguiini.c_str());

    stream.close();

    std::filesystem::remove_all("tmp");
}

void jif::JIFManager::OpenSaveLayoutWizard()
{
    m_SaveLayoutWizardOpen = true;
    m_LayoutNameBkp = m_LayoutName;
    m_LayoutIDBkp = m_LayoutID;
    m_LayoutName.clear();
    m_LayoutID.clear();
}

void jif::JIFManager::OpenNewLayoutWizard()
{
    m_NewLayoutWizardOpen = true;
}

void jif::JIFManager::OpenLoadLayoutWizard()
{
    m_LoadLayoutWizardOpen = true;
}

void jif::JIFManager::OpenAddViewWizard()
{
    m_AddViewWizardOpen = true;
}

void jif::JIFManager::OpenViewManager()
{
    m_ViewManagerOpen = true;
}

void jif::JIFManager::ShowSaveLayoutWizard()
{
    if (!m_SaveLayoutWizardOpen)
        return;

    if (ImGui::Begin("Save Layout", &m_SaveLayoutWizardOpen, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::InputText("Name", &m_LayoutName);
        ImGui::InputText("ID", &m_LayoutID);
        ImGui::Separator();
        if (ImGui::Button("Save"))
        {
            m_SaveLayoutWizardOpen = false;
            m_Saved = false;
            SaveLayout();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel"))
        {
            m_SaveLayoutWizardOpen = false;
            m_LayoutName = m_LayoutNameBkp;
            m_LayoutID = m_LayoutIDBkp;
        }
    }
    ImGui::End();
}

void jif::JIFManager::ShowNewLayoutWizard()
{
    if (!m_NewLayoutWizardOpen)
        return;

    if (m_Saved)
    {
        m_NewLayoutWizardOpen = false;
        Reset();
        return;
    }

    if (ImGui::Begin("New Layout", &m_NewLayoutWizardOpen, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::TextUnformatted("You have unsaved changes in the current layout. Do you want to save before you proceed?");
        if (ImGui::Button("Save"))
        {
            m_NewLayoutWizardOpen = false;
            OpenSaveLayoutWizard();
        }
        if (ImGui::Button("Dont Save"))
        {
            m_NewLayoutWizardOpen = false;
            Reset();
        }
        if (ImGui::Button("Cancel"))
        {
            m_NewLayoutWizardOpen = false;
        }
    }
    ImGui::End();
}

void jif::JIFManager::ShowLoadLayoutWizard()
{
    static std::string filename;

    if (!m_LoadLayoutWizardOpen)
        return;

    if (!m_Saved)
    {
        OpenNewLayoutWizard();
        return;
    }

    if (ImGui::Begin("Load Layout", &m_LoadLayoutWizardOpen, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::InputText("File Name", &filename);
        ImGui::Separator();
        if (ImGui::Button("Load"))
        {
            m_LoadLayoutWizardOpen = false;
            LoadLayout(filename);
            filename.clear();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel"))
        {
            m_LoadLayoutWizardOpen = false;
            filename.clear();
        }
    }
    ImGui::End();
}

void jif::JIFManager::ShowAddViewWizard()
{
    if (!m_AddViewWizardOpen)
        return;

    if (ImGui::Begin("Add View", &m_AddViewWizardOpen, ImGuiWindowFlags_AlwaysAutoResize))
    {
    }
    ImGui::End();
}

void jif::JIFManager::ShowViewManager()
{
    if (!m_ViewManagerOpen)
        return;

    if (ImGui::Begin("View Manager", &m_ViewManagerOpen, ImGuiWindowFlags_AlwaysAutoResize))
    {
        std::vector<std::string> markedForRemoval;
        for (auto &entry : m_Views)
        {
            auto &view = entry.second;
            ImGui::Checkbox(view->ImGuiID().c_str(), &view->IsOpen());
            ImGui::SameLine();
            auto buttonid = "Remove##" + view->ImGuiID();
            if (ImGui::Button(buttonid.c_str()))
                markedForRemoval.push_back(entry.first);
        }

        for (auto &key : markedForRemoval)
        {
            auto &view = m_Views[key];
            ImGui::ClearWindowSettings(view->ImGuiID().c_str());
            m_Views.erase(key);
            m_Saved = false;
        }
    }
    ImGui::End();
}

void jif::JIFManager::AddView(const std::string &id, const std::string &name, const std::string &type)
{
    m_Views[id] = std::make_shared<JIFView>(id, name, type);
    m_Saved = false;
}

void jif::JIFManager::Reset()
{
    m_LayoutID.clear();
    m_LayoutName.clear();
    m_Saved = false;
    for (auto &entry : m_Views)
    {
        auto &view = entry.second;
        ImGui::ClearWindowSettings(view->ImGuiID().c_str());
    }
    m_Views.clear();
}
