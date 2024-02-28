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
#include <imgui/imgui_internal.h>
#include <iostream>
#include <jif/jif.h>
#include <jif/manager.h>
#include <jif/resource.h>

jif::JIFManager::JIFManager(ResourceManager &resources, const std::string &id)
    : m_Resources(resources)
{
    LoadLayoutResource(id);
}

bool jif::JIFManager::HasChanges() const
{
    return m_HasChanges || ImGui::GetIO().WantSaveIniSettings;
}

void jif::JIFManager::SetNoChanges()
{
    m_HasChanges = false;
    ImGui::GetIO().WantSaveIniSettings = false;
}

void jif::JIFManager::SetHasChanges()
{
    m_HasChanges = true;
}

void jif::JIFManager::SchedulePre(const std::function<void()> &func)
{
    m_ScheduledPre.push_back(func);
}

void jif::JIFManager::ScheduleIn(const std::function<void()> &func)
{
    m_ScheduledIn.push_back(func);
}

void jif::JIFManager::NotifyPreNewFrame()
{
    for (auto &task : m_ScheduledPre)
        task();
    m_ScheduledPre.clear();
}

void jif::JIFManager::NotifyInNewFrame()
{
    for (auto &task : m_ScheduledIn)
        task();
    m_ScheduledIn.clear();
}

void jif::JIFManager::SaveLayout()
{
    if (m_LayoutName.empty() || m_LayoutID.empty())
    {
        OpenSaveLayout();
        return;
    }

    auto layout = std::make_shared<ViewLayout>();
    layout->Id = m_LayoutID;
    layout->Name = m_LayoutName;
    for (auto &v : m_Views)
    {
        auto view = std::make_shared<View>();
        view->Id = v.second->ID();
        view->Name = v.second->Label();
        view->ViewType = v.second->Type()->Id;
        view->Fields = v.second->Fields();
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

    if (!PackJIF("tmp", m_LayoutName + ".jif"))
        std::cerr << "[JIFManager] Failed to pack jif '" << m_LayoutName << ".jif' from 'tmp'" << std::endl;

    std::filesystem::remove_all("tmp"); // delete tmp afterwards

    SetNoChanges();
}

void jif::JIFManager::LoadLayout(const std::string filename)
{
    Reset();

    m_LayoutFilename = filename;

    std::filesystem::remove_all("tmp");
    std::filesystem::create_directory("tmp");
    std::filesystem::path layoutjson("tmp/layout.json");
    std::filesystem::path imguiini("tmp/imgui.ini");

    if (!UnpackJIF(filename, "tmp"))
    {
        std::cerr << "[JIFManager] Failed to unpack jif '" << filename << "' to 'tmp'" << std::endl;
        return;
    }

    std::ifstream stream(layoutjson);
    if (!stream)
    {
        std::cerr << "[JIFManager] Failed to open ifstream for " << layoutjson << std::endl;
        return;
    }

    nlohmann::json json;
    stream >> json;
    stream.close();

    auto layout = json.get<ViewLayoutPtr>();
    m_LayoutID = layout->Id;
    m_LayoutName = layout->Name;
    for (auto &view : layout->Views)
    {
        auto id = view->Id;
        auto name = view->Name;
        auto type = m_Resources.GetViewType(view->ViewType);
        auto fields = view->Fields;
        AddView(id, name, type, fields);
    }

    ImGui::LoadIniSettingsFromDisk(imguiini.c_str());
    std::filesystem::remove_all("tmp");
}

void jif::JIFManager::LoadLayoutResource(const std::string layoutid)
{
    Reset();

    m_LayoutFilename = layoutid;

    auto layout = m_Resources.GetViewLayout(layoutid);
    if (!layout)
    {
        std::cerr << "[JIFManager] Failed to get view layout for id '" << layoutid << "'" << std::endl;
        return;
    }

    m_LayoutID = layout->Id;
    m_LayoutName = layout->Name;
    std::cout << "[JIFManager] Loading layout '" << m_LayoutName << "' (" << m_LayoutID << ")" << std::endl;
    for (auto &view : layout->Views)
    {
        auto id = view->Id;
        auto name = view->Name;
        auto type = m_Resources.GetViewType(view->ViewType);
        auto fields = view->Fields;
        AddView(id, name, type, fields);
    }
}

void jif::JIFManager::ReloadLayout()
{
    if (m_LayoutFilename.empty())
        return;

    std::filesystem::path filepath(m_LayoutFilename);
    if (!filepath.has_extension())
        LoadLayoutResource(m_LayoutFilename);
    else
        SchedulePre([this]()
                    { LoadLayout(m_LayoutFilename); });
}

void jif::JIFManager::OpenSaveLayout()
{
    ImGui::OpenPopup("layout.save");
    m_LayoutName.clear();
    m_LayoutID.clear();
}

void jif::JIFManager::OpenNewLayout()
{
    ImGui::OpenPopup("layout.new");
}

void jif::JIFManager::OpenLoadLayout()
{
    ImGui::OpenPopup("layout.load");
}

void jif::JIFManager::OpenViewManager()
{
    m_ViewManagerOpen = true;
}

void jif::JIFManager::OpenEditView(const JIFViewPtr &view)
{
    m_EditViewOpen = true;
    m_EditViewView = view;
}

void jif::JIFManager::OpenAddView()
{
    m_AddViewWizardState = AddViewWizardState_Name;
    m_AddViewWizardData = {};
    m_AddViewOpen = true;
}

void jif::JIFManager::Reset()
{
    m_LayoutID.clear();
    m_LayoutName.clear();
    m_Views.clear();
    m_ScheduledPre.clear();
    m_LayoutFilename.clear();
    ImGui::ClearIniSettings();
    SetNoChanges();
}
