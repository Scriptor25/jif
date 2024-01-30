/*
 * ------------------------------------------------------------
 * "THE BEERWARE LICENSE" (Revision 42):
 * Felix Schreiber wrote this code. As long as you retain this
 * notice, you can do whatever you want with this stuff. If we
 * meet someday, and you think this stuff is worth it, you can
 * buy me a beer in return.
 * ------------------------------------------------------------
 */

#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>
#include <imgui/misc/cpp/imgui_stdlib.h>
#include <jif/manager.h>

void jif::JIFManager::NotifyBeforeNewFrame()
{
    if (!m_LoadLayoutName.empty())
    {
        LoadLayout(m_LoadLayoutName);
        m_LoadLayoutName.clear();
    }
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

    if (!HasChanges())
    {
        m_NewLayoutWizardOpen = false;
        Reset();
        return;
    }

    if (ImGui::Begin("New Layout", &m_NewLayoutWizardOpen, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::TextUnformatted("You may have unsaved changes in the current layout. Do you want to save before you proceed?");
        if (ImGui::Button("Save"))
        {
            m_NewLayoutWizardOpen = false;
            OpenSaveLayoutWizard();
        }
        ImGui::SameLine();
        if (ImGui::Button("Dont Save"))
        {
            m_NewLayoutWizardOpen = false;
            Reset();
        }
        ImGui::SameLine();
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

    if (HasChanges())
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
            std::filesystem::path filepath = filename;
            if (!filepath.has_extension())
                LoadLayoutResource(filename);
            else
                m_LoadLayoutName = filename;
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
            SetHasChanges();
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
        switch (m_AddViewWizardState)
        {
        case AddViewWizardState_Name:
            ShowAddViewWizardName();
            break;

        case AddViewWizardState_Type:
            ShowAddViewWizardType();
            break;

        default:
            m_AddViewWizardState = AddViewWizardState_Name;
            m_AddViewWizardData = {};
            m_AddViewWizardOpen = false;
            break;
        }
    }
    ImGui::End();
}

void jif::JIFManager::ShowAddViewWizardName()
{
    ImGui::TextWrapped("This wizard will guide you through the process of adding a view.");
    ImGui::TextWrapped("First, please enter a name/label for your view:");
    ImGui::InputText("Label", &m_AddViewWizardData.Label);
    if (ImGui::Button("Next"))
    {
        if (!m_AddViewWizardData.Label.empty())
            m_AddViewWizardState = AddViewWizardState_Type;
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel"))
    {
        m_AddViewWizardState = AddViewWizardState_;
    }
}

void jif::JIFManager::ShowAddViewWizardType()
{
    static int current = -1;

    ImGui::TextWrapped("Please select one of the following view types:");

    auto viewtypes = m_Resources.GetViewTypes();
    if (ImGui::BeginCombo("##typecombo", current < 0 ? nullptr : viewtypes[current]->Id.c_str()))
    {
        for (size_t n = 0; n < viewtypes.size(); n++)
        {
            bool is_selected = (current == int(n));
            if (ImGui::Selectable(viewtypes[n]->Id.c_str(), is_selected))
                current = n;
            if (is_selected)
                ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }

    if (current >= 0)
    {
        auto &viewtype = viewtypes[current];
        m_AddViewWizardData.Type = viewtype;
        for (auto &field : viewtype->Fields)
        {
            if (!m_AddViewWizardData.Fields.count(field->Id))
                m_AddViewWizardData.Fields[field->Id] = field->Default;

            ImGui::InputText(field->Label.c_str(), &m_AddViewWizardData.Fields[field->Id]);
        }
    }

    if (ImGui::Button("Previous"))
    {
        m_AddViewWizardState = AddViewWizardState_Name;
    }
    ImGui::SameLine();
    if (ImGui::Button("Create"))
    {
        CreateView(m_AddViewWizardData);
        m_AddViewWizardState = AddViewWizardState_;
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel"))
    {
        m_AddViewWizardState = AddViewWizardState_;
    }
}
