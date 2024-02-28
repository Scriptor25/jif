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

void jif::JIFManager::ShowSaveLayout()
{
    bool save_layout = false;

    if (ImGui::BeginPopup("layout.save"))
    {
        ImGui::InputText("Name", &m_LayoutName);
        ImGui::InputText("ID", &m_LayoutID);
        ImGui::Separator();
        if (ImGui::Button("Save"))
        {
            ImGui::CloseCurrentPopup();
            save_layout = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel"))
            ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    if (save_layout)
        SaveLayout();
}

void jif::JIFManager::ShowNewLayout()
{
    bool reset = false;
    bool open_save_layout = false;

    if (ImGui::BeginPopup("layout.new"))
    {
        if (!HasChanges())
        {
            ImGui::CloseCurrentPopup();
            reset = true;
        }

        ImGui::TextUnformatted("You may have unsaved changes in the current layout. Do you want to save before you proceed?");
        if (ImGui::Button("Save"))
        {
            ImGui::CloseCurrentPopup();
            open_save_layout = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Dont Save"))
        {
            ImGui::CloseCurrentPopup();
            reset = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel"))
        {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    if (reset)
        Reset();
    if (open_save_layout)
        OpenSaveLayout();
}

void jif::JIFManager::ShowLoadLayout()
{
    static std::string filename;

    bool open_new_layout = false;
    bool load_layout_resource = false;

    if (ImGui::BeginPopup("layout.load"))
    {
        if (HasChanges())
            open_new_layout = true;

        ImGui::InputText("File Name", &filename);
        ImGui::Separator();
        if (ImGui::Button("Load"))
        {
            ImGui::CloseCurrentPopup();
            std::filesystem::path filepath = filename;
            if (!filepath.has_extension())
            {
                load_layout_resource = true;
            }
            else
            {
                SchedulePre(
                    [this]()
                    {
                        LoadLayout(filename);
                        filename.clear();
                    });
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel"))
        {
            ImGui::CloseCurrentPopup();
            filename.clear();
        }
        ImGui::EndPopup();
    }

    if (open_new_layout)
        OpenNewLayout();
    if (load_layout_resource)
    {
        LoadLayoutResource(filename);
        filename.clear();
    }
}

void jif::JIFManager::ShowViewManager()
{
    if (!m_ViewManagerOpen)
        return;

    std::string toremove;
    std::string toedit;

    if (ImGui::Begin("View Manager", &m_ViewManagerOpen, ImGuiWindowFlags_AlwaysAutoResize))
    {
        for (auto &entry : m_Views)
        {
            auto &view = entry.second;
            ImGui::Checkbox(view->ImGuiID().c_str(), &view->IsOpen());

            ImGui::SameLine();
            auto removeid = "Remove##" + view->ImGuiID();
            if (ImGui::Button(removeid.c_str()))
                toremove = entry.first;

            ImGui::SameLine();
            auto editid = "Edit##" + view->ImGuiID();
            if (ImGui::Button(editid.c_str()))
                toedit = entry.first;
        }
    }
    ImGui::End();

    if (!toremove.empty())
    {
        auto view = m_Views[toremove];
        ImGui::ClearWindowSettings(view->ImGuiID().c_str());
        m_Views.erase(toremove);
        SetHasChanges();
    }
    if (!toedit.empty())
    {
        auto view = m_Views[toedit];
        OpenEditView(view);
    }
}

void jif::JIFManager::ShowEditView()
{
    static std::map<std::string, std::string> fields;
    static std::string label;
    static JIFViewPtr view;

    if (!m_EditViewOpen)
    {
        if (view)
        {
            fields.clear();
            label.clear();
            view = nullptr;
        }
        return;
    }

    bool apply = false;
    bool done = false;

    if (ImGui::Begin("Edit View", &m_EditViewOpen, ImGuiWindowFlags_AlwaysAutoResize))
    {
        if (!view)
        {
            view = m_EditViewView;
            label = view->Label();
            fields.clear();
            for (auto &field : view->Type()->Fields)
                fields[field->Id] = view->Fields()[field->Id];
        }

        ImGui::InputText("Label", &label);
        ImGui::Separator();

        for (auto &field : view->Type()->Fields)
            ImGui::InputText(field->Label.c_str(), &fields[field->Id]);

        apply = ImGui::Button("Apply");
        ImGui::SameLine();
        done = ImGui::Button("Done");
    }
    ImGui::End();

    if (apply)
    {
        view->Label() = label;
        for (auto &entry : fields)
            view->Fields()[entry.first] = entry.second;
        view->Data().clear();
        SetHasChanges();
    }
    if (done)
    {
        fields.clear();
        label.clear();
        view = nullptr;
        m_EditViewOpen = false;
    }
}

void jif::JIFManager::ShowAddView()
{
    if (!m_AddViewOpen)
        return;

    if (ImGui::Begin("Add View", &m_AddViewOpen, ImGuiWindowFlags_AlwaysAutoResize))
    {
        switch (m_AddViewWizardState)
        {
        case AddViewWizardState_Name:
            ShowAddViewName();
            break;

        case AddViewWizardState_Type:
            ShowAddViewType();
            break;

        default:
            m_AddViewWizardState = AddViewWizardState_Name;
            m_AddViewWizardData = {};
            m_AddViewOpen = false;
            break;
        }
    }
    ImGui::End();
}

void jif::JIFManager::ShowAddViewName()
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

void jif::JIFManager::ShowAddViewType()
{
    static int current = -1;

    ImGui::TextWrapped("Please select one of the following view types:");

    auto viewtypes = m_Resources.GetViewTypes();
    if (ImGui::BeginCombo("##typecombo", current < 0 ? nullptr : viewtypes[current]->Label.c_str()))
    {
        for (size_t n = 0; n < viewtypes.size(); n++)
        {
            bool is_selected = (current == int(n));
            if (ImGui::Selectable(viewtypes[n]->Label.c_str(), is_selected))
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
