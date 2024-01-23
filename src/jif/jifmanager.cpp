#include <imgui/imgui.h>
#include <imgui/misc/cpp/imgui_stdlib.h>
#include <iostream>
#include <jif/jifmanager.h>
#include <jif/jiffile.h>

void jif::JIFManager::CreateView(const std::string &label, JIFViewType type)
{
    static size_t id = 0;
    while (m_Views.count(id++))
        ;
    m_Views[id] = new JIFView(id++, label, type);
}

void jif::JIFManager::SaveLayout()
{
    if (m_Filename.empty())
    {
        OpenSaveWizard();
        return;
    }

    std::cout << "TODO: create layout.json" << std::endl;
    std::cout << "TODO: create info.json" << std::endl;
    std::cout << "TODO: copy imgui.ini" << std::endl;
    std::cout << "TODO: pack jif file" << std::endl;
    // PackJIF("", m_Filename);
}

void jif::JIFManager::OpenSaveWizard()
{
    m_SaveWizardOpen = true;
}

void jif::JIFManager::ShowSaveWizard()
{
    if (!m_SaveWizardOpen)
        return;

    if (ImGui::Begin("Save Layout", &m_SaveWizardOpen))
    {
        ImGui::InputText("Save File", &m_Filename);
        ImGui::Button("Save");
        ImGui::SameLine();
        ImGui::Button("Cancel");
    }
    ImGui::End();
}
