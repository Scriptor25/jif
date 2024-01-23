#include <imgui/imgui.h>
#include <jif/jifmanager.h>

void jif::JIFManager::CreateView(const std::string &label, JIFViewType type)
{
    static size_t id = 0;
    while (m_Views.count(id++))
        ;
    m_Views[id] = new JIFView(id++, label, type);
}

void jif::JIFManager::SaveLayoutAs(const std::string &filename)
{
}

void jif::JIFManager::SaveLayout()
{
    if (m_Filename.empty())
        return; // TODO: open save-file-wizard
    SaveLayoutAs(m_Filename);
}

void jif::JIFManager::ShowSaveWizard()
{
    if (!m_SaveWizardOpen)
        return;

    if (ImGui::Begin("Save Layout", &m_SaveWizardOpen))
    {
    }
    ImGui::End();
}
