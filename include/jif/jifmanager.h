#pragma once

#include <map>
#include <string>

namespace jif
{
    enum JIFViewType
    {
        JIFViewType_Help,
        JIFViewType_Debug,
        JIFViewType_Camera,
        JIFViewType_Joystick,
        JIFViewType_Terminal,
        JIFViewType_Graph,
    };

    class JIFView
    {
    public:
        JIFView(size_t id, const std::string &label, JIFViewType type)
            : m_ID(id), m_Label(label), m_Type(type)
        {
        }

        bool &IsOpen() { return m_IsOpen; }
        bool IsOpen() const { return m_IsOpen; }
        size_t ID() const { return m_ID; }
        std::string Label() const { return m_Label; }
        JIFViewType Type() const { return m_Type; }

        std::string ImGuiID() const { return m_Label + "##view@" + std::to_string(m_ID); }

    private:
        bool m_IsOpen = true;
        size_t m_ID;
        std::string m_Label;
        JIFViewType m_Type;
    };

    class JIFManager
    {
    public:
        void CreateView(const std::string &label, JIFViewType type);

        std::map<size_t, JIFView *> &Views() { return m_Views; }

        void SaveLayoutAs(const std::string &filename);
        void SaveLayout();

        void ShowSaveWizard(bool *open = nullptr);

    private:
        std::string m_Filename;
        std::map<size_t, JIFView *> m_Views;

        bool m_SaveWizardOpen = false;
    };
}
