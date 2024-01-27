/*
 * ------------------------------------------------------------
 * "THE BEERWARE LICENSE" (Revision 42):
 * Felix Schreiber wrote this code. As long as you retain this
 * notice, you can do whatever you want with this stuff. If we
 * meet someday, and you think this stuff is worth it, you can
 * buy me a beer in return.
 * ------------------------------------------------------------
 */

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

        void SaveLayout();

        void OpenSaveWizard();
        void ShowSaveWizard();

        static std::string ToString(JIFViewType type);

    private:
        std::string m_LayoutID;
        std::string m_LayoutName;
        bool m_SaveWizardOpen = false;

        std::map<size_t, JIFView *> m_Views;
    };
}
