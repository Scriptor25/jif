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

#include "resource.h"

#include <map>
#include <string>

namespace jif
{
    class JIFView
    {
    public:
        JIFView(std::string id, const std::string &label, const std::string &type)
            : m_ID(id), m_Label(label), m_Type(type)
        {
        }

        bool &IsOpen() { return m_IsOpen; }
        bool IsOpen() const { return m_IsOpen; }
        const std::string &ID() const { return m_ID; }
        const std::string &Label() const { return m_Label; }
        const std::string &Type() const { return m_Type; }

        std::string ImGuiID() const { return m_Label + "##view@" + m_ID; }

    private:
        bool m_IsOpen = true;
        std::string m_ID;
        std::string m_Label;
        std::string m_Type;
    };
    typedef std::shared_ptr<JIFView> JIFViewPtr;

    class JIFManager
    {
    public:
        JIFManager() {}
        JIFManager(ResourceManager &resources, const std::string &id);

        void CreateView(const std::string &label, const std::string &type);

        std::map<std::string, JIFViewPtr> &Views() { return m_Views; }

        void SaveLayout();

        void OpenSaveWizard();
        void OpenNewWizard();

        void OpenAddWizard();
        void OpenViewManager();

        void ShowSaveWizard();
        void ShowNewWizard();

        void ShowAddWizard();
        void ShowViewManager();

    private:
        void AddView(const std::string &id, const std::string &name, const std::string &type);

    private:
        std::string m_LayoutID;
        std::string m_LayoutName;

        bool m_SaveWizardOpen = false;
        bool m_NewWizardOpen = false;
        bool m_AddWizardOpen = false;
        bool m_ViewManagerOpen = false;

        bool m_Saved = false;
        std::map<std::string, JIFViewPtr> m_Views;
    };
}
