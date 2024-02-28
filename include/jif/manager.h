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
    struct ViewElementData;
    typedef std::shared_ptr<ViewElementData> ViewElementDataPtr;
    struct ViewType;
    typedef std::shared_ptr<ViewType> ViewTypePtr;
    class ResourceManager;

    class JIFView
    {
    public:
        JIFView(std::string id, const std::string &label, const ViewTypePtr &type)
            : JIFView(id, label, type, {})
        {
        }

        JIFView(std::string id, const std::string &label, const ViewTypePtr &type, const std::map<std::string, std::string> &fields)
            : m_ID(id), m_Label(label), m_Type(type), m_Fields(fields)
        {
        }

        bool &IsOpen() { return m_IsOpen; }
        bool IsOpen() const { return m_IsOpen; }
        const std::string &ID() const { return m_ID; }
        std::string &Label() { return m_Label; }
        const std::string &Label() const { return m_Label; }
        ViewTypePtr Type() const { return m_Type; }

        ViewElementDataPtr& Data(size_t i) { return m_Data[i]; }
        std::map<size_t, ViewElementDataPtr> &Data() { return m_Data; }
        std::map<std::string, std::string> &Fields() { return m_Fields; }

        std::string ImGuiID() const { return m_Label + "##view@" + m_ID; }

    private:
        bool m_IsOpen = true;
        std::string m_ID;
        std::string m_Label;
        ViewTypePtr m_Type;

        std::map<size_t, ViewElementDataPtr> m_Data;
        std::map<std::string, std::string> m_Fields;
    };
    typedef std::shared_ptr<JIFView> JIFViewPtr;

    enum AddViewWizardState
    {
        AddViewWizardState_Name,
        AddViewWizardState_Type,
        AddViewWizardState_,
    };

    struct AddViewWizardData
    {
        std::string Label;
        ViewTypePtr Type;
        std::map<std::string, std::string> Fields;
    };

    class JIFManager
    {
    public:
        JIFManager(ResourceManager &resources) : m_Resources(resources) {}
        JIFManager(ResourceManager &resources, const std::string &id);

        void CreateView(const std::string &label, const ViewTypePtr &type);
        void CreateView(const AddViewWizardData &data);

        bool HasChanges() const;
        void SetNoChanges();
        void SetHasChanges();

        std::map<std::string, JIFViewPtr> &Views() { return m_Views; }

        void SchedulePre(const std::function<void()> &func);
        void ScheduleIn(const std::function<void()> &func);

        void NotifyPreNewFrame();
        void NotifyInNewFrame();

        void SaveLayout();
        void LoadLayout(const std::string filename);
        void LoadLayoutResource(const std::string layoutid);
        void ReloadLayout();

        const std::string &GetLayoutName() const { return m_LayoutName; }

        void OpenSaveLayout();
        void OpenNewLayout();
        void OpenLoadLayout();

        void OpenViewManager();
        void OpenEditView(const JIFViewPtr &view);
        void OpenAddView();

        void ShowSaveLayout();
        void ShowNewLayout();
        void ShowLoadLayout();

        void ShowViewManager();
        void ShowEditView();
        void ShowAddView();

        void ShowAddViewName();
        void ShowAddViewType();

    private:
        void AddView(const std::string &id, const std::string &name, const ViewTypePtr &type, const std::map<std::string, std::string> &fields);
        void Reset();

    private:
        bool m_HasChanges = false;
        ResourceManager &m_Resources;

        std::vector<std::function<void()>> m_ScheduledPre;
        std::vector<std::function<void()>> m_ScheduledIn;

        std::string m_LayoutFilename;
        std::string m_LayoutID;
        std::string m_LayoutName;

        std::map<std::string, JIFViewPtr> m_Views;

        bool m_AddViewOpen = false;
        AddViewWizardState m_AddViewWizardState;
        AddViewWizardData m_AddViewWizardData;

        bool m_EditViewOpen = false;
        JIFViewPtr m_EditViewView;

        bool m_ViewManagerOpen = false;
    };
}
