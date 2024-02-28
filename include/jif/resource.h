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

#include "jifcore.h"
#include "manager.h"

#include <filesystem>
#include <fstream>
#include <imgui/imgui.h>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace jif
{
    class ResourceManager;
    class JIFManager;

    enum ResourceType
    {
        ResourceType_Error,

        ResourceType_Drawable,
        ResourceType_Font,

        ResourceType_Layout_MenuItem,
        ResourceType_Layout_Menu,
        ResourceType_Layout_MenuBar,
        ResourceType_Layout_View,
        ResourceType_Layout_ViewLayout,

        ResourceType_ViewType,
    };
    ResourceType ToType(const std::string &type);

    struct Resource
    {
        virtual ~Resource() {}
        virtual std::string GetID() const = 0;
        virtual ResourceType GetType() const = 0;
    };
    typedef std::shared_ptr<Resource> ResourcePtr;

    struct Drawable : Resource
    {
        std::string GetID() const override { return Id; }
        ResourceType GetType() const override { return ResourceType_Drawable; }

        bool Load();
        void Free();

        std::string Id;
        std::string Filename;

        unsigned char *Pixels = 0;
        int Width = 0;
        int Height = 0;
        int Channels = 0;
    };
    typedef std::shared_ptr<Drawable> DrawablePtr;

    struct Font : Resource
    {
        std::string GetID() const override { return Id; }
        ResourceType GetType() const override { return ResourceType_Font; }

        bool Load();
        void Free();

        std::string Id;
        std::string Filename;

        void *Data = 0;
        size_t Size = 0;
    };
    typedef std::shared_ptr<Font> FontPtr;

    // Layout
    struct Layout : Resource
    {
        virtual ~Layout() {}
    };
    typedef std::shared_ptr<Layout> LayoutPtr;

    struct MenuItem : Layout
    {
        std::string GetID() const override { return Id; }
        ResourceType GetType() const override { return ResourceType_Layout_MenuItem; }

        std::string Id;
        std::string Name;
        std::string Alt;
        std::string Action;
    };
    typedef std::shared_ptr<MenuItem> MenuItemPtr;

    struct Menu : Layout
    {
        std::string GetID() const override { return Id; }
        ResourceType GetType() const override { return ResourceType_Layout_Menu; }

        std::string Id;
        std::string Name;
        std::vector<MenuItemPtr> Items;
    };
    typedef std::shared_ptr<Menu> MenuPtr;

    struct MenuBar : Layout
    {
        std::string GetID() const override { return Id; }
        ResourceType GetType() const override { return ResourceType_Layout_MenuBar; }

        std::string Id;
        std::vector<std::string> Menus;
    };
    typedef std::shared_ptr<MenuBar> MenuBarPtr;

    struct View : Layout
    {
        std::string GetID() const override { return Id; }
        ResourceType GetType() const override { return ResourceType_Layout_View; }

        std::string Id;
        std::string Name;
        std::string ViewType;
        std::map<std::string, std::string> Fields;
    };
    typedef std::shared_ptr<View> ViewPtr;

    struct ViewLayout : Layout
    {
        std::string GetID() const override { return Id; }
        ResourceType GetType() const override { return ResourceType_Layout_ViewLayout; }

        std::string Id;
        std::string Name;
        std::vector<ViewPtr> Views;
    };
    typedef std::shared_ptr<ViewLayout> ViewLayoutPtr;

    // View Element Data
    struct ViewElementData
    {
        ViewElementData(JIFManager &manager, const JIFCorePtr &core)
            : Manager(manager), Core(core) {}

        virtual ~ViewElementData() {}

        JIFManager &Manager;
        JIFCorePtr Core;
    };
    typedef std::shared_ptr<ViewElementData> ViewElementDataPtr;

    struct TextData : ViewElementData
    {
        TextData(JIFManager &manager, const JIFCorePtr &core)
            : ViewElementData(manager, core) {}

        ~TextData();

        std::string Topic;
        std::string Label;
    };
    typedef std::shared_ptr<TextData> TextDataPtr;

    struct ButtonData : ViewElementData
    {
        ButtonData(JIFManager &manager, const JIFCorePtr &core)
            : ViewElementData(manager, core) {}

        ~ButtonData();

        std::string Topic;
        std::string Label;
    };
    typedef std::shared_ptr<ButtonData> ButtonDataPtr;

    struct ImageData : ViewElementData
    {
        ImageData(JIFManager &manager, const JIFCorePtr &core)
            : ViewElementData(manager, core) {}

        ~ImageData();

        std::string Topic;
        uint32_t TextureID;
        ImVec2 Size;
    };
    typedef std::shared_ptr<ImageData> ImageDataPtr;

    struct JoystickData : ViewElementData
    {
        JoystickData(JIFManager &manager, const JIFCorePtr &core)
            : ViewElementData(manager, core) {}

        ~JoystickData();

        std::string StatusTopic;
        std::string DestTopic;
    };
    typedef std::shared_ptr<JoystickData> JoystickDataPtr;

    struct ShowArgs
    {
        JIFCorePtr Core;
        std::map<std::string, std::string> &Fields;
        ViewElementDataPtr& Data;
    };

    // View Type Element
    struct ViewTypeElement
    {
        virtual ~ViewTypeElement() {}
        virtual void Show(JIFManager &manager, ResourceManager &resources, ShowArgs &args) const = 0;
    };
    typedef std::shared_ptr<ViewTypeElement> ViewTypeElementPtr;

    struct ElementText : ViewTypeElement
    {
        void Show(JIFManager &manager, ResourceManager &resources, ShowArgs &args) const override;

        std::string Source;
        std::string Value;
    };
    typedef std::shared_ptr<ElementText> ElementTextPtr;

    struct ElementButton : ViewTypeElement
    {
        void Show(JIFManager &manager, ResourceManager &resources, ShowArgs &args) const override;

        std::string TextSource;
        std::string TextValue;
        std::string Action;
    };
    typedef std::shared_ptr<ElementButton> ElementButtonPtr;

    struct ElementImage : ViewTypeElement
    {
        void Show(JIFManager &manager, ResourceManager &resources, ShowArgs &args) const override;

        std::string Source;
        std::string Value;
    };
    typedef std::shared_ptr<ElementImage> ElementImagePtr;

    struct ElementJoystick : ViewTypeElement
    {
        void Show(JIFManager &manager, ResourceManager &resources, ShowArgs &args) const override;

        std::string Status;
        std::string Dest;
    };
    typedef std::shared_ptr<ElementJoystick> ElementJoystickPtr;

    // View Type
    struct ViewTypeField
    {
        std::string Id;
        std::string Label;
        std::string Default;
    };
    typedef std::shared_ptr<ViewTypeField> ViewTypeFieldPtr;

    struct ViewType
    {
        std::string Id;
        std::string Label;
        std::vector<ViewTypeElementPtr> Elements;
        std::vector<ViewTypeFieldPtr> Fields;
    };
    typedef std::shared_ptr<ViewType> ViewTypePtr;

    // Resource Manager
    typedef std::shared_ptr<std::ifstream> IFStreamPtr;
    class ResourceManager
    {
    public:
        ResourceManager(const std::filesystem::path &executable);

        std::filesystem::path GetResource(const char *name) const;
        IFStreamPtr GetResourceStream(const char *name) const;
        std::filesystem::path GetRoot() const;

        void ScanResources();

        const DrawablePtr &GetDrawable(const std::string &id) { return m_Drawables[id]; }
        const FontPtr &GetFont(const std::string &id) { return m_Fonts[id]; }
        const LayoutPtr &GetLayout(ResourceType type, const std::string &id) { return m_Layouts[type][id]; }
        const ViewTypePtr &GetViewType(const std::string &id) { return m_ViewTypes[id]; }

        const MenuItemPtr GetMenuItem(const std::string &id) { return std::dynamic_pointer_cast<MenuItem>(m_Layouts[ResourceType_Layout_MenuItem][id]); }
        const MenuPtr GetMenu(const std::string &id) { return std::dynamic_pointer_cast<Menu>(m_Layouts[ResourceType_Layout_Menu][id]); }
        const MenuBarPtr GetMenuBar(const std::string &id) { return std::dynamic_pointer_cast<MenuBar>(m_Layouts[ResourceType_Layout_MenuBar][id]); }
        const ViewPtr GetView(const std::string &id) { return std::dynamic_pointer_cast<View>(m_Layouts[ResourceType_Layout_View][id]); }
        const ViewLayoutPtr GetViewLayout(const std::string &id) { return std::dynamic_pointer_cast<ViewLayout>(m_Layouts[ResourceType_Layout_ViewLayout][id]); }

        const std::vector<ViewTypePtr> GetViewTypes();

        static void Action(const std::string &id, JIFManager &manager);

    private:
        void ScanDir(const std::filesystem::path &dir);

    public:
        static std::map<std::string, std::function<void()>> ACTIONS;

    private:
        std::filesystem::path m_Root;

        std::map<std::string, DrawablePtr> m_Drawables;
        std::map<std::string, FontPtr> m_Fonts;
        std::map<ResourceType, std::map<std::string, LayoutPtr>> m_Layouts;
        std::map<std::string, ViewTypePtr> m_ViewTypes;
    };

    std::ostream &operator<<(std::ostream &out, const MenuItem &menuitem);
    std::ostream &operator<<(std::ostream &out, const Menu &menu);
    std::ostream &operator<<(std::ostream &out, const MenuBar &menubar);
    std::ostream &operator<<(std::ostream &out, const View &view);
    std::ostream &operator<<(std::ostream &out, const ViewLayout &layout);

    void to_json(nlohmann::json &json, const MenuItem &menuitem);
    void to_json(nlohmann::json &json, const Menu &menu);
    void to_json(nlohmann::json &json, const MenuBar &menubar);
    void to_json(nlohmann::json &json, const View &view);
    void to_json(nlohmann::json &json, const ViewLayout &layout);

    void to_json(nlohmann::json &json, const MenuItemPtr &menuitem);
    void to_json(nlohmann::json &json, const MenuPtr &menu);
    void to_json(nlohmann::json &json, const MenuBarPtr &menubar);
    void to_json(nlohmann::json &json, const ViewPtr &view);
    void to_json(nlohmann::json &json, const ViewLayoutPtr &layout);

    void from_json(const nlohmann::json &json, Drawable &drawable);
    void from_json(const nlohmann::json &json, Font &font);
    void from_json(const nlohmann::json &json, MenuItem &menuitem);
    void from_json(const nlohmann::json &json, Menu &menu);
    void from_json(const nlohmann::json &json, MenuBar &menubar);
    void from_json(const nlohmann::json &json, View &view);
    void from_json(const nlohmann::json &json, ViewLayout &layout);
    void from_json(const nlohmann::json &json, ViewType &viewtype);
    void from_json(const nlohmann::json &json, ElementText &element);
    void from_json(const nlohmann::json &json, ElementButton &element);
    void from_json(const nlohmann::json &json, ElementImage &element);
    void from_json(const nlohmann::json &json, ElementJoystick &element);
    void from_json(const nlohmann::json &json, ViewTypeField &field);

    void from_json(const nlohmann::json &json, DrawablePtr &drawable);
    void from_json(const nlohmann::json &json, FontPtr &font);
    void from_json(const nlohmann::json &json, MenuItemPtr &menuitem);
    void from_json(const nlohmann::json &json, MenuPtr &menu);
    void from_json(const nlohmann::json &json, MenuBarPtr &menubar);
    void from_json(const nlohmann::json &json, ViewPtr &view);
    void from_json(const nlohmann::json &json, ViewLayoutPtr &layout);
    void from_json(const nlohmann::json &json, ViewTypePtr &viewtype);
    void from_json(const nlohmann::json &json, ViewTypeElementPtr &element);
    void from_json(const nlohmann::json &json, ViewTypeFieldPtr &field);
}
