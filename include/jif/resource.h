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

#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <vector>

namespace jif
{
    struct Drawable
    {
        bool Load();
        void Free();

        std::filesystem::path FilePath;

        char *Pixels = 0;
        int Width = 0;
        int Height = 0;
        int Channels = 0;
    };
    typedef std::shared_ptr<Drawable> DrawablePtr;

    struct Font
    {
        bool Load();
        void Free();

        std::filesystem::path FilePath;

        void *Data = 0;
    };
    typedef std::shared_ptr<Font> FontPtr;

    enum LayoutType
    {
        LayoutType_Error,
        LayoutType_MenuItem,
        LayoutType_Menu,
        LayoutType_MenuBar,
        LayoutType_View,
        LayoutType_ViewLayout,
    };

    LayoutType ToType(const std::string &type);

    struct Layout
    {
        virtual ~Layout() {}
        virtual std::string GetID() const = 0;
        virtual LayoutType GetType() const = 0;
    };
    typedef std::shared_ptr<Layout> LayoutPtr;

    struct MenuItem : Layout
    {
        std::string GetID() const override { return Id; }
        LayoutType GetType() const override { return LayoutType_MenuItem; }

        std::string Id;
        std::string Name;
        std::string Alt;
        std::string Action;
    };
    typedef std::shared_ptr<MenuItem> MenuItemPtr;

    struct Menu : Layout
    {
        std::string GetID() const override { return Id; }
        LayoutType GetType() const override { return LayoutType_Menu; }

        std::string Id;
        std::string Name;
        std::vector<MenuItemPtr> Items;
    };
    typedef std::shared_ptr<Menu> MenuPtr;

    struct MenuBar : Layout
    {
        std::string GetID() const override { return Id; }
        LayoutType GetType() const override { return LayoutType_MenuBar; }

        std::string Id;
        std::vector<std::string> Menus;
    };
    typedef std::shared_ptr<MenuBar> MenuBarPtr;

    struct View : Layout
    {
        std::string GetID() const override { return Id; }
        LayoutType GetType() const override { return LayoutType_View; }

        std::string Id;
        std::string Name;
        std::string ViewType;
        std::map<std::string, std::string> Extra;
    };
    typedef std::shared_ptr<View> ViewPtr;

    struct ViewLayout : Layout
    {
        std::string GetID() const override { return Id; }
        LayoutType GetType() const override { return LayoutType_ViewLayout; }

        std::string Id;
        std::string Name;
        std::vector<ViewPtr> Views;
    };
    typedef std::shared_ptr<ViewLayout> ViewLayoutPtr;

    typedef std::shared_ptr<std::ifstream> IFStreamPtr;

    class ResourceManager
    {
    public:
        ResourceManager(const std::filesystem::path &executable);

        std::filesystem::path GetResource(const char *name) const;
        IFStreamPtr GetResourceStream(const char *name) const;

        void ScanResources();

        const DrawablePtr &GetDrawable(const std::string &id) { return m_Drawables[id]; }
        const FontPtr &GetFont(const std::string &id) { return m_Fonts[id]; }
        const LayoutPtr &GetLayout(LayoutType type, const std::string &id) { return m_Layouts[type][id]; }

        const MenuItemPtr GetMenuItem(const std::string &id) { return std::dynamic_pointer_cast<MenuItem>(m_Layouts[LayoutType_MenuItem][id]); }
        const MenuPtr GetMenu(const std::string &id) { return std::dynamic_pointer_cast<Menu>(m_Layouts[LayoutType_Menu][id]); }
        const MenuBarPtr GetMenuBar(const std::string &id) { return std::dynamic_pointer_cast<MenuBar>(m_Layouts[LayoutType_MenuBar][id]); }
        const ViewPtr GetView(const std::string &id) { return std::dynamic_pointer_cast<View>(m_Layouts[LayoutType_View][id]); }
        const ViewLayoutPtr GetViewLayout(const std::string &id) { return std::dynamic_pointer_cast<ViewLayout>(m_Layouts[LayoutType_ViewLayout][id]); }

    private:
        void ScanDir(const std::filesystem::path &dir);

    private:
        std::filesystem::path m_Root;

        std::map<std::string, DrawablePtr> m_Drawables;
        std::map<std::string, FontPtr> m_Fonts;
        std::map<LayoutType, std::map<std::string, LayoutPtr>> m_Layouts;
    };

    template <typename T>
    std::ostream &operator<<(std::ostream &out, const std::vector<T> &vec);
    template <typename K, typename V>
    std::ostream &operator<<(std::ostream &out, const std::map<K, V> &map);

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

    void from_json(const nlohmann::json &json, MenuItem &menuitem);
    void from_json(const nlohmann::json &json, Menu &menu);
    void from_json(const nlohmann::json &json, MenuBar &menubar);
    void from_json(const nlohmann::json &json, View &view);
    void from_json(const nlohmann::json &json, ViewLayout &layout);

    void from_json(const nlohmann::json &json, MenuItemPtr &menuitem);
    void from_json(const nlohmann::json &json, MenuPtr &menu);
    void from_json(const nlohmann::json &json, MenuBarPtr &menubar);
    void from_json(const nlohmann::json &json, ViewPtr &view);
    void from_json(const nlohmann::json &json, ViewLayoutPtr &layout);
}
