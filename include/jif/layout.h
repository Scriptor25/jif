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
#include <nlohmann/json.hpp>
#include <vector>

namespace jif
{
    struct MenuItem
    {
        std::string Id;
        std::string Name;
        std::string Alt;
        std::string Action;
    };

    struct Menu
    {
        Menu() {}
        Menu(const std::string &id) : Id(id), Name(id) {}

        std::string Id;
        std::string Name;
        std::vector<MenuItem> Items;
    };

    struct MenuBar
    {
        void SetMenu(const Menu &menu);

        std::string Id;
        std::vector<Menu> Menus;
    };

    struct View
    {
        std::string Id;
        std::string Name;
        std::string View;
        float X;
        float Y;
        float Width;
        float Height;
        std::map<std::string, std::string> Extra;
    };

    struct Layout
    {
        std::string Id;
        std::string Name;
        std::vector<View> Views;
    };

    class LayoutManager
    {
    public:
        static void Init();

        static const MenuBar *GetMenuBar(const std::string &id);
        static const Layout *GetLayout(const std::string &id);

        static void to_json(nlohmann::json &json, const MenuItem &menuitem);
        static void to_json(nlohmann::json &json, const Menu &menu);
        static void to_json(nlohmann::json &json, const MenuBar &menubar);
        static void to_json(nlohmann::json &json, const View &view);
        static void to_json(nlohmann::json &json, const Layout &layout);

        static void from_json(const nlohmann::json &json, MenuItem &menuitem);
        static void from_json(const nlohmann::json &json, Menu &menu);
        static void from_json(const nlohmann::json &json, MenuBar &menubar);
        static void from_json(const nlohmann::json &json, View &view);
        static void from_json(const nlohmann::json &json, Layout &layout);

    private:
        static std::map<std::string, MenuItem> m_MenuItems;
        static std::map<std::string, Menu> m_Menus;
        static std::map<std::string, MenuBar> m_MenuBars;
        static std::map<std::string, View> m_Views;
        static std::map<std::string, Layout> m_Layouts;

        static std::map<std::string, std::vector<std::string>> m_WaitForMenus;
    };

    template <typename T>
    std::ostream &operator<<(std::ostream &out, const std::vector<T> &vec);
    template <typename K, typename V>
    std::ostream &operator<<(std::ostream &out, const std::map<K, V> &map);

    std::ostream &operator<<(std::ostream &out, const MenuItem &menuitem);
    std::ostream &operator<<(std::ostream &out, const Menu &menu);
    std::ostream &operator<<(std::ostream &out, const MenuBar &menubar);
    std::ostream &operator<<(std::ostream &out, const View &view);
    std::ostream &operator<<(std::ostream &out, const Layout &layout);

    void to_json(nlohmann::json &json, const MenuItem &menuitem);
    void to_json(nlohmann::json &json, const Menu &menu);
    void to_json(nlohmann::json &json, const MenuBar &menubar);
    void to_json(nlohmann::json &json, const View &view);
    void to_json(nlohmann::json &json, const Layout &layout);

    void from_json(const nlohmann::json &json, MenuItem &menuitem);
    void from_json(const nlohmann::json &json, Menu &menu);
    void from_json(const nlohmann::json &json, MenuBar &menubar);
    void from_json(const nlohmann::json &json, View &view);
    void from_json(const nlohmann::json &json, Layout &layout);
}
