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
        std::string Id;
        std::string Name;
        std::vector<MenuItem> Items;
    };

    struct MenuBar
    {
        std::string Id;
        std::vector<std::string> Menus;
    };

    struct View
    {
        std::string Id;
        std::string Name;
        std::string View;
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
        LayoutManager();

        void Reinit();

        const MenuBar *GetMenuBar(const std::string &id);
        const Menu *GetMenu(const std::string &id);
        const Layout *GetLayout(const std::string &id);

        std::ostream &operator>>(std::ostream &out) const;

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
        std::map<std::string, MenuItem> m_MenuItems;
        std::map<std::string, Menu> m_Menus;
        std::map<std::string, MenuBar> m_MenuBars;
        std::map<std::string, View> m_Views;
        std::map<std::string, Layout> m_Layouts;
    };

    template <typename T>
    std::ostream &operator<<(std::ostream &out, const std::vector<T> &vec);
    template <typename K, typename V>
    std::ostream &operator<<(std::ostream &out, const std::map<K, V> &map);

    std::ostream &operator<<(std::ostream &out, const LayoutManager &mgr);
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
