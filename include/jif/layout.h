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
        std::vector<Menu> Menus;
    };

    class Layouts
    {
    public:
        static void Init();

        static const MenuBar &GetMenuBar(const std::string &id);

        static void to_json(nlohmann::json &json, const MenuItem &menuitem);
        static void to_json(nlohmann::json &json, const Menu &menu);
        static void to_json(nlohmann::json &json, const MenuBar &menubar);

        static void from_json(const nlohmann::json &json, MenuItem &menuitem);
        static void from_json(const nlohmann::json &json, Menu &menu);
        static void from_json(const nlohmann::json &json, MenuBar &menubar);

    private:
        static std::map<std::string, MenuItem> m_MenuItems;
        static std::map<std::string, Menu> m_Menus;
        static std::map<std::string, MenuBar> m_MenuBars;

        static std::map<std::string, std::vector<std::string>> m_WaitForMenus;
    };

    template <typename T>
    std::ostream &operator<<(std::ostream &out, const std::vector<T> &vec);

    std::ostream &operator<<(std::ostream &out, const MenuItem &menuitem);
    std::ostream &operator<<(std::ostream &out, const Menu &menu);
    std::ostream &operator<<(std::ostream &out, const MenuBar &menubar);

    void to_json(nlohmann::json &json, const MenuItem &menuitem);
    void to_json(nlohmann::json &json, const Menu &menu);
    void to_json(nlohmann::json &json, const MenuBar &menubar);

    void from_json(const nlohmann::json &json, MenuItem &menuitem);
    void from_json(const nlohmann::json &json, Menu &menu);
    void from_json(const nlohmann::json &json, MenuBar &menubar);
}
