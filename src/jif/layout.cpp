#include <fstream>
#include <iostream>
#include <jif/layout.h>
#include <jif/resources.h>

std::map<std::string, jif::MenuItem> jif::Layouts::m_MenuItems;
std::map<std::string, jif::Menu> jif::Layouts::m_Menus;
std::map<std::string, jif::MenuBar> jif::Layouts::m_MenuBars;

std::map<std::string, std::vector<std::string>> jif::Layouts::m_WaitForMenus;

void jif::Layouts::Init()
{
    m_MenuItems.clear();
    m_Menus.clear();
    m_MenuBars.clear();

    auto layouts = Resources::GetResource("layout");
    for (auto &file : std::filesystem::directory_iterator(layouts))
    {
        if (!file.is_regular_file())
            continue;

        std::ifstream stream(file.path());
        if (!stream)
        {
            std::cerr << "Failed to open file " << file << std::endl;
            continue;
        }

        nlohmann::json json;
        stream >> json;

        std::string type = json["type"];
        if (type == "menuitem")
            json.get<MenuItem>();
        else if (type == "menu")
            json.get<Menu>();
        else if (type == "menubar")
            json.get<MenuBar>();
    }

    /*std::cout << "Menu Items:" << std::endl;
    for (auto &entry : m_MenuItems)
        std::cout << entry.second << std::endl;
    std::cout << std::endl;

    std::cout << "Menus:" << std::endl;
    for (auto &entry : m_Menus)
        std::cout << entry.second << std::endl;
    std::cout << std::endl;

    std::cout << "Menu Bars:" << std::endl;
    for (auto &entry : m_MenuBars)
        std::cout << entry.second << std::endl;
    std::cout << std::endl;*/
}

const jif::MenuBar &jif::Layouts::GetMenuBar(const std::string &id)
{
    return m_MenuBars[id];
}

void jif::Layouts::to_json(nlohmann::json &json, const MenuItem &menuitem)
{
    json["type"] = "menuitem";
    json["id"] = menuitem.Id;
    json["name"] = menuitem.Name;
    json["alt"] = menuitem.Alt;
    json["action"] = menuitem.Action;
}

void jif::Layouts::to_json(nlohmann::json &json, const Menu &menu)
{
    json["type"] = "menu";
    json["id"] = menu.Id;
    json["name"] = menu.Name;
    json["items"] = menu.Items;
}

void jif::Layouts::to_json(nlohmann::json &json, const MenuBar &menubar)
{
    json["type"] = "menubar";
    json["id"] = menubar.Id;
    json["menus"] = menubar.Menus;
}

void jif::Layouts::from_json(const nlohmann::json &json, MenuItem &menuitem)
{
    menuitem.Id = json["id"];
    menuitem.Name = json["name"];
    menuitem.Alt = json["alt"];
    menuitem.Action = json["action"];

    m_MenuItems[menuitem.Id] = menuitem;
}

void jif::Layouts::from_json(const nlohmann::json &json, Menu &menu)
{
    menu.Id = json["id"];
    menu.Name = json["name"];
    menu.Items = json["items"];

    m_Menus[menu.Id] = menu;

    if (m_WaitForMenus.count(menu.Id))
    {
        auto &menubars = m_WaitForMenus[menu.Id];
        for (auto &id : menubars)
            m_MenuBars[id].Menus.push_back(menu);
        m_WaitForMenus.erase(menu.Id);
    }
}

void jif::Layouts::from_json(const nlohmann::json &json, MenuBar &menubar)
{
    menubar.Id = json["id"];

    std::vector<std::string> menus = json["menus"];
    for (auto &id : menus)
    {
        if (!m_Menus.count(id))
        {
            m_WaitForMenus[id].push_back(menubar.Id);
            continue;
        }
        menubar.Menus.push_back(m_Menus[id]);
    }

    m_MenuBars[menubar.Id] = menubar;
}

template <typename T>
std::ostream &jif::operator<<(std::ostream &out, const std::vector<T> &vec)
{
    out << "[ ";

    bool first = true;
    for (auto &elem : vec)
    {
        if (first)
            first = false;
        else
            out << ", ";
        out << elem;
    }

    return out << " ]";
}

std::ostream &jif::operator<<(std::ostream &out, const MenuItem &menuitem)
{
    return out << "{ Id: " << menuitem.Id << ", Name: " << menuitem.Name << ", Alt: " << menuitem.Alt << ", Action: " << menuitem.Action << " }";
}

std::ostream &jif::operator<<(std::ostream &out, const Menu &menu)
{
    return out << "{ Id: " << menu.Id << ", Name: " << menu.Name << ", Items: " << menu.Items << " }";
}

std::ostream &jif::operator<<(std::ostream &out, const MenuBar &menubar)
{
    return out << "{ Id: " << menubar.Id << ", Name: " << menubar.Menus << " }";
}

void jif::to_json(nlohmann::json &json, const MenuItem &menuitem)
{
    Layouts::to_json(json, menuitem);
}

void jif::to_json(nlohmann::json &json, const Menu &menu)
{
    Layouts::to_json(json, menu);
}

void jif::to_json(nlohmann::json &json, const MenuBar &menubar)
{
    Layouts::to_json(json, menubar);
}

void jif::from_json(const nlohmann::json &json, MenuItem &menuitem)
{
    Layouts::from_json(json, menuitem);
}

void jif::from_json(const nlohmann::json &json, Menu &menu)
{
    Layouts::from_json(json, menu);
}

void jif::from_json(const nlohmann::json &json, MenuBar &menubar)
{
    Layouts::from_json(json, menubar);
}
