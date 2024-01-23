#include <fstream>
#include <iostream>
#include <jif/layout.h>
#include <jif/resource.h>

std::map<std::string, jif::MenuItem> jif::LayoutManager::m_MenuItems;
std::map<std::string, jif::Menu> jif::LayoutManager::m_Menus;
std::map<std::string, jif::MenuBar> jif::LayoutManager::m_MenuBars;
std::map<std::string, jif::View> jif::LayoutManager::m_Views;
std::map<std::string, jif::Layout> jif::LayoutManager::m_Layouts;
std::map<std::string, std::vector<std::string>> jif::LayoutManager::m_WaitForMenus;

void jif::MenuBar::SetMenu(const Menu &menu)
{
    for (auto &m : Menus)
        if (m.Id == menu.Id)
        {
            m = menu;
            break;
        }
}

void jif::LayoutManager::Init()
{
    m_MenuItems.clear();
    m_Menus.clear();
    m_MenuBars.clear();
    m_Views.clear();
    m_Layouts.clear();
    m_WaitForMenus.clear();

    auto layouts = ResourceManager::GetResource("layout");
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
        else if (type == "view")
            json.get<View>();
        else if (type == "layout")
            json.get<Layout>();
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
    std::cout << std::endl;

    std::cout << "Views:" << std::endl;
    for (auto &entry : m_Views)
        std::cout << entry.second << std::endl;
    std::cout << std::endl;

    std::cout << "Layouts:" << std::endl;
    for (auto &entry : m_Layouts)
        std::cout << entry.second << std::endl;
    std::cout << std::endl;*/
}

const jif::MenuBar *jif::LayoutManager::GetMenuBar(const std::string &id)
{
    return &m_MenuBars[id];
}

const jif::Layout *jif::LayoutManager::GetLayout(const std::string &id)
{
    return &m_Layouts[id];
}
