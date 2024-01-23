#include <fstream>
#include <iostream>
#include <jif/layout.h>
#include <jif/resource.h>

jif::LayoutManager::LayoutManager()
{
    Reinit();
}

void jif::LayoutManager::Reinit()
{
    m_MenuItems.clear();
    m_Menus.clear();
    m_MenuBars.clear();
    m_Views.clear();
    m_Layouts.clear();

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
        std::string id = json["id"];
        if (type == "menuitem")
            m_MenuItems[id] = json.get<MenuItem>();
        else if (type == "menu")
            m_Menus[id] = json.get<Menu>();
        else if (type == "menubar")
            m_MenuBars[id] = json.get<MenuBar>();
        else if (type == "view")
            m_Views[id] = json.get<View>();
        else if (type == "layout")
            m_Layouts[id] = json.get<Layout>();
    }
}

const jif::MenuBar *jif::LayoutManager::GetMenuBar(const std::string &id)
{
    return &m_MenuBars[id];
}

const jif::Menu *jif::LayoutManager::GetMenu(const std::string &id)
{
    return &m_Menus[id];
}

const jif::Layout *jif::LayoutManager::GetLayout(const std::string &id)
{
    return &m_Layouts[id];
}

std::ostream &jif::LayoutManager::operator>>(std::ostream &out) const
{
    out << "Menu Items:" << std::endl;
    for (auto &entry : m_MenuItems)
        out << entry.second << std::endl;
    out << std::endl;

    out << "Menus:" << std::endl;
    for (auto &entry : m_Menus)
        out << entry.second << std::endl;
    out << std::endl;

    out << "Menu Bars:" << std::endl;
    for (auto &entry : m_MenuBars)
        out << entry.second << std::endl;
    out << std::endl;

    out << "Views:" << std::endl;
    for (auto &entry : m_Views)
        out << entry.second << std::endl;
    out << std::endl;

    out << "Layouts:" << std::endl;
    for (auto &entry : m_Layouts)
        out << entry.second << std::endl;
    return out << std::endl;
}
