/*
 * ------------------------------------------------------------
 * "THE BEERWARE LICENSE" (Revision 42):
 * Felix Schreiber wrote this code. As long as you retain this
 * notice, you can do whatever you want with this stuff. If we
 * meet someday, and you think this stuff is worth it, you can
 * buy me a beer in return.
 * ------------------------------------------------------------
 */

#include <iostream>
#include <jif/resource.h>

std::map<std::string, std::function<void()>> jif::ResourceManager::ACTIONS;

jif::ResourceType jif::ToType(const std::string &type)
{
    if (type == "drawable")
        return ResourceType_Drawable;
    if (type == "font")
        return ResourceType_Font;

    if (type == "menuitem")
        return ResourceType_Layout_MenuItem;
    if (type == "menu")
        return ResourceType_Layout_Menu;
    if (type == "menubar")
        return ResourceType_Layout_MenuBar;
    if (type == "view")
        return ResourceType_Layout_View;
    if (type == "viewlayout")
        return ResourceType_Layout_ViewLayout;

    if (type == "viewtype")
        return ResourceType_ViewType;

    return ResourceType_Error;
}

jif::ResourceManager::ResourceManager(const std::filesystem::path &executable)
{
    m_Root = executable.parent_path() / "res";

    ScanResources();
}

std::filesystem::path jif::ResourceManager::GetResource(const char *name) const
{
    return m_Root / name;
}

jif::IFStreamPtr jif::ResourceManager::GetResourceStream(const char *name) const
{
    return std::make_shared<std::ifstream>(GetResource(name));
}

void jif::ResourceManager::ScanResources()
{
    m_Drawables.clear();
    m_Fonts.clear();
    m_Layouts.clear();

    ScanDir(GetResource("drawable"));
    ScanDir(GetResource("font"));
    ScanDir(GetResource("layout"));
    ScanDir(GetResource("viewtype"));
}

const std::vector<jif::ViewTypePtr> jif::ResourceManager::GetViewTypes()
{
    std::vector<ViewTypePtr> viewtypes;
    for (auto &entry : m_ViewTypes)
        viewtypes.push_back(entry.second);
    return viewtypes;
}

void jif::ResourceManager::Action(const std::string &id)
{
    if (auto action = ACTIONS[id])
        action();
    else
        std::cerr << "[ResourceManager] Undefined action '" << id << "'" << std::endl;
}

void jif::ResourceManager::ScanDir(const std::filesystem::path &dir)
{
    for (auto &file : std::filesystem::directory_iterator(dir))
    {
        if (file.is_directory())
        {
            ScanDir(file);
            continue;
        }

        if (file.path().extension() != ".json")
        {
            std::cerr << "[ResourceManager] Skipping non-json file " << file << " (" << file.path().extension() << ")" << std::endl;
            continue;
        }

        std::ifstream stream(file.path());
        if (!stream)
        {
            std::cerr << "[ResourceManager] Failed to open file " << file << std::endl;
            continue;
        }

        nlohmann::json json;
        stream >> json;
        stream.close();

        std::string type = json["type"];
        std::string id = json["id"];

        auto resType = ToType(type);
        switch (resType)
        {
        case ResourceType_Drawable:
            break;

        case ResourceType_Font:
            break;

        case ResourceType_Layout_MenuItem:
            m_Layouts[resType][id] = json.get<MenuItemPtr>();
            break;

        case ResourceType_Layout_Menu:
            m_Layouts[resType][id] = json.get<MenuPtr>();
            break;

        case ResourceType_Layout_MenuBar:
            m_Layouts[resType][id] = json.get<MenuBarPtr>();
            break;

        case ResourceType_Layout_View:
            m_Layouts[resType][id] = json.get<ViewPtr>();
            break;

        case ResourceType_Layout_ViewLayout:
            m_Layouts[resType][id] = json.get<ViewLayoutPtr>();
            break;

        case ResourceType_ViewType:
            m_ViewTypes[id] = json.get<ViewTypePtr>();
            break;

        case ResourceType_Error:
        default:
            break;
        }
    }
}
