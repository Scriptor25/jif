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

jif::LayoutType jif::ToType(const std::string &type)
{
    if (type == "menuitem")
        return LayoutType_MenuItem;
    if (type == "menu")
        return LayoutType_Menu;
    if (type == "menubar")
        return LayoutType_MenuBar;
    if (type == "view")
        return LayoutType_View;
    if (type == "viewlayout")
        return LayoutType_ViewLayout;

    return LayoutType_Error;
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

    // ScanDir(GetResource("drawable"));
    // ScanDir(GetResource("font"));
    ScanDir(GetResource("layout"));
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
            std::cerr << "Skipping non-json file " << file << " (" << file.path().extension() << ")" << std::endl;
            continue;
        }

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

        LayoutPtr ptr;
        auto layouttype = ToType(type);
        switch (layouttype)
        {
        case LayoutType_MenuItem:
            ptr = json.get<MenuItemPtr>();
            break;

        case LayoutType_Menu:
            ptr = json.get<MenuPtr>();
            break;

        case LayoutType_MenuBar:
            ptr = json.get<MenuBarPtr>();
            break;

        case LayoutType_View:
            ptr = json.get<ViewPtr>();
            break;

        case LayoutType_ViewLayout:
            ptr = json.get<ViewLayoutPtr>();
            break;

        case LayoutType_Error:
        default:
            break;
        }
        m_Layouts[layouttype][id] = ptr;

        stream.close();
    }
}
