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

std::filesystem::path jif::ResourceManager::GetRoot() const
{
    return m_Root;
}

void jif::ResourceManager::ScanResources()
{
    m_Drawables.clear();
    m_Fonts.clear();
    m_Layouts.clear();

    ScanDir(GetRoot());
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

static std::string tolower(const std::string &str)
{
    std::string out = str;
    std::transform(
        str.begin(),
        str.end(),
        out.begin(),
        [](unsigned char c)
        { return std::tolower(c); });
    return out;
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

        auto ext = tolower(file.path().extension());
        auto name = file.path().filename();
        if (ext == ".png" || ext == ".jpg" || ext == ".jpeg")
        {
            if (m_Drawables.count(name))
            {
                std::cout << "[ResourceManager] Skipping already existing drawable '" << name << "' (" << file << ")" << std::endl;
                continue;
            }

            auto drawable = std::make_shared<Drawable>();
            drawable->Filename = file.path().string();
            m_Drawables[name] = drawable;

            continue;
        }

        if (ext == ".ttf")
        {
            if (m_Fonts.count(name))
            {
                std::cout << "[ResourceManager] Skipping already existing font '" << name << "' (" << file << ")" << std::endl;
                continue;
            }

            auto font = std::make_shared<Font>();
            font->Filename = file.path().string();
            m_Fonts[name] = font;

            continue;
        }

        if (ext != ".json")
        {
            std::cerr << "[ResourceManager] Skipping non-json file " << file << " (" << ext << ")" << std::endl;
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
            m_Drawables[id] = json.get<DrawablePtr>();
            break;

        case ResourceType_Font:
            m_Fonts[id] = json.get<FontPtr>();
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
