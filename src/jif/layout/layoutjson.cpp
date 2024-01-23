#include <iostream>
#include <jif/layout.h>

void jif::LayoutManager::to_json(nlohmann::json &json, const MenuItem &menuitem)
{
    json["type"] = "menuitem";
    json["id"] = menuitem.Id;
    json["name"] = menuitem.Name;
    json["alt"] = menuitem.Alt;
    json["action"] = menuitem.Action;
}

void jif::LayoutManager::to_json(nlohmann::json &json, const Menu &menu)
{
    json["type"] = "menu";
    json["id"] = menu.Id;
    json["name"] = menu.Name;
    json["items"] = menu.Items;
}

void jif::LayoutManager::to_json(nlohmann::json &json, const MenuBar &menubar)
{
    json["type"] = "menubar";
    json["id"] = menubar.Id;
    json["menus"] = menubar.Menus;
}

void jif::LayoutManager::to_json(nlohmann::json &json, const View &view)
{
    json["type"] = "view";
    json["id"] = view.Id;
    json["name"] = view.Name;
    json["view"] = view.View;
    json["extra"] = view.Extra;
}

void jif::LayoutManager::to_json(nlohmann::json &json, const Layout &layout)
{
    json["type"] = "layout";
    json["id"] = layout.Id;
    json["name"] = layout.Name;
    json["views"] = layout.Views;
}

void jif::LayoutManager::from_json(const nlohmann::json &json, MenuItem &menuitem)
{
    menuitem.Id = json["id"];
    menuitem.Name = json["name"];
    menuitem.Alt = json["alt"];
    menuitem.Action = json["action"];
}

void jif::LayoutManager::from_json(const nlohmann::json &json, Menu &menu)
{
    menu.Id = json["id"];
    menu.Name = json["name"];
    menu.Items = json["items"];
}

void jif::LayoutManager::from_json(const nlohmann::json &json, MenuBar &menubar)
{
    menubar.Id = json["id"];
    menubar.Menus = json["menus"];
}

void jif::LayoutManager::from_json(const nlohmann::json &json, View &view)
{
    view.Id = json["id"];
    view.Name = json["name"];
    view.View = json["view"];
    view.Extra = json["extra"];
}

void jif::LayoutManager::from_json(const nlohmann::json &json, Layout &layout)
{
    layout.Id = json["id"];
    layout.Name = json["name"];
    layout.Views = json["views"];
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

template <typename K, typename V>
std::ostream &jif::operator<<(std::ostream &out, const std::map<K, V> &map)
{
    out << "{ ";

    bool first = true;
    for (auto &entry : map)
    {
        if (first)
            first = false;
        else
            out << ", ";
        out << entry.first << ": " << entry.second;
    }

    return out << " }";
}

std::ostream &jif::operator<<(std::ostream &out, const LayoutManager &mgr)
{
    return mgr >> out;
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
    return out << "{ Id: " << menubar.Id << ", Menus: " << menubar.Menus << " }";
}

std::ostream &jif::operator<<(std::ostream &out, const View &view)
{
    return out << "{ Id: " << view.Id << ", Name: " << view.Name << ", View: " << view.View << ", Extra: " << view.Extra << " }";
}

std::ostream &jif::operator<<(std::ostream &out, const Layout &layout)
{
    return out << "{ Id: " << layout.Id << ", Name: " << layout.Name << ", Views: " << layout.Views << " }";
}

void jif::to_json(nlohmann::json &json, const MenuItem &menuitem)
{
    LayoutManager::to_json(json, menuitem);
}

void jif::to_json(nlohmann::json &json, const Menu &menu)
{
    LayoutManager::to_json(json, menu);
}

void jif::to_json(nlohmann::json &json, const MenuBar &menubar)
{
    LayoutManager::to_json(json, menubar);
}

void jif::to_json(nlohmann::json &json, const View &view)
{
    LayoutManager::to_json(json, view);
}

void jif::to_json(nlohmann::json &json, const Layout &layout)
{
    LayoutManager::to_json(json, layout);
}

void jif::from_json(const nlohmann::json &json, MenuItem &menuitem)
{
    LayoutManager::from_json(json, menuitem);
}

void jif::from_json(const nlohmann::json &json, Menu &menu)
{
    LayoutManager::from_json(json, menu);
}

void jif::from_json(const nlohmann::json &json, MenuBar &menubar)
{
    LayoutManager::from_json(json, menubar);
}

void jif::from_json(const nlohmann::json &json, View &view)
{
    LayoutManager::from_json(json, view);
}

void jif::from_json(const nlohmann::json &json, Layout &layout)
{
    LayoutManager::from_json(json, layout);
}
