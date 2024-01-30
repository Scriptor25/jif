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

void jif::to_json(nlohmann::json &json, const MenuItem &menuitem)
{
    json["type"] = "menuitem";
    json["id"] = menuitem.Id;
    json["name"] = menuitem.Name;
    json["alt"] = menuitem.Alt;
    json["action"] = menuitem.Action;
}

void jif::to_json(nlohmann::json &json, const Menu &menu)
{
    json["type"] = "menu";
    json["id"] = menu.Id;
    json["name"] = menu.Name;
    json["items"] = menu.Items;
}

void jif::to_json(nlohmann::json &json, const MenuBar &menubar)
{
    json["type"] = "menubar";
    json["id"] = menubar.Id;
    json["menus"] = menubar.Menus;
}

void jif::to_json(nlohmann::json &json, const View &view)
{
    json["type"] = "view";
    json["id"] = view.Id;
    json["name"] = view.Name;
    json["viewtype"] = view.ViewType;
    json["fields"] = view.Fields;
}

void jif::to_json(nlohmann::json &json, const ViewLayout &layout)
{
    json["type"] = "viewlayout";
    json["id"] = layout.Id;
    json["name"] = layout.Name;
    json["views"] = layout.Views;
}

void jif::to_json(nlohmann::json &json, const MenuItemPtr &menuitem)
{
    to_json(json, *menuitem);
}

void jif::to_json(nlohmann::json &json, const MenuPtr &menu)
{
    to_json(json, *menu);
}

void jif::to_json(nlohmann::json &json, const MenuBarPtr &menubar)
{
    to_json(json, *menubar);
}

void jif::to_json(nlohmann::json &json, const ViewPtr &view)
{
    to_json(json, *view);
}

void jif::to_json(nlohmann::json &json, const ViewLayoutPtr &layout)
{
    to_json(json, *layout);
}

void jif::from_json(const nlohmann::json &json, Drawable &drawable)
{
    drawable.Id = json["id"];
    drawable.Filename = json["filename"];
}

void jif::from_json(const nlohmann::json &json, Font &font)
{
    font.Id = json["id"];
    font.Filename = json["filename"];
}

void jif::from_json(const nlohmann::json &json, MenuItem &menuitem)
{
    menuitem.Id = json["id"];
    menuitem.Name = json["name"];
    menuitem.Alt = json["alt"];
    menuitem.Action = json["action"];
}

void jif::from_json(const nlohmann::json &json, Menu &menu)
{
    menu.Id = json["id"];
    menu.Name = json["name"];
    menu.Items = json["items"];
}

void jif::from_json(const nlohmann::json &json, MenuBar &menubar)
{
    menubar.Id = json["id"];
    menubar.Menus = json["menus"];
}

void jif::from_json(const nlohmann::json &json, View &view)
{
    view.Id = json["id"];
    view.Name = json["name"];
    view.ViewType = json["viewtype"];
    view.Fields = json["fields"];
}

void jif::from_json(const nlohmann::json &json, ViewLayout &layout)
{
    layout.Id = json["id"];
    layout.Name = json["name"];
    layout.Views = json["views"];
}

void jif::from_json(const nlohmann::json &json, ViewType &viewtype)
{
    viewtype.Id = json["id"];
    viewtype.Elements = json["elements"];
    viewtype.Fields = json["fields"];
}

void jif::from_json(const nlohmann::json &json, ElementText &element)
{
    element.Source = json["text"]["src"];
    element.Value = json["text"]["value"];
}

void jif::from_json(const nlohmann::json &json, ElementButton &element)
{
    element.TextSource = json["text"]["src"];
    element.TextValue = json["text"]["value"];
    element.Action = json["action"];
}

void jif::from_json(const nlohmann::json &json, ElementImage &element)
{
    element.Source = json["img"]["src"];
    element.Value = json["img"]["value"];
}

void jif::from_json(const nlohmann::json &json, ViewTypeField &field)
{
    field.Id = json["id"];
    field.Label = json["label"];
    field.Default = json["default"];
}

void jif::from_json(const nlohmann::json &json, DrawablePtr &drawable)
{
    drawable = std::make_shared<Drawable>();
    from_json(json, *drawable);
}

void jif::from_json(const nlohmann::json &json, FontPtr &font)
{
    font = std::make_shared<Font>();
    from_json(json, *font);
}

void jif::from_json(const nlohmann::json &json, MenuItemPtr &menuitem)
{
    menuitem = std::make_shared<MenuItem>();
    from_json(json, *menuitem);
}

void jif::from_json(const nlohmann::json &json, MenuPtr &menu)
{
    menu = std::make_shared<Menu>();
    from_json(json, *menu);
}

void jif::from_json(const nlohmann::json &json, MenuBarPtr &menubar)
{
    menubar = std::make_shared<MenuBar>();
    from_json(json, *menubar);
}

void jif::from_json(const nlohmann::json &json, ViewPtr &view)
{
    view = std::make_shared<View>();
    from_json(json, *view);
}

void jif::from_json(const nlohmann::json &json, ViewLayoutPtr &layout)
{
    layout = std::make_shared<ViewLayout>();
    from_json(json, *layout);
}

void jif::from_json(const nlohmann::json &json, ViewTypePtr &viewtype)
{
    viewtype = std::make_shared<ViewType>();
    from_json(json, *viewtype);
}

void jif::from_json(const nlohmann::json &json, ViewTypeElementPtr &element)
{
    std::string type = json["type"];
    if (type == "text")
    {
        auto elem = std::make_shared<ElementText>();
        from_json(json, *elem);
        element = elem;
        return;
    }
    if (type == "button")
    {
        auto elem = std::make_shared<ElementButton>();
        from_json(json, *elem);
        element = elem;
        return;
    }
    if (type == "image")
    {
        auto elem = std::make_shared<ElementImage>();
        from_json(json, *elem);
        element = elem;
        return;
    }
}

void jif::from_json(const nlohmann::json &json, ViewTypeFieldPtr &field)
{
    field = std::make_shared<ViewTypeField>();
    from_json(json, *field);
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
    return out << "{ Id: " << view.Id << ", Name: " << view.Name << ", View: " << view.ViewType << " }";
}

std::ostream &jif::operator<<(std::ostream &out, const ViewLayout &layout)
{
    return out << "{ Id: " << layout.Id << ", Name: " << layout.Name << ", Views: " << layout.Views << " }";
}
