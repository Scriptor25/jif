/*
 * ------------------------------------------------------------
 * "THE BEERWARE LICENSE" (Revision 42):
 * Felix Schreiber wrote this code. As long as you retain this
 * notice, you can do whatever you want with this stuff. If we
 * meet someday, and you think this stuff is worth it, you can
 * buy me a beer in return.
 * ------------------------------------------------------------
 */

#include <imgui/imgui.h>
#include <jif/resource.h>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <stb/stb_image.h>

void jif::ElementText::Show(ResourceManager &resources, JIFCorePtr core, std::map<std::string, std::string> &fields, ViewElementDataPtr &data) const
{
    (void)resources;
    (void)core;
    (void)fields;

    if (!data)
    {
        auto textdata = std::make_shared<TextData>();

        auto value =
            (Value[0] == '$')
                ? fields[Value.substr(1)]
                : Value;

        if (Source == "value")
        {
            textdata->Label = value;
        }
        else if (Source == "file")
        {
        }
        else if (Source == "resource")
        {
        }
        else if (Source == "ros")
        {
            core->RegisterSubscription<std_msgs::msg::String>(
                value,
                [&label = textdata->Label](const std_msgs::msg::String &msg)
                {
                    label = msg.data;
                });
        }

        data = textdata;
    }

    auto textdata = std::dynamic_pointer_cast<TextData>(data);
    ImGui::TextUnformatted(textdata->Label.c_str());
}

void jif::ElementButton::Show(ResourceManager &resources, JIFCorePtr core, std::map<std::string, std::string> &fields, ViewElementDataPtr &data) const
{
    (void)resources;
    (void)core;
    (void)fields;

    if (!data)
    {
        auto buttondata = std::make_shared<ButtonData>();

        auto value =
            (TextValue[0] == '$')
                ? fields[TextValue.substr(1)]
                : TextValue;

        if (TextSource == "value")
        {
            buttondata->Label = value;
        }
        else if (TextSource == "file")
        {
        }
        else if (TextSource == "resource")
        {
        }
        else if (TextSource == "ros")
        {
            core->RegisterSubscription<std_msgs::msg::String>(
                value,
                [&label = buttondata->Label](const std_msgs::msg::String &msg)
                {
                    label = msg.data;
                });
        }

        data = buttondata;
    }

    auto buttondata = std::dynamic_pointer_cast<ButtonData>(data);
    if (ImGui::Button(buttondata->Label.c_str()))
        ResourceManager::Action(Action);
}

void jif::ElementImage::Show(ResourceManager &resources, JIFCorePtr core, std::map<std::string, std::string> &fields, ViewElementDataPtr &data) const
{
    (void)resources;
    (void)core;
    (void)fields;

    if (!data)
    {
        auto imagedata = std::make_shared<ImageData>();

        auto value =
            (Value[0] == '$')
                ? fields[Value.substr(1)]
                : Value;

        if (Source == "ros")
        {
            core->RegisterSubscription<sensor_msgs::msg::Image>(
                value,
                [](const sensor_msgs::msg::Image &msg)
                {
                    std::cout << "Got image encoding: " << msg.encoding << " width: " << msg.width << " height: " << msg.height << " step: " << msg.step << " is_bigendian: " << msg.is_bigendian << std::endl;
                });
        }

        data = imagedata;
    }

    auto imagedata = std::dynamic_pointer_cast<ImageData>(data);
    ImGui::Image(imagedata->TextureID, imagedata->Size);
}
