/*
 * ------------------------------------------------------------
 * "THE BEERWARE LICENSE" (Revision 42):
 * Felix Schreiber wrote this code. As long as you retain this
 * notice, you can do whatever you want with this stuff. If we
 * meet someday, and you think this stuff is worth it, you can
 * buy me a beer in return.
 * ------------------------------------------------------------
 */

#include <GL/glew.h>
#include <imgui/imgui.h>
#include <jif/resource.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <stb/stb_image.h>
#include <std_msgs/msg/string.hpp>

jif::TextData::~TextData()
{
    if (!Topic.empty())
        Manager.Schedule([core = Core, topic = Topic]()
                         { core->UnregisterSubscription(topic); });
}

jif::ButtonData::~ButtonData()
{
    if (!Topic.empty())
        Manager.Schedule([core = Core, topic = Topic]()
                         { core->UnregisterSubscription(topic); });
}

jif::ImageData::~ImageData()
{
    if (!Topic.empty())
        Manager.Schedule([core = Core, topic = Topic]()
                         { core->UnregisterSubscription(topic); });
    glDeleteTextures(1, &TextureID);
    TextureID = 0;
}

void jif::ElementText::Show(JIFManager &manager, ResourceManager &resources, JIFCorePtr core, std::map<std::string, std::string> &fields, ViewElementDataPtr &data) const
{
    (void)resources;
    (void)fields;

    if (!data)
    {
        auto textdata = std::make_shared<TextData>(manager, core);

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
            textdata->Topic = value;
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

void jif::ElementButton::Show(JIFManager &manager, ResourceManager &resources, JIFCorePtr core, std::map<std::string, std::string> &fields, ViewElementDataPtr &data) const
{
    (void)resources;
    (void)fields;

    if (!data)
    {
        auto buttondata = std::make_shared<ButtonData>(manager, core);

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
            buttondata->Topic = value;
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

void jif::ElementImage::Show(JIFManager &manager, ResourceManager &resources, JIFCorePtr core, std::map<std::string, std::string> &fields, ViewElementDataPtr &data) const
{
    (void)resources;
    (void)fields;

    if (!data)
    {
        auto imagedata = std::make_shared<ImageData>(manager, core);

        manager.Schedule(
            [imagedata]()
            {
                glGenTextures(1, &imagedata->TextureID);
                glBindTexture(GL_TEXTURE_2D, imagedata->TextureID);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            });

        auto value =
            (Value[0] == '$')
                ? fields[Value.substr(1)]
                : Value;

        if (Source == "ros")
        {
            imagedata->Topic = value;
            core->RegisterSubscription<sensor_msgs::msg::CompressedImage>(
                value,
                [&manager, imagedata](const sensor_msgs::msg::CompressedImage &msg)
                {
                    manager.Schedule(
                        [msg, imagedata]()
                        {
                            int width, height, channels;
                            auto data = stbi_load_from_memory(msg.data.data(), msg.data.size(), &width, &height, &channels, 3);
                            if (!data)
                            {
                                std::cerr << "[ElementImage] Failed to load image from compressed image message data" << std::endl;
                                return;
                            }

                            imagedata->Size.x = width;
                            imagedata->Size.y = height;

                            glBindTexture(GL_TEXTURE_2D, imagedata->TextureID);
                            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
                            glBindTexture(GL_TEXTURE_2D, 0);

                            stbi_image_free(data);
                        });
                });
        }

        data = imagedata;
    }

    auto imagedata = std::dynamic_pointer_cast<ImageData>(data);

    auto iw = imagedata->Size.x;
    auto ih = imagedata->Size.y;
    auto region = ImGui::GetContentRegionAvail();
    auto ww = region.x;
    auto wh = region.y;

    auto scale = std::min(ww / iw, wh / ih);
    ImVec2 size(scale * iw, scale * ih);
    ImGui::Image((ImTextureID)(intptr_t)imagedata->TextureID, size);
}
