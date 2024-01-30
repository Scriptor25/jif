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
            core->RegisterSubscription<sensor_msgs::msg::CompressedImage>(
                value,
                [imagedata](const sensor_msgs::msg::CompressedImage &msg)
                {
                    int width, height, channels;
                    auto data = stbi_load_from_memory(msg.data.data(), msg.data.size(), &width, &height, &channels, 4);
                    if (!data)
                    {
                        std::cerr << "[ElementImage] Failed to load image from compressed image message data" << std::endl;
                        return;
                    }

                    imagedata->Size.x = width;
                    imagedata->Size.y = height;

                    auto &tex = imagedata->TextureID;
                    if (!tex)
                        glGenTextures(1, &tex);
                    glBindTexture(GL_TEXTURE_2D, tex);
                    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
                    glBindTexture(GL_TEXTURE_2D, 0);

                    stbi_image_free(data);
                });
        }

        data = imagedata;
    }

    auto imagedata = std::dynamic_pointer_cast<ImageData>(data);
    ImGui::Image((ImTextureID)(intptr_t)imagedata->TextureID, imagedata->Size);
}
