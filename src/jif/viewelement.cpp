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
    Manager.SchedulePre([core = Core, topic = Topic]()
                        { core->UnregisterSubscription(topic); });
}

jif::ButtonData::~ButtonData()
{
    Manager.SchedulePre([core = Core, topic = Topic]()
                        { core->UnregisterSubscription(topic); });
}

jif::ImageData::~ImageData()
{
    Manager.SchedulePre([core = Core, topic = Topic]()
                        { core->UnregisterSubscription(topic); });
    glDeleteTextures(1, &TextureID);
    TextureID = 0;
}

jif::JoystickData::~JoystickData()
{
    Manager.SchedulePre(
        [core = Core, status = StatusTopic, dest = DestTopic]()
        {
            core->UnregisterSubscription(status);
            core->UnregisterPublisher(dest);
        });
}

static std::string get(const std::string &value, std::map<std::string, std::string> &fields)
{
    return (value[0] == '$')
               ? fields[value.substr(1)]
               : value;
}

void jif::ElementText::Show(JIFManager &manager, ResourceManager &resources, Window &window, ShowArgs &args) const
{
    if (!args.Data)
    {
        auto data = std::make_shared<TextData>(manager, args.Core);

        auto value = get(Value, args.Fields);

        if (Source == "value")
        {
            data->Label = value;
        }
        else if (Source == "file")
        {
        }
        else if (Source == "resource")
        {
        }
        else if (Source == "ros")
        {
            data->Topic = value;
            args.Core->RegisterSubscription<std_msgs::msg::String>(
                value,
                [&label = data->Label](const std_msgs::msg::String &msg)
                {
                    label = msg.data;
                });
        }

        args.Data = data;
    }

    auto data = std::dynamic_pointer_cast<TextData>(args.Data);
    ImGui::TextUnformatted(data->Label.c_str());
}

void jif::ElementButton::Show(JIFManager &manager, ResourceManager &resources, Window &window, ShowArgs &args) const
{
    if (!args.Data)
    {
        auto data = std::make_shared<ButtonData>(manager, args.Core);

        auto value = get(TextValue, args.Fields);

        if (TextSource == "value")
        {
            data->Label = value;
        }
        else if (TextSource == "file")
        {
        }
        else if (TextSource == "resource")
        {
        }
        else if (TextSource == "ros")
        {
            data->Topic = value;
            args.Core->RegisterSubscription<std_msgs::msg::String>(
                value,
                [&label = data->Label](const std_msgs::msg::String &msg)
                {
                    label = msg.data;
                });
        }

        args.Data = data;
    }

    auto data = std::dynamic_pointer_cast<ButtonData>(args.Data);
    if (ImGui::Button(data->Label.c_str()))
        ResourceManager::Action(Action, manager);
}

void jif::ElementImage::Show(JIFManager &manager, ResourceManager &resources, Window &window, ShowArgs &args) const
{
    if (!args.Data)
    {
        auto data = std::make_shared<ImageData>(manager, args.Core);

        manager.SchedulePre(
            [data]()
            {
                glGenTextures(1, &data->TextureID);
                glBindTexture(GL_TEXTURE_2D, data->TextureID);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            });

        auto value = get(Value, args.Fields);

        if (Source == "ros")
        {
            data->Topic = value;
            args.Core->RegisterSubscription<sensor_msgs::msg::CompressedImage>(
                value,
                [&manager, data](const sensor_msgs::msg::CompressedImage &msg)
                {
                    manager.SchedulePre(
                        [msg, data]()
                        {
                            int width, height, channels;
                            auto pixels = stbi_load_from_memory(msg.data.data(), msg.data.size(), &width, &height, &channels, 3);
                            if (!pixels)
                            {
                                std::cerr << "[ElementImage] Failed to load image from compressed image message data" << std::endl;
                                return;
                            }

                            data->Size.x = width;
                            data->Size.y = height;

                            glBindTexture(GL_TEXTURE_2D, data->TextureID);
                            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, pixels);
                            glBindTexture(GL_TEXTURE_2D, 0);

                            stbi_image_free(pixels);
                        });
                });
        }

        args.Data = data;
    }

    auto data = std::dynamic_pointer_cast<ImageData>(args.Data);

    auto iw = data->Size.x;
    auto ih = data->Size.y;
    auto region = ImGui::GetContentRegionAvail();
    auto ww = region.x;
    auto wh = region.y;

    auto scale = std::min(ww / iw, wh / ih);
    ImVec2 size(scale * iw, scale * ih);
    ImGui::Image((ImTextureID)(intptr_t)data->TextureID, size);
}

void jif::ElementJoystick::Show(JIFManager &manager, ResourceManager &resources, Window &window, ShowArgs &args) const
{
    if (!args.Data)
    {
        auto data = std::make_shared<JoystickData>(manager, args.Core);

        auto status = get(Status, args.Fields);
        auto dest = get(Dest, args.Fields);

        data->StatusTopic = status;
        data->DestTopic = dest;

        args.Core->RegisterSubscription<std_msgs::msg::String>(status, [](const std_msgs::msg::String &msg) {});
        args.Core->RegisterPublisher<geometry_msgs::msg::Twist>(dest);

        window.Register(
            [&msg = data->Msg](int key, int scancode, int action, int mods) -> bool
            {
                if (key == GLFW_KEY_W && action == GLFW_PRESS)
                {
                    msg.linear.z = 1;
                    return true;
                }
                if (key == GLFW_KEY_S && action == GLFW_PRESS)
                {
                    msg.linear.z = -1;
                    return true;
                }
                if (key == GLFW_KEY_D && action == GLFW_PRESS)
                {
                    msg.angular.y = 1;
                    return true;
                }
                if (key == GLFW_KEY_A && action == GLFW_PRESS)
                {
                    msg.angular.y = -1;
                    return true;
                }
                if (key == GLFW_KEY_Q && action == GLFW_PRESS)
                {
                    msg.linear.x = msg.angular.y = msg.angular.z = msg.linear.x = msg.linear.y = msg.linear.z = 0;
                    return true;
                }
                return false;
            });

        args.Data = data;
    }

    auto data = std::dynamic_pointer_cast<JoystickData>(args.Data);

    args.Core->Publish(data->DestTopic, data->Msg);
}
