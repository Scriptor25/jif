/*
 * ------------------------------------------------------------
 * "THE BEERWARE LICENSE" (Revision 42):
 * Felix Schreiber wrote this code. As long as you retain this
 * notice, you can do whatever you want with this stuff. If we
 * meet someday, and you think this stuff is worth it, you can
 * buy me a beer in return.
 * ------------------------------------------------------------
 */

#include <cstdarg>
#include <iostream>
#include <jif/window.h>
#include <sstream>
#include <stb/stb_image.h>
#include <string>

std::string jif::Shortcut::ToString() const
{
    return std::string(Ctrl ? "ctrl+" : "") + std::string(Alt ? "alt+" : "") + std::string(Shift ? "shift+" : "") + std::string(Super ? "super+" : "") + Key;
}

jif::Shortcut jif::Shortcut::Parse(const std::string &shortcut)
{
    std::stringstream stream(shortcut);
    std::string segment;

    Shortcut sc;
    while (std::getline(stream, segment, '+'))
    {
        if (segment == "shift")
        {
            sc.Shift = true;
            continue;
        }
        if (segment == "ctrl")
        {
            sc.Ctrl = true;
            continue;
        }
        if (segment == "alt")
        {
            sc.Alt = true;
            continue;
        }
        if (segment == "super")
        {
            sc.Super = true;
            continue;
        }

        sc.Key = segment;
    }

    return sc;
}

bool jif::operator<(const Shortcut &a, const Shortcut &b)
{
    auto astr = a.ToString();
    auto bstr = b.ToString();
    return astr < bstr;
}

std::ostream &jif::operator<<(std::ostream &out, const Shortcut &shortcut)
{
    return out << shortcut.ToString();
}

jif::Window::Window(int width, int height, const char *title, const char *iconname)
{
    m_Width = width;
    m_Height = height;

    glfwDefaultWindowHints();
    m_GLFW = glfwCreateWindow(width, height, title, NULL, NULL);
    if (!m_GLFW)
    {
        std::cerr << "[Window] Failed to create window for width " << width << " height " << height << " title '" << title << "'" << std::endl;
        return;
    }

    glfwSetWindowUserPointer(m_GLFW, this);

    GLFWimage icon;
    icon.pixels = stbi_load(iconname, &icon.width, &icon.height, NULL, 4);
    if (!icon.pixels)
        std::cerr << "Failed to load icon '" << iconname << "'" << std::endl;
    glfwSetWindowIcon(m_GLFW, 1, &icon);
    stbi_image_free(icon.pixels);

    glfwSetWindowSizeCallback(m_GLFW, GLFWWindowSizeCallback);
    glfwSetKeyCallback(m_GLFW, GLFWKeyCallback);
}

jif::Window::operator bool() const
{
    return m_GLFW;
}

int jif::Window::GetWidth() const
{
    return m_Width;
}

int jif::Window::GetHeight() const
{
    return m_Height;
}

GLFWwindow *jif::Window::GetGLFW() const
{
    return m_GLFW;
}

bool jif::Window::Spin() const
{
    glfwSwapBuffers(m_GLFW);
    glfwPollEvents();

    return !glfwWindowShouldClose(m_GLFW);
}

void jif::Window::MakeCurrent() const
{
    glfwMakeContextCurrent(m_GLFW);
}

void jif::Window::Close() const
{
    glfwSetWindowShouldClose(m_GLFW, GLFW_TRUE);
}

void jif::Window::Register(const std::function<void(int width, int height)> &callback)
{
    m_WindowSizeCallbacks.push_back(callback);
}

void jif::Window::Register(const std::function<bool(int key, int scancode, int action, int mods)> &callback)
{
    m_KeyCallbacks.push_back(callback);
}

void jif::Window::Register(const std::string &shortcut, const std::function<void()> &callback)
{
    std::stringstream stream(shortcut);
    std::string segment;

    while (std::getline(stream, segment, '|'))
    {
        auto sc = Shortcut::Parse(segment);
        m_Shortcuts[sc] = callback;
    }
}

void jif::Window::SetSize(int width, int height)
{
    m_Width = width;
    m_Height = height;
}

void jif::Window::GLFWWindowSizeCallback(GLFWwindow *window, int width, int height)
{
    auto self = (Window *)glfwGetWindowUserPointer(window);
    self->SetSize(width, height);

    for (auto callback : self->m_WindowSizeCallbacks)
        callback(width, height);
}

void jif::Window::GLFWKeyCallback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    auto self = (Window *)glfwGetWindowUserPointer(window);

    if (action == GLFW_RELEASE)
    {
        bool shift = (mods & GLFW_MOD_SHIFT) != 0;
        bool ctrl = (mods & GLFW_MOD_CONTROL) != 0;
        bool alt = (mods & GLFW_MOD_ALT) != 0;
        bool super = (mods & GLFW_MOD_SUPER) != 0;

        Shortcut sc(GetKeyName(key, scancode), shift, ctrl, alt, super);
        if (auto scf = self->m_Shortcuts[sc])
        {
            scf();
            return;
        }
    }

    for (auto callback : self->m_KeyCallbacks)
        if (callback(key, scancode, action, mods))
            return;
}

std::string jif::Window::GetKeyName(int key, int scancode)
{
    if (auto str = glfwGetKeyName(key, scancode))
        return str;

    switch (key)
    {
    case GLFW_KEY_ESCAPE:
        return "esc";

    default:
        return "";
    }
}
