/*
 * ------------------------------------------------------------
 * "THE BEERWARE LICENSE" (Revision 42):
 * Felix Schreiber wrote this code. As long as you retain this
 * notice, you can do whatever you want with this stuff. If we
 * meet someday, and you think this stuff is worth it, you can
 * buy me a beer in return.
 * ------------------------------------------------------------
 */

#pragma once
#define GLFW_INCLUDE_NONE

#include <functional>
#include <GLFW/glfw3.h>
#include <map>
#include <ostream>
#include <string>
#include <vector>

namespace jif
{
    struct Shortcut
    {
        Shortcut()
        {
        }

        Shortcut(const std::string &key, bool shift, bool ctrl, bool alt, bool super)
            : Key(key), Shift(shift), Ctrl(ctrl), Alt(alt), Super(super)
        {
        }

        std::string ToString() const;

        static Shortcut Parse(const std::string &shortcut);

        std::string Key;
        bool Shift = false;
        bool Ctrl = false;
        bool Alt = false;
        bool Super = false;
    };

    bool operator<(const Shortcut &a, const Shortcut &b);
    std::ostream &operator<<(std::ostream &out, const Shortcut &shortcut);

    class Window
    {
    public:
        Window(int width, int height, const char *title, const char *iconname);

        operator bool() const;

        int GetWidth() const;
        int GetHeight() const;
        GLFWwindow *GetGLFW() const;

        bool Spin() const;
        void MakeCurrent() const;
        void Close() const;

        void Register(const std::function<void(int width, int height)> &callback);
        void Register(const std::function<bool(int key, int scancode, int action, int mods)> &callback);

        void Register(const std::string &shortcut, const std::function<void()> &callback);

        void SetSaved(bool saved);

    private:
        void SetSize(int width, int height);

        static void GLFWWindowSizeCallback(GLFWwindow *window, int width, int height);
        static void GLFWKeyCallback(GLFWwindow *window, int key, int scancode, int action, int mods);

        static std::string GetKeyName(int key, int scancode);

    private:
        int m_Width, m_Height;
        bool m_Saved;
        std::string m_Title;
        GLFWwindow *m_GLFW;

        std::vector<std::function<void(int width, int height)>> m_WindowSizeCallbacks;
        std::vector<std::function<bool(int key, int scancode, int action, int mods)>> m_KeyCallbacks;
        std::map<Shortcut, std::function<void()>> m_Shortcuts;
    };
}
