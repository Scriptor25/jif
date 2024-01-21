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

namespace jif
{
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
        void Register(const std::function<void(int key, int scancode, int action, int mods)> &callback);

    private:
        void SetSize(int width, int height);

        static int Error(const char *format, ...);
        static int Warning(const char *format, ...);

        static void GLFWWindowSizeCallback(GLFWwindow *window, int width, int height);
        static void GLFWKeyCallback(GLFWwindow *window, int key, int scancode, int action, int mods);

    private:
        int m_Width, m_Height;
        GLFWwindow *m_GLFW;

        std::vector<std::function<void(int width, int height)>> m_WindowSizeCallbacks;
        std::vector<std::function<void(int key, int scancode, int action, int mods)>> m_KeyCallbacks;
    };
}
