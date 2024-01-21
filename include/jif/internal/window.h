#pragma once

#include <GLFW/glfw3.h>

namespace jif
{
    class Window
    {
    public:
        Window(int width, int height, const char *title, const char *iconname);

        operator bool() const;

        bool Spin();

    private:
        int Error(const char *format, ...);

    private:
        int m_Width, m_Height;
        GLFWwindow *m_GLFW;
    };
}
