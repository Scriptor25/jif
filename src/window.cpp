#include <jif/internal/window.h>
#include <stb/stb_image.h>
#include <cstdarg>

jif::Window::Window(int width, int height, const char *title, const char *iconname)
{
    m_Width = width;
    m_Height = height;
    m_GLFW = glfwCreateWindow(width, height, title, NULL, NULL);
    if (!m_GLFW)
    {
        Error("Failed to create window");
        return;
    }

    GLFWimage icon;
    icon.pixels = stbi_load(iconname, &icon.width, &icon.height, NULL, 4);
    glfwSetWindowIcon(m_GLFW, 1, &icon);
    stbi_image_free(icon.pixels);
}

jif::Window::operator bool() const
{
    return m_GLFW;
}

int jif::Window::Error(const char *format, ...)
{
    printf("[Window Error] ");

    va_list ap;
    va_start(ap, format);
    vprintf(format, ap);
    va_end(ap);

    printf("\r\n");

    return 1;
}
