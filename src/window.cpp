/*
 * ------------------------------------------------------------
 * "THE BEERWARE LICENSE" (Revision 42):
 * Felix Schreiber wrote this code. As long as you retain this
 * notice, you can do whatever you want with this stuff. If we
 * meet someday, and you think this stuff is worth it, you can
 * buy me a beer in return.
 * ------------------------------------------------------------
 */

#include <jif/internal/window.h>
#include <stb/stb_image.h>

#include <cstdarg>

jif::Window::Window(int width, int height, const char *title, const char *iconname)
{
    m_Width = width;
    m_Height = height;

    glfwDefaultWindowHints();
    m_GLFW = glfwCreateWindow(width, height, title, NULL, NULL);
    if (!m_GLFW)
    {
        Error("Failed to create window");
        return;
    }

    glfwSetWindowUserPointer(m_GLFW, this);

    GLFWimage icon;
    icon.pixels = stbi_load(iconname, &icon.width, &icon.height, NULL, 4);
    if (!icon.pixels)
        Warning("Failed to load icon");
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

void jif::Window::Register(const std::function<void(int key, int scancode, int action, int mods)> &callback)
{
    m_KeyCallbacks.push_back(callback);
}

void jif::Window::SetSize(int width, int height)
{
    m_Width = width;
    m_Height = height;
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

int jif::Window::Warning(const char *format, ...)
{
    printf("[Window Warning] ");

    va_list ap;
    va_start(ap, format);
    vprintf(format, ap);
    va_end(ap);

    printf("\r\n");

    return 2;
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

    if (key == GLFW_KEY_ESCAPE && action == GLFW_RELEASE)
        self->Close();

    for (auto callback : self->m_KeyCallbacks)
        callback(key, scancode, action, mods);
}
