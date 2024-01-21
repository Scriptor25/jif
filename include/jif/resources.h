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

#include <filesystem>

namespace jif
{
    class Resources
    {
    public:
        static void Init(const std::filesystem::path &executable);
        static std::filesystem::path GetResource(const char *name);

    private:
        static std::filesystem::path m_Root;
    };
}
