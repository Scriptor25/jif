#pragma once

#include <filesystem>
#include <string>

namespace jif
{
    void UnpackJIF(const std::filesystem::path &srcjif, const std::filesystem::path &dstdir);
    void PackJIF(const std::filesystem::path &srcdir, const std::filesystem::path &dstjif);
}
