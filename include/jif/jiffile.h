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
#include <string>

namespace jif
{
    void UnpackJIF(const std::filesystem::path &srcjif, const std::filesystem::path &dstdir);
    void PackJIF(const std::filesystem::path &srcdir, const std::filesystem::path &dstjif);
}
