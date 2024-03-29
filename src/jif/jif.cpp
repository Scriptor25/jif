/*
 * ------------------------------------------------------------
 * "THE BEERWARE LICENSE" (Revision 42):
 * Felix Schreiber wrote this code. As long as you retain this
 * notice, you can do whatever you want with this stuff. If we
 * meet someday, and you think this stuff is worth it, you can
 * buy me a beer in return.
 * ------------------------------------------------------------
 */

#include <iostream>
#include <jif/jif.h>
#include <kubazip/zip.h>

bool jif::UnpackJIF(const std::filesystem::path &srcjif, const std::filesystem::path &dstdir)
{
    auto err = zip_extract(srcjif.c_str(), dstdir.c_str(), NULL, NULL);
    if (err != 0)
    {
        std::cerr << "[jif] Failed to extract zip " << srcjif << " into dir " << dstdir << ": error code " << err << std::endl;
        return false;
    }
    return true;
}

bool jif::PackJIF(const std::filesystem::path &srcdir, const std::filesystem::path &dstjif)
{
    auto layoutjson = srcdir / "layout.json";
    auto imguiini = srcdir / "imgui.ini";

    const char *files[]{
        layoutjson.c_str(),
        imguiini.c_str(),
    };

    auto err = zip_create(dstjif.c_str(), files, 2);
    if (err != 0)
    {
        std::cerr << "[jif] Failed to create zip " << dstjif << " from dir " << srcdir << ": error code " << err << std::endl;
        return false;
    }
    return true;
}
