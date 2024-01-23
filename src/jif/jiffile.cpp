#include <iostream>
#include <jif/jiffile.h>
#include <kubazip/zip.h>

void jif::UnpackJIF(const std::filesystem::path &srcjif, const std::filesystem::path &dstdir)
{
    auto err = zip_extract(srcjif.c_str(), dstdir.c_str(), NULL, NULL);
    if (err != 0)
        std::cerr << "Failed to extract zip " << srcjif << " into dir " << dstdir << ": error code " << err << std::endl;
}

void jif::PackJIF(const std::filesystem::path &srcdir, const std::filesystem::path &dstjif)
{
    auto layoutjson = srcdir / "layout.json";
    auto imguiini = srcdir / "imgui.ini";

    const char *files[]{
        layoutjson.c_str(),
        imguiini.c_str(),
    };

    auto err = zip_create(dstjif.c_str(), files, 2);
    if (err != 0)
        std::cerr << "Failed to create zip " << dstjif << " from dir " << srcdir << ": error code " << err << std::endl;
}
