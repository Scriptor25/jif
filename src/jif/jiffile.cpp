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
    const char *files[]{
        (srcdir / "").c_str(),
        (srcdir / "").c_str(),
        (srcdir / "").c_str(),
    };

    auto err = zip_create(dstjif.c_str(), files, 3);
    if (err != 0)
        std::cerr << "Failed to create zip " << dstjif << " from dir " << srcdir << ": error code " << err << std::endl;
}
