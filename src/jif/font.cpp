/*
 * ------------------------------------------------------------
 * "THE BEERWARE LICENSE" (Revision 42):
 * Felix Schreiber wrote this code. As long as you retain this
 * notice, you can do whatever you want with this stuff. If we
 * meet someday, and you think this stuff is worth it, you can
 * buy me a beer in return.
 * ------------------------------------------------------------
 */

#include <jif/resource.h>

bool jif::Font::Load()
{
    std::ifstream stream(Filename);
    if (!stream)
        return false;

    std::string data(std::istreambuf_iterator<char>(stream), {});
    Data = data.data();
    Size = data.size();

    return true;
}

void jif::Font::Free()
{
    free(Data);
    Data = 0;
    Size = 0;
}
