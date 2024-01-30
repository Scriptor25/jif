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
