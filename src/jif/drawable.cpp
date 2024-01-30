#include <jif/resource.h>
#include <stb/stb_image.h>

bool jif::Drawable::Load()
{
    Pixels = stbi_load(Filename.c_str(), &Width, &Height, &Channels, 4);
    return Pixels;
}

void jif::Drawable::Free()
{
    if (Pixels)
        stbi_image_free(Pixels);
    Pixels = 0;
}
