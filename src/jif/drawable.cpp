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
