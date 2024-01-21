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

std::filesystem::path jif::ResourceManager::m_Root;

void jif::ResourceManager::Init(const std::filesystem::path &executable)
{
    m_Root = executable.parent_path();
}

std::filesystem::path jif::ResourceManager::GetResource(const char *name)
{
    return m_Root / "res" / name;
}
