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
#include <jif/manager.h>

void jif::JIFManager::CreateView(const std::string &label, const ViewTypePtr &type)
{
    static size_t id = 0;
    while (m_Views.count(std::to_string(id)))
        ++id;
    m_Views[std::to_string(id)] = std::make_shared<JIFView>(std::to_string(id), label, type);
    std::cout << "[JIFManager] Created view '" << label << "' (" << id << ") type '" << type->Id << "'" << std::endl;

    SetHasChanges();
}

void jif::JIFManager::CreateView(const AddViewWizardData &data)
{
    static size_t id = 0;
    while (m_Views.count(std::to_string(id)))
        ++id;
    m_Views[std::to_string(id)] = std::make_shared<JIFView>(std::to_string(id), data.Label, data.Type, data.Fields);
    std::cout << "[JIFManager] Created view '" << data.Label << "' (" << id << ") type '" << data.Type->Id << std::endl;

    SetHasChanges();
}

void jif::JIFManager::AddView(const std::string &id, const std::string &name, const ViewTypePtr &type, const std::map<std::string, std::string> &fields)
{
    m_Views[id] = std::make_shared<JIFView>(id, name, type, fields);
    std::cout << "[JIFManager] Added view '" << name << "' (" << id << ") type '" << type->Id << "'" << std::endl;
}
