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

void jif::JIFManager::CreateView(const std::string &label, const std::string &type)
{
    static size_t id = 0;
    while (m_Views.count(std::to_string(id++)))
        ;
    m_Views[std::to_string(id)] = std::make_shared<JIFView>(std::to_string(id), label, type);
    std::cout << "[JIFManager] Created view '" << label << "' (" << id << ") type '" << type << "'" << std::endl;
    id++;

    m_HasChanges = true;
}

void jif::JIFManager::CreateView(const AddViewWizardData &data)
{
    static size_t id = 0;
    while (m_Views.count(std::to_string(id++)))
        ;
    auto view = std::make_shared<JIFView>(std::to_string(id), data.Label, data.Type->Id, data.Fields);
    m_Views[std::to_string(id)] = view;
    std::cout << "[JIFManager] Created view '" << data.Label << "' (" << id << ") type '" << data.Type->Id << std::endl;
    id++;

    m_HasChanges = true;
}

void jif::JIFManager::AddView(const std::string &id, const std::string &name, const std::string &type, const std::map<std::string, std::string> &fields)
{
    m_Views[id] = std::make_shared<JIFView>(id, name, type, fields);
    std::cout << "[JIFManager] Added view '" << name << "' (" << id << ") type '" << type << "'" << std::endl;
}
