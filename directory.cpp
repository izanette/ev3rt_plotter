#include "ev3api.h"
#include "utils.h"
#include "directory.h"
#include <string.h>

Directory::Directory(const char* path)
: m_dirId(E_PAR)
, m_pos(0)
, m_size(0)
{
    this->open(path);
}

Directory::~Directory()
{
    this->close();
}

void Directory::close()
{
    if (m_dirId > 0)
        ev3_sdcard_closedir(m_dirId);
    m_dirId = E_PAR;
}

ER_ID Directory::open(const char* path)
{
    close();
    
    strcpy(m_path, path);
    
    return open();
}

ER_ID Directory::open()
{
    m_size = calculateSize();
    
    m_dirId = ev3_sdcard_opendir(m_path);
    m_pos = 0;
    
    return m_dirId;
}

fileinfo_t* Directory::getFile(int i)
{
    if (m_dirId <= 0 || i < 0 || i >= m_size) return NULL;
    
    if (i < m_pos)
    {
        close();
        open();
    }
    
    ER er = E_OK;
    do
    {
        er = ev3_sdcard_readdir(m_dirId, &m_fileInfo);
        m_pos++;
    }
    while(er == E_OK && i >= m_pos);
    
    return &m_fileInfo;
}

int Directory::calculateSize()
{
    int result = 0;
    ER_ID dirId = ev3_sdcard_opendir(m_path);
    
    if (dirId <= 0)
        return result;

    fileinfo_t fileInfo;
    while (ev3_sdcard_readdir(dirId, &fileInfo) == E_OK)
        result++;
    
    ev3_sdcard_closedir(dirId);
    
    return result;
}
