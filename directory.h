#ifndef __DIRECTORY_H__
#define __DIRECTORY_H__

#include "ev3api.h"

class Directory
{
public:
    Directory(const char* path);
    ~Directory();
    
    void close();
    ER_ID open(const char* path);
    ER_ID open();
    fileinfo_t* getFile(int i);
    
    int size() { return m_size; }
    int getDirId() { return m_dirId; }
    
private:
    int calculateSize();
    
private:
    char m_path[TMAX_FILENAME_LEN+1];
    fileinfo_t m_fileInfo;
    ER_ID m_dirId;
    int m_pos;
    int m_size;
};

#endif // __DIRECTORY_H__