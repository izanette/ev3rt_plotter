#ifndef __SELECT_FILE_H__
#define __SELECT_FILE_H__

#include "ev3api.h"
#include "directory.h"

#define NUM_FILES_PER_PAGE 6

class SelectFile
{
public:
    SelectFile(const char* path);
    fileinfo_t* select();

    int getNumPages();
    int getNumFilesCurrentPage(int pageNum);
    void drawSelection(int selection);
    void drawPage(int pageNum, int selection = 0);
    
private:
    Directory m_dir;
};

#endif // __SELECT_FILE_H__