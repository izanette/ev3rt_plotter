#include "ev3api.h"
#include "utils.h"
#include "selectfile.h"

SelectFile::SelectFile(const char* path)
: m_dir(path)
{}

fileinfo_t* SelectFile::select()
{
    if (m_dir.getDirId() <= 0)
    {
        print(2, "Probs to open dir");
        waitButtonPressed();
        return NULL;
    }
    
    int currentPage = 0;
    int currentSelection = 0;
    int numFilesCurrentPage = getNumFilesCurrentPage(currentPage);
    drawPage(currentPage);
    drawSelection(currentSelection);
    
    while(true)
    {
        button_t button = waitButtonPressed();
        
        switch(button)
        {
        case UP_BUTTON:
            if (currentSelection > 0) 
            {
                currentSelection--;
                drawSelection(currentSelection);
                break;
            }
            // don't break to go to the previous page
            
        case LEFT_BUTTON:
            if (currentPage > 0) 
            {
                currentPage--;
                currentSelection = NUM_FILES_PER_PAGE-1;
                drawPage(currentPage, currentSelection);
                numFilesCurrentPage = getNumFilesCurrentPage(currentPage);
            }
            break;
            
        case DOWN_BUTTON:
            if (currentSelection < numFilesCurrentPage-1)
            {
                currentSelection++;
                drawSelection(currentSelection);
                break;
            }
            // don't break to go to the next page
            
        case RIGHT_BUTTON:
            if (currentPage < this->getNumPages() - 1) 
            {
                currentPage++;
                currentSelection = 0;
                drawPage(currentPage, currentSelection);
                numFilesCurrentPage = getNumFilesCurrentPage(currentPage);
            }
            break;
            
        case BACK_BUTTON:
            waitButtonRelease(button);
            return NULL;
            
        case ENTER_BUTTON:
            waitButtonRelease(button);
            return m_dir.getFile(currentPage * NUM_FILES_PER_PAGE + currentSelection);
        
        default:
            break;
        }
        
        waitButtonRelease(button);
    }
}

int SelectFile::getNumPages()
{
    return (m_dir.size() + NUM_FILES_PER_PAGE - 1) / NUM_FILES_PER_PAGE;
}

int SelectFile::getNumFilesCurrentPage(int pageNum)
{
    int result = m_dir.size() - pageNum * NUM_FILES_PER_PAGE;
    
    return (result < NUM_FILES_PER_PAGE) ? result : NUM_FILES_PER_PAGE;
}

void SelectFile::drawSelection(int selection)
{
    lcdfont_t font = EV3_FONT_MEDIUM;
    ev3_lcd_set_font(font);
    int32_t fontw, fonth;
    ev3_font_get_size(font, &fontw, &fonth);
    
    ev3_lcd_fill_rect(0, fonth, fontw, EV3_LCD_HEIGHT-fonth, EV3_LCD_WHITE); // clear previous selection
    ev3_lcd_draw_string(">", 0, fonth * (selection+1));
}

void SelectFile::drawPage(int pageNum, int selection /*= 0*/)
{
    lcdfont_t font = EV3_FONT_MEDIUM;
    ev3_lcd_set_font(font);
    char buf[50];
  
    if (m_dir.getDirId() <= 0) return;
    
    clearScreen();
    print(0, "Select File");
    
    int line = 1;
    for(int fileIdx = pageNum * NUM_FILES_PER_PAGE; fileIdx < m_dir.size(); fileIdx++)
    {
        fileinfo_t* fi = m_dir.getFile(fileIdx);
        sprintf(buf, " %s", fi->name);
        print(line++, buf);
    }
    
    drawSelection(selection);
    
    sprintf(buf, " [PAGE %d / %d]", pageNum+1, getNumPages());
    print(7, buf);
}
