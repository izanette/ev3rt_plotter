#include "ev3api.h"
#include "utils.h"

void waitEnterButtonPressed()
{
    while(!ev3_button_is_pressed(ENTER_BUTTON));
    while(ev3_button_is_pressed(ENTER_BUTTON));
}

button_t waitButtonPressed()
{
    while(1)
    {
        for(int i = 0; i < TNUM_BUTTON; i++)
        {
            if (ev3_button_is_pressed((button_t)i))
                return (button_t)i;
        }
    }
    
    return TNUM_BUTTON;
}

void waitButtonRelease(button_t button)
{
    while(ev3_button_is_pressed(button));
}

bool hasAnyButtonPressed()
{
    for(int i = 0; i < TNUM_BUTTON; i++)
    {
        if (ev3_button_is_pressed((button_t)i))
            return true;
    }
    
    return false;
}

void waitNoButtonPressed()
{
    while(1)
    {
        // wait 10 mili-seconds
        tslp_tsk(10);
        
        int stop = 1;
        for(int i = 0; i < TNUM_BUTTON; i++)
        {
            if (ev3_button_is_pressed((button_t)i))
                stop = 0;
        }
        if (stop) break;
    }
}

void clearScreen()
{
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
}

void print(int line, const char* msg)
{
    // Draw information
    lcdfont_t font = EV3_FONT_MEDIUM;
    ev3_lcd_set_font(font);
    int32_t fontw, fonth;
    ev3_font_get_size(font, &fontw, &fonth);
    const char* lcdclean = "                    ";
    
    ev3_lcd_draw_string(lcdclean, 0, fonth * line);
    ev3_lcd_draw_string(msg, 0, fonth * line);
}
