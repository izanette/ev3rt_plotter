/*
 * cli_menu.c
 *
 *  Created on: 08/Sep/2018
 *      Author: izanette
 */

#include "ev3api.h"
#include "app.h"
#include "utils.h"
#include "cli_menu.h"
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

FILE *fio;

int32_t default_menu_font_width;
int32_t default_menu_font_height;

static void draw_menu_entry(const CliMenu *cm, int index, bool selected, int offset_x, int offset_y, lcdfont_t font) {
    // Get font information
    int32_t fontw, fonth;
    ev3_lcd_set_font(font);
    ev3_font_get_size(font, &fontw, &fonth);

    ev3_lcd_draw_string(cm->entry_tab[index].title, offset_x + fontw + 2, offset_y + fonth * index);
    if (selected)
        ev3_lcd_draw_string(">", offset_x, offset_y + fonth * index);
    else
        ev3_lcd_draw_string(" ", offset_x, offset_y + fonth * index);
}

void draw_title(const char* title, int offset_x, int offset_y, lcdfont_t font)
{
    // Get font information
    int32_t fontw, fonth;
    ev3_lcd_set_font(font);
    ev3_font_get_size(font, &fontw, &fonth);
    
    unsigned int lcd_width = EV3_LCD_WIDTH - offset_x;
    
    // Draw title
    if (lcd_width > strlen(title) * fontw)
        offset_x += (EV3_LCD_WIDTH - offset_x - strlen(title) * fontw) / 2;
    ev3_lcd_draw_string(title, offset_x, offset_y);
    ev3_lcd_draw_line(0, offset_y + fonth - 1, EV3_LCD_WIDTH, offset_y + fonth - 1);
}

void show_cli_menu(const CliMenu *cm, int offset_x, int offset_y, lcdfont_t font) {
    // Clear menu area
    ev3_lcd_fill_rect(offset_x, offset_y, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);

    // Draw title
    draw_title(cm->title, offset_x, offset_y, font);
}

const CliMenuEntry* select_menu_entry(const CliMenu *cm, int offset_x, int offset_y, lcdfont_t font) {
    // Get font information
    int32_t fontw, fonth;
    ev3_lcd_set_font(font);
    ev3_font_get_size(font, &fontw, &fonth);

    // Draw menu entries
    for(SIZE i = 0; i < cm->entry_num; ++i)
        draw_menu_entry(cm, i, false, offset_x, offset_y, font);

    int current = 0;

    bool select_finished = false;
    while (!select_finished) {
        draw_menu_entry(cm, current, true, offset_x, offset_y, font);
        while(1) {
            if (ev3_button_is_pressed(UP_BUTTON)) {
                while(ev3_button_is_pressed(UP_BUTTON));
                draw_menu_entry(cm, current, false, offset_x, offset_y, font);
                current = (current - 1) % cm->entry_num;
                break;
            }
            if (ev3_button_is_pressed(DOWN_BUTTON)) {
                while(ev3_button_is_pressed(DOWN_BUTTON));
                draw_menu_entry(cm, current, false, offset_x, offset_y, font);
                current = (current + 1) % cm->entry_num;
                break;
            }
            if (ev3_button_is_pressed(ENTER_BUTTON)) {
                while(ev3_button_is_pressed(ENTER_BUTTON));
                select_finished = true;
                break;
            }
            if (ev3_button_is_pressed(BACK_BUTTON)) {
                while(ev3_button_is_pressed(BACK_BUTTON));
                for(SIZE i = 0; i < cm->entry_num; ++i) {
                    if(toupper(cm->entry_tab[i].key) == toupper((int8_t)'Q')) { // BACK => 'Q'
                        current = i;
                        select_finished = true;
                    }
                }
                break;
            }
        }
    }

    assert(current >= 0 && (unsigned int)current < cm->entry_num);
    return &cm->entry_tab[current];
}

void show_message_box(const char *title, const char *msg) {
    int offset_x = 0, offset_y = 0;

    // Clear menu area
    ev3_lcd_fill_rect(offset_x, offset_y, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);

    unsigned int lcd_width = EV3_LCD_WIDTH - offset_x;
    
    // Draw title
    if (lcd_width > strlen(title) * MENU_FONT_WIDTH)
        offset_x += (EV3_LCD_WIDTH - offset_x - strlen(title) * MENU_FONT_WIDTH) / 2;
    ev3_lcd_draw_string(title, offset_x, offset_y);
    ev3_lcd_draw_line(0, offset_y + MENU_FONT_HEIGHT - 1, EV3_LCD_WIDTH, offset_y + MENU_FONT_HEIGHT - 1);
    offset_y += MENU_FONT_HEIGHT;

    // Draw message
    offset_x = MENU_FONT_WIDTH, offset_y += MENU_FONT_HEIGHT;
    while (*msg != '\0') {
        if (*msg == '\n' || offset_x + MENU_FONT_WIDTH > EV3_LCD_WIDTH) { // newline
            offset_x = MENU_FONT_WIDTH;
            offset_y += MENU_FONT_HEIGHT;
        }
        if (*msg != '\n') {
            char buf[2] = { *msg, '\0' };
            ev3_lcd_draw_string(buf, offset_x, offset_y);
            offset_x += MENU_FONT_WIDTH;
        }
        msg++;
    }

    // Draw & wait 'OK' button
    ev3_lcd_draw_string("--- OK ---", (EV3_LCD_WIDTH - strlen("--- OK ---") * MENU_FONT_WIDTH) / 2, EV3_LCD_HEIGHT - MENU_FONT_HEIGHT - 5);
    while(!ev3_button_is_pressed(ENTER_BUTTON));
    while(ev3_button_is_pressed(ENTER_BUTTON));
}
