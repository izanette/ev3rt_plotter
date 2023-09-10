#ifndef __CLI_MENU_H__
#define __CLI_MENU_H__

extern FILE *fio;

typedef struct {
    uint8_t     key;
    const char *title;
    ISR         handler;
    intptr_t    exinf;
} CliMenuEntry;

typedef struct {
    const char         *title;
    const CliMenuEntry *entry_tab;
    SIZE                entry_num;
} CliMenu;

void draw_title(const char* title, int offset_x, int offset_y, lcdfont_t font);
void show_cli_menu(const CliMenu *cm, int offset_x, int offset_y, lcdfont_t font);
const CliMenuEntry* select_menu_entry(const CliMenu *cm, int offset_x, int offset_y, lcdfont_t font);
void show_message_box(const char *title, const char *msg);

static inline
void fio_clear_screen() {
    fprintf(fio, "\033[2J\033[;H"); // Clear Screen
}

static inline
void fio_clear_line() {
    fprintf(fio, "\033[2K\033[255D"); // Clear Screen
}

extern const CliMenu climenu_main;

extern int32_t default_menu_font_width;
extern int32_t default_menu_font_height;
#define MENU_FONT (EV3_FONT_MEDIUM)
#define MENU_FONT_WIDTH (default_menu_font_width)
#define MENU_FONT_HEIGHT (default_menu_font_height)

#endif // __CLI_MENU_H__