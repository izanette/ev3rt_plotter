/**
 * Plott3r
 *
 */

#include "ev3api.h"
#include "app.h"
#include "cli_menu.h"
#include "motion.h"
#include "utils.h"
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

void main_task(intptr_t unused)
{
    char buf[100];
    
    print(0, "App: Plotter v03");
    sprintf(buf, "Port%c:X motor", 'A' + x_motor);
    print(1, buf);
    sprintf(buf, "Port%c:Y motor", 'A' + y_motor);
    print(2, buf);
    sprintf(buf, "Port%c:Z motor", 'A' + z_motor);
    print(3, buf);
    sprintf(buf, "Port%c:X stop",  '1' + xend_sensor);
    print(5, buf);
    sprintf(buf, "Port%c:Y stop",  '1' + yend_sensor);
    print(6, buf);

    // Configure motors
    ev3_motor_config(x_motor, LARGE_MOTOR);
    ev3_motor_config(y_motor, LARGE_MOTOR);
    ev3_motor_config(z_motor, MEDIUM_MOTOR);
    
    // Configure sensors
    ev3_sensor_config(xend_sensor, TOUCH_SENSOR);
    ev3_sensor_config(yend_sensor, TOUCH_SENSOR);

    ev3_font_get_size(MENU_FONT, &default_menu_font_width, &default_menu_font_height);
    
    waitEnterButtonPressed();
    
    while(1) {
        show_cli_menu(&climenu_main, 0, 0, MENU_FONT);
        const CliMenuEntry *cme = select_menu_entry(&climenu_main, 0, MENU_FONT_HEIGHT, MENU_FONT);
        if(cme != NULL) {
            assert(cme->handler != NULL);
            cme->handler(cme->exinf);
        }
    }
}
