/*
  menu.c - MiSTeryNano menu based in u8g2

  This version includes the old static MiSTeryNano type of menu
  as well as the new config driven one.

*/
  
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <ff.h>
#include <diskio.h>

#ifdef ESP_PLATFORM
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#else
#include <FreeRTOS.h>
#include <timers.h>
#include <task.h>
#include <queue.h>
#endif

#include "sdc.h"
#include "osd.h"
#include "inifile.h"
#include "menu.h"
#include "sysctrl.h"
#include "debug.h"
#include "mcu_hw.h"

#ifdef ENABLE_BLUETOOTH
#include "bluetooth.h"
#endif

// this is the u8g2_font_helvR08_te with any trailing
// spaces removed
#include "font_helvR08_te.c"

// some constants for arrangement
// The OSD (currently) is 64 pixel high. To allow for a proper
// box around a text line, it needs to be 12 pixels high. A total
// of five lines is 5*12 = 60 + title seperation line
#define MENU_LINE_Y      13   // y pos of seperator line
#define MENU_ENTRY_H     12   // height of regular menu entries
#define MENU_ENTRY_BASE   9   // font baseline offset


#define MENU_FORM_FSEL           -1

#define MENU_ENTRY_INDEX_ID       0
#define MENU_ENTRY_INDEX_LABEL    1
#define MENU_ENTRY_INDEX_FORM     2
#define MENU_ENTRY_INDEX_OPTIONS  2
#define MENU_ENTRY_INDEX_VARIABLE 3

// menu types
#define MENU_TYPE_MENU          0
#define MENU_TYPE_FILESELECTOR  1
#define MENU_TYPE_CUSTOM        2  // only internally used

typedef struct config_custom_S {
  char *label;

  // Pointer to function the returns the number of active entries in
  // this custom dialog. This is the number of "lines" that can be
  // selected with the up and down keys
  int (*length)(void); 
  
  // Pointer to function that draws the custom contents
  void (*draw)(void); 
  
} config_custom_t;

/* new menu state */
typedef struct menu_state {
  int type;
  int selected; 
  int scroll;
  union {
    const config_menu_t *menu;  
    const char *label;             // used for file/image selector, only
    const config_custom_t *custom;  
  };
  
  // file selector related
  sdc_dir_entry_t *dir;

  struct menu_state *prev;
} menu_state_t;

typedef struct {
  char index;
  char **ext;
  config_action_t *action;  // action to be run afterwards

  // string and icon to be used if nothing is selected
  // by default this is "[X] No Disk"
  char *none_str;
  unsigned char *none_icn;

  // set if this is the list of a <cfgselector> and not a file selector
  const config_cfgsel_t *cfgsel;
} fsel_state_t;

// menu state is a stack since as the menu is hierarchical
static menu_state_t *menu_state = NULL;
static fsel_state_t fsel_state;

/* =========== handling of variables ============= */
static menu_variable_t *variables = NULL;

menu_variable_t *menu_get_variables(void) {
  return variables;
}

static int menu_variable_get(char id) {
  menu_variable_t *v = variables;

  while(v) {
    if(v->id == id) return v->value;
    v = v->next;
  }
  return 0;  
}

static bool menu_variable_exists(char id) {
  menu_variable_t *v = variables;

  while(v) {
    if(v->id == id) return true;
    v = v->next;
  }
  return false;  
}

static void menu_variable_set(char id, int value) {
  menu_variable_t *v = variables;
  
  while(v) {
    if(v->id == id) {
      if(v->value != value) {
	v->value = value;
	// also set this in the core
	sys_set_val(id, value);
      }
    }
    v = v->next;
  }
}

static void menu_setup_variable(char id, int value) {
  // menu_debugf("setup variable '%c' = %d", id, value);

  // allocate new entry
  menu_variable_t *variable = pvPortMalloc(sizeof(menu_variable_t));
  variable->id = id;
  variable->value = value;
  variable->next = NULL;

  if(!variables)
    variables = variable;
  else {
    menu_variable_t *v = variables;
    while(v->next) v = v->next;
    v->next = variable;
  }

  // also set the value in the core
  sys_set_val(id, value);
}

// At start the variables are created with the default values from the xml. With
// reset they are set to these values again.
static void menu_init_variable(char id, int value, bool reset) {
  if(reset) menu_variable_set(id, value);
  else      menu_setup_variable(id, value);
}

static void menu_setup_menu_variables(const config_menu_t *menu, bool reset) {
  config_menu_entry_t *me = menu->entries;
  while(me) {
    if(me->type == CONFIG_MENU_ENTRY_MENU)
      menu_setup_menu_variables(me->menu, reset);
    
    if(me->type == CONFIG_MENU_ENTRY_LIST) {
      // setup variable ...
      menu_init_variable(me->list->id, me->list->def, reset);
    }
    
    if(me->type == CONFIG_MENU_ENTRY_TOGGLE) {
      // setup variable ...
      menu_init_variable(me->toggle->id, me->toggle->def, reset);
    }
    
    if(me->type == CONFIG_MENU_ENTRY_RANGE) {
      // setup variable ...
      menu_init_variable(me->range->id, me->range->def, reset);
    }
    
    me = me->next;
  }
}

static void menu_setup_variables(void) {
  // variables occur in two places:
  // in the set command used in actions
  // in menu items (currently only in lists as buttons use actions)

  // Only the variables from menu items are actually stored permanently.
  // Action variables are sent into the core and processed there.
  
  // search through menu tree for lists
  menu_setup_menu_variables(cfg->menu, false);
}

// all variables of the menu back to the values given in the xml
static void menu_reset_variables(void) {
  menu_setup_menu_variables(cfg->menu, true);
}


void menu_set_value(unsigned char id, int8_t value) {
  // this is called when reading the ini file
  // We allow values in the ini file which are actually not in the
  // menu at all. This can be used for "static" changes which are made
  // once by manually manipulating the ini file e.g. with a text
  // editor

  if(!menu_variable_exists(id)) {
    // previous companion versions were writing the action related variables
    // into the ini file as well. This mainly affected the 'R' variable often
    // used for reset. We ignore this particular variable by now as this was
    // not intentinal. This should be removed one day.
    if(id == 'R') {
      menu_debugf("suppressing 'R' reset variable");
      return;
    }
    menu_setup_variable(id, value);
  } else
    menu_variable_set(id, value);
}

/* =========== config selector: the ini currently selected ============= */

// name of the ini selected through the <cfgselector>. NULL means the default
// ini. Only one cfgselector per core is supported.
static char *cfgsel_current = NULL;

// Set while a config is being switched. The rom uploads it starts are served by
// another task and may be done before the switch has mounted everything. They
// must not release the core then, the switch does that itself when it's done.
static volatile bool menu_switching = false;

static const char *menu_cfgsel_current(const config_cfgsel_t *sel) {
  return cfgsel_current?cfgsel_current:sel->def;
}

static void menu_cfgsel_set_current(const char *name) {
  char *n = pvPortMalloc(strlen(name)+1);
  strcpy(n, name);
  if(cfgsel_current) vPortFree(cfgsel_current);
  cfgsel_current = n;
}

// "game.amiga.ini" -> "game". A name that is the extension alone stays as it is.
static void menu_cfgsel_strip(char *name, char **exts) {
  size_t nl = strlen(name);
  for(int i=0;exts && exts[i];i++) {
    size_t el = strlen(exts[i]);
    if(el && nl > el+1 && !strcasecmp(name+nl-el, exts[i]) && name[nl-el-1] == '.') {
      name[nl-el-1] = '\0';
      return;
    }
  }
}

// last entry of the config list and title of its dialog
#define NEW_CFG_FILE "New Config File"

// number of characters of the current config shown in the menu
#define CFGSEL_SHOWN_CHARS 20

// What is shown for an ini: "game.amiga.ini" -> "game", the default ini ->
// "Default Config". buf needs room for strlen(name)+1 characters, the result
// is either buf or a constant string.
static const char *menu_cfgsel_pretty(char *buf, const char *name, const config_cfgsel_t *sel) {
  if(sel->def && strcasecmp(name, sel->def) == 0) return "Default Config";
  strcpy(buf, name);
  menu_cfgsel_strip(buf, sel->ext);
  return buf;
}

// various 8x8 icons
static const unsigned char icn_right_bits[]  = { 0x00,0x04,0x0c,0x1c,0x3c,0x1c,0x0c,0x04 };
static const unsigned char icn_left_bits[]   = { 0x00,0x20,0x30,0x38,0x3c,0x38,0x30,0x20 };
static const unsigned char icn_floppy_bits[] = { 0xff,0x81,0x83,0x81,0xbd,0xad,0x6d,0x3f };
static const unsigned char icn_empty_bits[] =  { 0xc3,0xe7,0x7e,0x3c,0x3c,0x7e,0xe7,0xc3 };
static const unsigned char icn_on_bits[] =     { 0x3c,0x42,0x99,0xbd,0xbd,0x99,0x42,0x3c };
static const unsigned char icn_off_bits[] =    { 0x3c,0x42,0x81,0x81,0x81,0x81,0x42,0x3c };

void u8g2_DrawStrT(u8g2_t *u8g2, u8g2_uint_t x, u8g2_uint_t y, const char *s) {
  // get length of string
  int n = 0;
  while(s[n] && s[n] != ';' && s[n] != ',' && s[n] != '|') n++;

  // create a 0 terminated copy in the stack
  char buffer[n+1];
  strncpy(buffer, s, n);
  buffer[n] = '\0';
  
  u8g2_DrawStr(u8g2, x, y, buffer);
}

#define FS_ICON_WIDTH 10
static int fs_scroll_cur = -1;

static int dir_len(sdc_dir_entry_t *d) {
  int len = 0;
  while(d) {
    d = d->next;
    len++;
  }
  return len;
}

static sdc_dir_entry_t *dir_entry(sdc_dir_entry_t *d, int n) {
  while(n && d) {
    d = d->next;
    n--;
  }
  return d;
}

static void menu_fs_scroll_entry(void) {
  // no scrolling
  if(fs_scroll_cur < 0) return;
  
  // don't scroll anything else
  if(menu_state->type != MENU_TYPE_FILESELECTOR) return;
  
  int row = menu_state->selected - 1;
  int y =  MENU_LINE_Y + MENU_ENTRY_H * (row-menu_state->scroll+1);
  int width = u8g2_GetDisplayWidth(&u8g2);

  // jump to the row'th entry in the dir listing
  const char *name = dir_entry(menu_state->dir, row)->name;
  if(name[0] == '/') return;
  char pbuf[strlen(name)+1];
  if(fsel_state.cfgsel) name = menu_cfgsel_pretty(pbuf, name, fsel_state.cfgsel);
  int swid = u8g2_GetStrWidth(&u8g2, name) + 1;

  // fill the area where the scrolling entry would show
  u8g2_SetClipWindow(&u8g2, FS_ICON_WIDTH, y-MENU_ENTRY_BASE, width, y+MENU_ENTRY_H-MENU_ENTRY_BASE);  
  u8g2_DrawBox(&u8g2, FS_ICON_WIDTH, y-MENU_ENTRY_BASE, width-FS_ICON_WIDTH, MENU_ENTRY_H);
  u8g2_SetDrawColor(&u8g2, 0);

  int scroll = fs_scroll_cur++ - 25;   // 25 means 1 sec delay
  if(fs_scroll_cur > swid-width+FS_ICON_WIDTH+50) fs_scroll_cur = 0;
  if(scroll < 0) scroll = 0;
  if(scroll > swid-width+FS_ICON_WIDTH) scroll = swid-width+FS_ICON_WIDTH;
  
  u8g2_DrawStr(&u8g2, FS_ICON_WIDTH-scroll, y, name);
  
  // restore previous draw mode
  u8g2_SetDrawColor(&u8g2, 1);
  u8g2_SetMaxClipWindow(&u8g2);
  u8g2_SendBuffer(&u8g2);
}

void menu_timer_enable(bool on);

static void menu_fs_draw_entry(int row, sdc_dir_entry_t *entry) {      
  static const unsigned char folder_icon[] = { 0x70,0x8e,0xff,0x81,0x81,0x81,0x81,0x7e };
  static const unsigned char up_icon[] =     { 0x04,0x0e,0x1f,0x0e,0xfe,0xfe,0xfe,0x00 };
  static const unsigned char empty_icon[] =  { 0xc3,0xe7,0x7e,0x3c,0x3c,0x7e,0xe7,0xc3 };

  int y =  MENU_LINE_Y + MENU_ENTRY_H * (row+1);
  int width = u8g2_GetDisplayWidth(&u8g2);
  
  if(entry->name[0] == '/') {
    if(fsel_state.none_str) u8g2_DrawStr(&u8g2, FS_ICON_WIDTH, y, fsel_state.none_str);
    else                    u8g2_DrawStr(&u8g2, FS_ICON_WIDTH, y, "No Disk");
  } else {
    const char *name = entry->name;
    char pbuf[strlen(name)+1];
    if(fsel_state.cfgsel) name = menu_cfgsel_pretty(pbuf, name, fsel_state.cfgsel);
    char str[strlen(name)+1];
    strcpy(str, name);
  
    // properly ellipsize string
    int dotlen = u8g2_GetStrWidth(&u8g2, "...");
    if(u8g2_GetStrWidth(&u8g2, str) > width-FS_ICON_WIDTH) {
      // the entry is too long to fit the menu.    
      // check if this is the selected file and then enable scrolling
      if(row == menu_state->selected - menu_state->scroll - 1)
	fs_scroll_cur = 0;
      
      // enable timer, to allow animations
      menu_timer_enable(true);
      
      while(u8g2_GetStrWidth(&u8g2, str) > width-FS_ICON_WIDTH-dotlen) str[strlen(str)-1] = 0;
      if(strlen(str) < sizeof(str)-4) strcat(str, "...");
    }
  
    u8g2_DrawStr(&u8g2, FS_ICON_WIDTH, y, str);
  }
  
  // draw folder icon in front of directories
  if(entry->is_dir)
    u8g2_DrawXBM(&u8g2, 1, y-8, 8, 8,
		 (entry->name[0] == '/')?(fsel_state.none_icn?fsel_state.none_icn:empty_icon):
		 strcmp(entry->name, "..")?folder_icon:
		 up_icon);

  if(menu_state->selected == row+menu_state->scroll+1)
    u8g2_DrawButtonFrame(&u8g2, 0, y, U8G2_BTN_INV, width, 1, 1);
}

static void menu_push(void) {
  menu_debugf("menu_push()");

  // allocate a new menu entry
  menu_state_t *state = pvPortMalloc(sizeof(menu_state_t));

  // insert the new entry at begin of chain
  state->prev = menu_state;
  menu_state = state;
}

static int menu_len(const config_menu_t *menu) {
  int entries = 0;
  config_menu_entry_t *me = menu->entries;
  while(me) {
    entries++;
    me = me->next;
  }
  return entries;
}

static int menu_count_entries(void) {
  int entries = 0;

  if(menu_state->type == MENU_TYPE_MENU)
    entries = menu_len(menu_state->menu);
  else if(menu_state->type == MENU_TYPE_FILESELECTOR)
    entries = dir_len(menu_state->dir);
  else if(menu_state->type == MENU_TYPE_CUSTOM)
    if(menu_state->custom->length)
      entries = menu_state->custom->length();
    
  return entries+1;  // title is also an entry
}

static bool menu_is_root(const config_menu_t *menu) {
  return menu == cfg->menu;
}

static int menu_entry_is_usable(void) {
  // not root menu? Then all entries are usable. Root menu
  // title is only usable if no hid keyboard or gamepad is present
  if(!menu_is_root(menu_state->menu) || !mcu_hw_hid_present()) return 1;

  // in root menu only the title is unusable
  return menu_state->selected != 0;
}

static config_menu_entry_t *menu_get_selected_entry(void) {
  // first check if there's a menu at all  
  if(menu_state->type != MENU_TYPE_MENU)
    return NULL;
  
  config_menu_entry_t *entry = menu_state->menu->entries;
  for(int i=0;i<menu_state->selected - 1;i++) entry=entry->next;
  return entry;
}
  
static void menu_entry_go(int step) {

  // the up/down events change the a range value in edit mode
  config_menu_entry_t *entry = menu_get_selected_entry();
  if(entry && entry->type == CONFIG_MENU_ENTRY_RANGE && entry->range->edit) {
    int value = menu_variable_get(entry->range->id);

    value -= step;
    if(value > entry->range->max) value = entry->range->max;
    if(value < entry->range->min) value = entry->range->min;          

    menu_variable_set(entry->range->id, value);
    return;
  }							  
  
  int entries = menu_count_entries();
  do {
    menu_state->selected += step;
    
    // single step wraps top/bottom, paging does not
    if(abs(step) == 1) {    
      if(menu_state->selected < 0) menu_state->selected = entries + menu_state->selected;
      if(menu_state->selected >= entries) menu_state->selected = menu_state->selected - entries;
    } else {
      // limit to top/bottom. Afterwards step 1 in opposite
      // direction to skip unusable entries
      if(menu_state->selected < 1) { menu_state->selected = 1; step = 1; }	
      if(menu_state->selected >= entries) { menu_state->selected = entries - 1; step = -1; }
    }

    // scrolling needed?
    if(step > 0) {
      if(entries <= 5)                            menu_state->scroll = 0;
      else {
	if(menu_state->selected <= 3)             menu_state->scroll = 0;
	else if(menu_state->selected < entries-2) menu_state->scroll = menu_state->selected - 3;
	else                                      menu_state->scroll = entries-5;
      }
    }

    if(step < 0) {
      if(entries <= 5)                            menu_state->scroll = 0;
      else {
	if(menu_state->selected <= 2)             menu_state->scroll = 0;
	else if(menu_state->selected < entries-3) menu_state->scroll = menu_state->selected - 2;
	else                                      menu_state->scroll = entries-5;
      }
    }    
  } while(!menu_entry_is_usable());
}

// hint: optional text shown right aligned in the title line, e.g. "DEL=Delete"
static void menu_draw_title(const char *s, bool arrow, bool selected, const char *hint) {
  int x = 1;

  // draw left arrow for submenus
  if(arrow) {
    u8g2_DrawXBM(&u8g2, 0, 1, 8, 8, icn_left_bits);    
    x = 8;
  } else if(!mcu_hw_hid_present()) {
    // without keyboard or joystick connected, the menu runs in
    // "single button" mode
  
    // no arrow? Then this is the root menu. In that case
    // a cross is drawn in "single button mode"
    u8g2_DrawXBM(&u8g2, 0, 1, 8, 8, icn_empty_bits);    
    x = 10;
  }
  
  // draw title in bold and seperator line
  u8g2_SetFont(&u8g2, u8g2_font_helvB08_tr);
  u8g2_DrawStr(&u8g2, x, MENU_ENTRY_BASE, s);

  // the hint in the regular font, if there's room for it behind the title
  if(hint) {
    int xtitle = x + u8g2_GetStrWidth(&u8g2, s);
    u8g2_SetFont(&u8g2, font_helvR08_te);
    int xhint = u8g2_GetDisplayWidth(&u8g2) - u8g2_GetStrWidth(&u8g2, hint) - 2;
    if(xtitle + 4 <= xhint) u8g2_DrawStr(&u8g2, xhint, MENU_ENTRY_BASE, hint);
  }
  
  u8g2_DrawHLine(&u8g2, 0, MENU_LINE_Y, u8g2_GetDisplayWidth(&u8g2));

  if(selected)
    u8g2_DrawButtonFrame(&u8g2, 0, MENU_ENTRY_BASE, U8G2_BTN_INV,
	 u8g2_GetDisplayWidth(&u8g2), 1, 1);
  
  // draw the rest with normal font
  u8g2_SetFont(&u8g2, font_helvR08_te);
}

static char *menuentry_get_label(config_menu_entry_t *entry) {
  if(entry->type == CONFIG_MENU_ENTRY_MENU)
    return entry->menu->label;
  if(entry->type == CONFIG_MENU_ENTRY_FILESELECTOR)
    return entry->fsel->label;
  if(entry->type == CONFIG_MENU_ENTRY_LIST)
    return entry->list->label;
  if(entry->type == CONFIG_MENU_ENTRY_BUTTON)
    return entry->button->label;
  if(entry->type == CONFIG_MENU_ENTRY_IMAGE)
    return entry->image->label;
  if(entry->type == CONFIG_MENU_ENTRY_TOGGLE)
    return entry->toggle->label;
  if(entry->type == CONFIG_MENU_ENTRY_RANGE)
    return entry->range->label;
  if(entry->type == CONFIG_MENU_ENTRY_CFGSELECTOR)
    return entry->cfgsel->label;
  
  return NULL;
}
  
static char *menu_get_listentry(config_menu_entry_t *entry, int value) {
  if(!entry || entry->type != CONFIG_MENU_ENTRY_LIST) return NULL;

  config_listentry_t *le = entry->list->listentries;
  while(le) {
    if(le->value == value)
      return le->label;
    le = le->next;
  }

  return NULL;
}

static void menu_draw_entry(config_menu_entry_t *entry, int row, bool selected) {
  menu_debugf("row %d: %s '%s'", row,
	      config_menuentry_get_type_str(entry),
	      menuentry_get_label(entry));

  // all menu entries use some kind of label
  char *s = menuentry_get_label(entry);
  int ypos = MENU_LINE_Y+MENU_ENTRY_H + MENU_ENTRY_H * row;
  int width = u8g2_GetDisplayWidth(&u8g2);
  
  // all menu entries are a plain text
  u8g2_DrawStr(&u8g2, 1, ypos, s);
    
  // prepare highlight
  int hl_w = width;

  // handle second string for list entries
  if(entry->type == CONFIG_MENU_ENTRY_LIST) {
    // get matching variable
    int value = menu_variable_get(entry->list->id);
    char *str = menu_get_listentry(entry, value);

    if(str) {
      // right align entry
      int sw = u8g2_GetStrWidth(&u8g2, str) + 1;
      if(sw > width/2) sw = width/2;
      u8g2_DrawStr(&u8g2, width-sw, ypos, str);
    }
		  
    hl_w = width/2;
  }
  
  // some entries have a small icon to the right    
  else if(entry->type == CONFIG_MENU_ENTRY_MENU)
    u8g2_DrawXBM(&u8g2, hl_w-8, ypos-8, 8, 8, icn_right_bits);

  else if(entry->type == CONFIG_MENU_ENTRY_FILESELECTOR) {
    // icon depends if floppy is inserted
    u8g2_DrawXBM(&u8g2, hl_w-MENU_ENTRY_BASE, ypos-8, 8, 8,
		 sdc_get_image_name(entry->fsel->index)?icn_floppy_bits:icn_empty_bits);
  }
  
  else if(entry->type == CONFIG_MENU_ENTRY_CFGSELECTOR) {
    // the current config is shown right aligned, like the value of a list
    const char *cur = menu_cfgsel_current(entry->cfgsel);
    if(cur) {
      char pbuf[strlen(cur)+1];
      const char *shown = menu_cfgsel_pretty(pbuf, cur, entry->cfgsel);
      char str[strlen(shown)+1];
      strcpy(str, shown);

      // up to CFGSEL_SHOWN_CHARS characters, as far as they fit next to the label
      if(strlen(str) > CFGSEL_SHOWN_CHARS) str[CFGSEL_SHOWN_CHARS] = '\0';
      int avail = width - u8g2_GetStrWidth(&u8g2, s) - 8;
      while(strlen(str) > 1 && u8g2_GetStrWidth(&u8g2, str) + 1 > avail)
	str[strlen(str)-1] = '\0';
      u8g2_DrawStr(&u8g2, width-u8g2_GetStrWidth(&u8g2, str)-1, ypos, str);
    }
    // the whole row is highlighted, hl_w stays at the full width
  }

  else if(entry->type == CONFIG_MENU_ENTRY_TOGGLE) 
    u8g2_DrawXBM(&u8g2, hl_w-MENU_ENTRY_BASE, ypos-8, 8, 8,
		 menu_variable_get(entry->toggle->id)?icn_on_bits:icn_off_bits);
  
  if(entry->type == CONFIG_MENU_ENTRY_IMAGE) {
    const unsigned char *icon = sdc_get_image_name(entry->image->index+MAX_DRIVES)?icn_floppy_bits:
      entry->image->none_icn?entry->image->none_icn:
      icn_empty_bits;
    
    u8g2_DrawXBM(&u8g2, hl_w-MENU_ENTRY_BASE, ypos-8, 8, 8, icon);
  }
    
  else if(entry->type == CONFIG_MENU_ENTRY_RANGE) {
    char str[5];  // max length 5 for "-100\0"
    
    // draw range value
    int value = menu_variable_get(entry->range->id);
    sprintf(str, "%d", value);

    // right align entry
    int sw = u8g2_GetStrWidth(&u8g2, str) + 1;
    if(sw > width/2) sw = width/2;
    u8g2_DrawStr(&u8g2, width-sw, ypos, str);

    // in "normal" menu mode, the entire range is highlighted when
    // selected. In "edit" mode only the value is highlighted
    if(entry->range->edit) {    
      // only highlight the value
      hl_w = u8g2_GetStrWidth(&u8g2, str) + 1;      
    }
  }
    
  if(selected)
    u8g2_DrawButtonFrame(&u8g2, width-hl_w, ypos, U8G2_BTN_INV, hl_w, 1, 1);
}

// Draws msg centered in lines no wider than the display, from y_in down, and
// returns the y below the last line. With y_in 0 it only measures. A '\n' ends
// a line, the rest breaks at spaces, and a word wider than the display is split.
static int menu_wrap_text(int y_in, const char *msg) {
  const int width = u8g2_GetDisplayWidth(&u8g2);
  char line[80];
  int y = y_in;

  u8g2_SetFont(&u8g2, font_helvR08_te);
  if(!*msg) return y + 11;   // an empty message still takes one line

  while(*msg) {
    size_t take = 0;   // characters that fit for sure
    size_t gap  = 0;   // last space among them, 0 = none
    size_t i;

    while(*msg == ' ') msg++;                        // a line never starts with a space
    if(*msg == '\n') { y += 11; msg++; continue; }   // an empty line
    if(!*msg) break;

    // take characters until the line is too wide, the buffer is full or the text ends
    for(i = 0; msg[i] && msg[i] != '\n' && i < sizeof(line) - 1; i++) {
      line[i]     = msg[i];
      line[i + 1] = '\0';
      if(u8g2_GetStrWidth(&u8g2, line) > width) break;
      if(msg[i] == ' ') gap = i;
      take = i + 1;
    }

    // stopped inside a word with a space before it: break at the space instead.
    // Stopped at a space: everything before it fits, the line ends right there.
    if(msg[i] && msg[i] != '\n' && msg[i] != ' ' && gap) take = gap;
    if(take == 0) take = 1;   // even one character is too wide, it goes out anyway
    while(take > 1 && msg[take - 1] == ' ') take--;   // spaces at the end would shift the centering

    line[take] = '\0';
    if(y_in) u8g2_DrawStr(&u8g2, (width - u8g2_GetStrWidth(&u8g2, line)) / 2, y, line);
    y += 11;

    // the next line starts after the '\n' that ended this one
    msg += take;
    while(*msg == ' ') msg++;
    if(*msg == '\n') msg++;
  }

  return y;
}

// draw a dialog box
static bool dialog_opened_osd = false;
static TimerHandle_t dialog_disappear_timer = NULL;  
static char network_ipaddr[16];
static bool network_was_connected = false;

static bool menu_dialog_is_open(void) {
  if(!dialog_disappear_timer) return false;  // no timer -> no dialog
  return xTimerIsTimerActive(dialog_disappear_timer) != pdFALSE;
}

static void menu_dialog_timeout(__attribute__((unused)) TimerHandle_t arg) {
  menu_debugf("Close dialog");
  if(dialog_opened_osd) osd_enable(OSD_INVISIBLE);    
  else                  menu_do(0);
}

static void menu_dialog_close(void) {
  if(!dialog_disappear_timer) return;
  xTimerStop(dialog_disappear_timer, 0);
  menu_dialog_timeout(NULL);
}
  
static void menu_draw_dialog_for(const char *title, const char *msg,
                                  TickType_t duration) {
  // check if osd is already visible
  if(!menu_dialog_is_open()) {
    dialog_opened_osd = !osd_is_visible();
    if(dialog_opened_osd) osd_enable(OSD_VISIBLE);
  }

  if(!dialog_disappear_timer)
    dialog_disappear_timer = xTimerCreate("Dialog timer", duration, pdFALSE,
					   NULL, menu_dialog_timeout);
  else
    xTimerChangePeriod(dialog_disappear_timer, duration, 0);
  xTimerStart(dialog_disappear_timer, 0);
  
  u8g2_ClearBuffer(&u8g2);

  // MENU_LINE_Y is the height of the title incl line
  int y = (64 - MENU_LINE_Y - menu_wrap_text(0, msg))/2;
  
  u8g2_SetFont(&u8g2, u8g2_font_helvB08_tr);
  
  int width = u8g2_GetDisplayWidth(&u8g2);
  int swid = u8g2_GetStrWidth(&u8g2, title);
 
  // draw title in bold and seperator line
  u8g2_DrawStr(&u8g2, (width-swid)/2, y+MENU_ENTRY_BASE, title);
  u8g2_DrawHLine(&u8g2, (width-swid)/2, y+MENU_ENTRY_H, swid);

  u8g2_SetFont(&u8g2, font_helvR08_te);

  menu_wrap_text(y+23, msg);
  
  u8g2_SendBuffer(&u8g2);
}

void menu_draw_dialog(const char *title, const char *msg) {
  menu_draw_dialog_for(title, msg, pdMS_TO_TICKS(2000));
}

static void menu_draw(const config_menu_t *menu, int selected, int scroll) {
  u8g2_ClearBuffer(&u8g2);
 
  // =============== draw a regular menu =================
  menu_debugf("drawing '%s'", menu->label);  
    
  // draw the title
  menu_draw_title(menu->label, !menu_is_root(menu), selected == 0, NULL);

  config_menu_entry_t *entry = menu->entries;
  for(int i=0;i<scroll;i++) entry=entry->next;  // skip first "scroll" entries
  for(int i=0;i<4 && entry;i++,entry=entry->next)           // then draw up to four entries
    menu_draw_entry(entry, i, selected == scroll+i+1);    
  
  u8g2_SendBuffer(&u8g2);
}

static void menu_fsel_draw(const char *label, sdc_dir_entry_t *dir, int selected, int scroll) {
  u8g2_ClearBuffer(&u8g2);

  // =============== draw a fileselector =================    
  menu_debugf("drawing '%s'", label);
  
  // the list of configs shows what the delete key does
  menu_draw_title(label, true, selected == 0, fsel_state.cfgsel?"DEL=Delete":NULL);
  menu_timer_enable(false);
  fs_scroll_cur = -1;

  // draw up to four entries
  for(int i=0;i<4 && i<dir_len(dir)-scroll;i++) {            
    menu_debugf("file %s", dir_entry(dir, i+scroll)->name);
    menu_fs_draw_entry(i, dir_entry(dir, i+scroll));
  }
  u8g2_SendBuffer(&u8g2);
}

static void menu_custom_draw(config_custom_t *custom, int selected, int scroll) {
  u8g2_ClearBuffer(&u8g2);

  // =============== draw a custom dialog =================    
  menu_debugf("drawing custom '%s' (%d)", custom->label, selected);

  // title is a regular entry
  menu_draw_title(custom->label, true, selected == 0, NULL);

  if(custom->draw)
    custom->draw();
  
  u8g2_SendBuffer(&u8g2);
}

void menu_goto(const config_menu_t *menu) {
  menu_push();
  
  // prepare menu state ...
  menu_state->menu = menu;
  menu_state->selected = 1;
  menu_state->scroll = 0;
  menu_state->type = MENU_TYPE_MENU;
}

static void menu_file_selector_open(config_menu_entry_t *entry) {
  menu_debugf("menu_file_selector_open()");
  fsel_state.cfgsel = NULL;

  // the file selector can either be opened from disk image file
  // selectors or from rom image selectors  
  if(entry->type == CONFIG_MENU_ENTRY_IMAGE) {
    menu_debugf("opening rom image selector");
    fsel_state.ext = entry->image->ext;
    fsel_state.index = entry->image->index + MAX_DRIVES;  // the images are stored after the drives
    fsel_state.none_str = entry->image->none_str;
    fsel_state.none_icn = entry->image->none_icn;
    fsel_state.action = entry->image->action;
  } else {
    menu_debugf("opening drive image selector");
    fsel_state.ext = entry->fsel->ext;
    fsel_state.index = entry->fsel->index;
    fsel_state.none_str = NULL;
    fsel_state.none_icn = NULL;
    fsel_state.action = entry->fsel->action;
  }
    
  // The file selector usually works on the sd card as that is what
  // the cores are reading data from. But cores may be loaded from
  // USB as well. The file selectors default path will begin with
  // "/usb" in that case

  // Initialize current working directory if needed
  if(!sdc_get_cwd(fsel_state.index)) {
    bool is_usb = strncasecmp(entry->fsel->def, "/usb", 4) == 0;
    
    // check if USB is to be used but isn't present
    if(is_usb && (disk_status(1) != RES_OK)) {
      menu_draw_dialog("USB Error", "No USB mass storage device connected!");
      return;
    }

    sdc_set_cwd(fsel_state.index, is_usb?"/usb":"/sd");
  }

  // IMAGE and file selectors are both drawn by the fileselector routines
  menu_push();
  menu_state->label = (entry->type == CONFIG_MENU_ENTRY_IMAGE)?entry->image->label:entry->fsel->label;
  menu_state->selected = 1;
  menu_state->scroll = 0;
  menu_state->type = MENU_TYPE_FILESELECTOR;
  
  // scan file system
  menu_state->dir = sdc_readdir(fsel_state.index, NULL, (void*)fsel_state.ext);
  // try to jump to current file. Get the current image name and path
  char *name = sdc_get_image_name(fsel_state.index);
  if(name) {
    debugf("trying to jump to %s", name);
    // try to find name in file list
    for(int i=0;i<dir_len(menu_state->dir);i++) {
      if(strcmp(dir_entry(menu_state->dir, i)->name, name) == 0) {
	debugf("found preset entry %d", i);
	
	// file found, adjust entry and offset
	menu_state->selected = i+1;
	
	if(dir_len(menu_state->dir) > 4 && menu_state->selected > 3) {
	  debugf("more than 4 files and selected is > 3");
	  if(menu_state->selected < dir_len(menu_state->dir)-1) menu_state->scroll = menu_state->selected - 3;
	  else                                                  menu_state->scroll = dir_len(menu_state->dir)-4;
	}
      }
    }
  }  
}

// all other entries in step down
static void menu_pop(void) {
  menu_debugf("menu_pop()");

  // "pop"ing the root menu means to hide it. This will only
  // happen in single button mode with the roots title
  if(!menu_state->prev) {
    osd_enable(OSD_INVISIBLE);
    return;
  }

  // neither should this as we never really close the
  // root menu
  if(menu_state->menu == cfg->menu) {
    vPortFree(menu_state);
    menu_state = NULL;
    return;
  }

  // de-chain first entry
  menu_state_t *m = menu_state;
  menu_state = menu_state->prev;
  vPortFree(m);
}

/* ============================================================================ */
/* ============================ config selector =============================== */
/* ============================================================================ */

// Opens the list of the configs. The entry named focus is selected, the config
// in use if there is none.
static void menu_cfgsel_list(const config_cfgsel_t *sel, const char *focus) {
  menu_debugf("menu_cfgsel_list()");

  // only the ini files meant for this core are listed, followed by "New Config File"
  sdc_dir_entry_t *dir = sdc_readdir_cfg(sel->ext);

  // the list is drawn by the file selector code
  static const unsigned char plus_icon[] = { 0x00,0x18,0x18,0x7e,0x7e,0x18,0x18,0x00 };
  fsel_state.ext = sel->ext;
  fsel_state.index = -1;
  fsel_state.action = NULL;
  fsel_state.none_str = NEW_CFG_FILE;
  fsel_state.none_icn = (unsigned char*)plus_icon;
  fsel_state.cfgsel = sel;

  menu_push();
  menu_state->label = sel->label?sel->label:"Config";
  menu_state->selected = 1;
  menu_state->scroll = 0;
  menu_state->type = MENU_TYPE_FILESELECTOR;
  menu_state->dir = dir;

  // jump to the config in focus
  const char *cur = focus?focus:menu_cfgsel_current(sel);
  menu_debugf("Config list: %d entries, focus '%s'", dir_len(dir), cur?cur:"-");
  int len = dir_len(dir);
  for(int i=0;cur && i<len;i++) {
    if(strcasecmp(dir_entry(dir, i)->name, cur) == 0) {
      menu_state->selected = i+1;
      if(len > 4 && menu_state->selected > 3) {
	if(menu_state->selected < len-1) menu_state->scroll = menu_state->selected - 3;
	else                             menu_state->scroll = len-4;
      }
    }
  }
}

static void menu_cfgsel_open(config_menu_entry_t *entry) {
  menu_debugf("menu_cfgsel_open()");
  menu_cfgsel_list(entry->cfgsel, NULL);
}

static void menu_cfgsel_apply(const config_cfgsel_t *sel, const char *name, bool fresh);

/* ========================== "New Config File" =============================== */

// A name for a new config is typed in. The keyboard delivers latin1 events
// for this (see menu_handle_latin1()), the text is shown by a custom dialog.

#define NEWCFG_MAX      32     // characters of the name, without the extension
#define MENU_SWALLOW_MS 150    // see menu_select_swallowed()

static struct {
  const config_cfgsel_t *sel;
  char name[NEWCFG_MAX+1];
} newcfg;

static int newcfg_length(void) { return 0; }

static void newcfg_draw(void) {
  int width = u8g2_GetDisplayWidth(&u8g2);
  int y = MENU_LINE_Y + 2*MENU_ENTRY_H;

  // what is added to the name, right aligned
  const char *ext = (newcfg.sel && newcfg.sel->ext && newcfg.sel->ext[0])?newcfg.sel->ext[0]:"";
  char suffix[strlen(ext)+2];
  snprintf(suffix, sizeof(suffix), ".%s", ext);
  int sw = u8g2_GetStrWidth(&u8g2, suffix);
  u8g2_DrawStr(&u8g2, width-sw-2, y, suffix);

  // the name with a cursor in front of it. Only its end is shown if it doesn't fit
  char str[NEWCFG_MAX+2];
  strcpy(str, newcfg.name);
  strcat(str, "_");
  const char *show = str;
  while(show[1] && u8g2_GetStrWidth(&u8g2, show) > width-sw-6) show++;
  u8g2_DrawStr(&u8g2, 2, y, show);

  // the keys, in the last line
  y = MENU_LINE_Y + 4*MENU_ENTRY_H;
  u8g2_DrawStr(&u8g2, 2, y, "Esc=Cancel");
  u8g2_DrawStr(&u8g2, width-u8g2_GetStrWidth(&u8g2, "Enter=OK")-2, y, "Enter=OK");
}

static const config_custom_t newcfg_custom = {
  .label = NEW_CFG_FILE,
  .length = newcfg_length,
  .draw = newcfg_draw
};

static bool menu_newcfg_active(void) {
  return menu_state && menu_state->type == MENU_TYPE_CUSTOM && menu_state->custom == &newcfg_custom;
}

// Enter and the space key do not only send a latin1 event but also a select
// event, right behind it. While a name is typed in, that must neither cancel
// the dialog (space) nor select something in the list that is shown after
// the dialog was left (enter).
static TickType_t menu_swallow_start;
static bool menu_swallow_armed = false;

static void menu_swallow_select(void) {
  menu_swallow_start = xTaskGetTickCount();
  menu_swallow_armed = true;
}

static bool menu_select_swallowed(void) {
  // the select that belongs to the enter that was just handled
  if(menu_swallow_armed) {
    menu_swallow_armed = false;
    if((xTaskGetTickCount() - menu_swallow_start) < pdMS_TO_TICKS(MENU_SWALLOW_MS)) {
      menu_debugf("select ignored (belongs to enter)");
      return true;
    }
  }

  // the select of the space key. If a dialog is open (e.g. "name exists"), a
  // select is allowed to close it
  if(menu_newcfg_active() && !menu_dialog_is_open()) {
    menu_debugf("select ignored (name is typed in)");
    return true;
  }
  return false;
}

static void menu_newcfg_open(const config_cfgsel_t *sel) {
  menu_debugf("menu_newcfg_open()");
  newcfg.sel = sel;
  newcfg.name[0] = '\0';

  menu_push();
  menu_state->type = MENU_TYPE_CUSTOM;
  menu_state->custom = &newcfg_custom;
  menu_state->selected = 0;
  menu_state->scroll = 0;
}

// The name is complete: the file is reserved (it is never overwritten), the menu is
// closed and the new config is started: all values at their defaults, all drives
// ejected, the file written with that state and the core reset. From then on the
// settings are saved into the new file.
static void menu_newcfg_create(void) {
  const config_cfgsel_t *sel = newcfg.sel;
  const char *ext = (sel->ext && sel->ext[0])?sel->ext[0]:NULL;

  // leading dots would hide the file from the list, trailing ones and spaces are no good on FAT
  const char *b = newcfg.name;
  while(*b == ' ' || *b == '.') b++;
  char name[NEWCFG_MAX+1];
  strcpy(name, b);
  while(name[0] && (name[strlen(name)-1] == ' ' || name[strlen(name)-1] == '.'))
    name[strlen(name)-1] = '\0';

  if(!ext)          { menu_draw_dialog(NEW_CFG_FILE, "Config has no\nextension"); return; }
  if(!name[0])      { menu_draw_dialog(NEW_CFG_FILE, "Please enter\na name");      return; }

  // "name.ext", unless the name already ends like that
  char file[strlen(name)+strlen(ext)+2];
  if(sdc_cfg_name_ok(name, sel->ext)) strcpy(file, name);
  else                                snprintf(file, sizeof(file), "%s.%s", name, ext);

  int ret = inifile_create(file);
  if(ret == 1) { menu_draw_dialog(NEW_CFG_FILE, "A config with this\nname exists"); return; }
  if(ret != 0) { menu_draw_dialog(NEW_CFG_FILE, "Creating the\nfile failed");       return; }

  menu_debugf("Config: created '%s'", file);

  // leave the dialog and the config selector, close the menu and start the new config
  fsel_state.cfgsel = NULL;
  menu_pop();
  menu_pop();
  osd_enable(OSD_INVISIBLE);
  menu_cfgsel_apply(sel, file, true);
}

// a key typed in
static void menu_newcfg_key(uint8_t code) {
  size_t len = strlen(newcfg.name);
  
  if(code == 8) {                                // backspace
    if(!len) return;
    newcfg.name[len-1] = '\0';
  } else if(code == 10) {                        // enter
    menu_newcfg_create();
    menu_swallow_select();
    return;
  } else if(code >= 32 && code < 127 && len < NEWCFG_MAX && !strchr("\\/:*?\"<>|", code)) {
    newcfg.name[len] = code;
    newcfg.name[len+1] = '\0';
  } else
    return;

  menu_do(0);   // redraw
}

/* ============================ delete a config =============================== */

// Pressing the Delete key in the list of the configs asks if the selected config
// is to be deleted. The question only reacts to y (yes), n and escape (no), all
// other keys are ignored.

// The dialog sits on top of the list. The entry to be deleted is the selected one
// there, so no copy of its name is kept (this is RAM that is scarce on the RP2040).
static sdc_dir_entry_t *delcfg_entry(void) {
  return dir_entry(menu_state->prev->dir, menu_state->prev->selected-1);
}

static int delcfg_length(void) { return 0; }

static void delcfg_draw(void) {
  int width = u8g2_GetDisplayWidth(&u8g2);

  // "Delete <name>?", the name is shortened if needed
  const char *file = delcfg_entry()->name;
  char pbuf[strlen(file)+1];
  const char *pretty = menu_cfgsel_pretty(pbuf, file, fsel_state.cfgsel);
  char name[strlen(pretty)+1];
  strcpy(name, pretty);
  char str[strlen(name)+10];
  for(;;) {
    snprintf(str, sizeof(str), "Delete %s?", name);
    if(strlen(name) <= 1 || u8g2_GetStrWidth(&u8g2, str) <= width-4) break;
    name[strlen(name)-1] = '\0';
  }
  u8g2_DrawStr(&u8g2, 2, MENU_LINE_Y + 1*MENU_ENTRY_H, str);

  // The keys, in the last line: No at the left, Yes at the right
  int y = MENU_LINE_Y + 4*MENU_ENTRY_H;
  u8g2_DrawStr(&u8g2, 2, y, "N=No");
  u8g2_DrawStr(&u8g2, width-u8g2_GetStrWidth(&u8g2, "Y=Yes")-2, y, "Y=Yes");
}

static const config_custom_t delcfg_custom = {
  .label = "Delete Config",
  .length = delcfg_length,
  .draw = delcfg_draw
};

static bool menu_delcfg_active(void) {
  return menu_state && menu_state->type == MENU_TYPE_CUSTOM && menu_state->custom == &delcfg_custom;
}

// the delete key was pressed in the list: ask
static void menu_delcfg_open(void) {
  if(menu_state->selected < 1) return;     // the title
  const config_cfgsel_t *sel = fsel_state.cfgsel;
  sdc_dir_entry_t *entry = dir_entry(menu_state->dir, menu_state->selected-1);

  // not the "New Config File" entry and nothing that isn't a config of this core
  if(entry->name[0] == '/' || !sdc_cfg_name_ok(entry->name, sel->ext)) return;

  // The default ini holds the selection and is loaded at power up. The config in
  // use is what the settings are saved into.
  if(sel->def && strcasecmp(entry->name, sel->def) == 0) {
    menu_draw_dialog("Delete Config", "The default config\ncan't be deleted");
    return;
  }
  const char *cur = menu_cfgsel_current(sel);
  if(cur && strcasecmp(entry->name, cur) == 0) {
    menu_draw_dialog("Delete Config", "This config is\nin use");
    return;
  }

  menu_push();
  menu_state->type = MENU_TYPE_CUSTOM;
  menu_state->custom = &delcfg_custom;
  menu_state->selected = 0;
  menu_state->scroll = 0;
  menu_do(0);   // draw the question, a key press does not do that
}

static void menu_delcfg_choose(bool yes) {
  const config_cfgsel_t *sel = fsel_state.cfgsel;

  if(yes) {
    // The names are copied: the list they are in is released when the new one is read.
    // The entry behind the deleted one is selected afterwards.
    sdc_dir_entry_t *entry = delcfg_entry();
    char file[strlen(entry->name)+1];
    char next[(entry->next?strlen(entry->next->name):0)+1];
    strcpy(file, entry->name);
    strcpy(next, entry->next?entry->next->name:"");

    menu_pop();                             // the question
    if(inifile_remove(file) == 0) {
      menu_debugf("Config: deleted '%s'", file);
      menu_pop();                           // the old list
      menu_cfgsel_list(sel, next);
    } else
      menu_draw_dialog("Delete Config", "Deleting failed");
  } else
    menu_pop();

  menu_do(0);   // draw the list
}

// a key typed in
static void menu_delcfg_key(uint8_t code) {
  // the key labelled Y of a german keyboard delivers a z (the keyboard table is american)
  if(code == 'y' || code == 'Y' || code == 'z' || code == 'Z') menu_delcfg_choose(true);
  else if(code == 'n' || code == 'N') menu_delcfg_choose(false);
}

// let the core run again, like at the end of the boot process
static void menu_core_release(void) {
  if(config_get_action("ready")) sys_run_action_by_name("ready");
  else                           sys_set_val('R', 0);
}

// Switch to another ini. The images have to be exchanged while nothing is
// using them, so this is done with the core held in reset: Before, a running
// core was changed under its feet, there was no reset afterwards and some
// images were not mounted properly.
// Switches to a config. All drives are ejected first, so no drive of the previous
// config is left behind. With fresh the config is a new one: it starts like after power
// up without any ini file. The variables are set to the values of the xml and the ini
// file is written with this state. Neither the default ini nor the new ini is read.
static void menu_cfgsel_apply(const config_cfgsel_t *sel, const char *name, bool fresh) {
  menu_debugf("Config: switching to '%s'", name);
  menu_switching = true;

  // 1. Hold the core in reset. It won't ask for any more sectors then. Wait
  //    until the sd card is done with whatever it was doing for the core.
  sys_set_val('R', 1);
  sdc_wait_idle();

  // The config to switch to has to be there. Nothing is changed otherwise.
  if(!fresh && !inifile_exists(name)) {
    menu_debugf("Config: '%s' not found", name);
    menu_switching = false;
    menu_draw_dialog("Config", "Loading failed");
    menu_core_release();
    return;
  }

  // 2. Eject all drives, the names of the images are forgotten. The drives of the old
  //    config must not stay: afterwards only the ones the default ini and the new
  //    config ask for are mounted, as after power up. The rom images keep their names,
  //    they are not drives and are uploaded again below.
  for(int drive=0;drive<MAX_DRIVES;drive++) sdc_image_open(drive, NULL);

  bool written = true;
  if(fresh) {
    // 3. Everything as after power up without ini file: the variables have the
    //    values of the xml and no drive is mounted. The new ini is written with that.
    menu_reset_variables();
    written = (inifile_write((char*)name) == 0);
    if(!written) menu_debugf("Config: writing '%s' failed", name);
  } else {
    // 3. Read the settings: the default ini first and the selected one on top,
    //    just like it happens at power up. This only changes the variables and the
    //    names of the images to be used, nothing is mounted, yet.
    bool is_default = !sel->def || (strcasecmp(name, sel->def) == 0);
    if(!is_default) inifile_read(sel->def);
    if(inifile_read((char*)name) != 0) {
      menu_debugf("Config: reading '%s' failed", name);
      menu_switching = false;
      menu_draw_dialog("Config", "Loading failed");
      menu_core_release();
      return;
    }
  }

  // 4. Remember the selection and store it in the default ini. This is done
  //    before the images are mounted: that starts the rom uploads, which run
  //    on another task and use the file system and the spi bus as well.
  menu_cfgsel_set_current(name);
  int saved = written && (!sel->def || inifile_selection_set(sel->def, name) == 0);

  // 5. Mount all the images, one by one, each one closed and opened under
  //    the sd lock. The core has been told that its disks are gone before.
  int failed = sdc_mount_defaults();

  // 6. Let the core run. If rom images are still being uploaded this happens once
  //    they are done, as it does at power up. Uploads that were finished meanwhile
  //    did not do that, see menu_switching.
  menu_switching = false;
  if(!sdc_image_upload_in_progress() && !sdc_check_for_pending_image_uploads())
    menu_core_release();
  else
    menu_debugf("Image upload in progress, delaying ready action");

  if(failed || !saved) {
    // tell what went wrong. The dialog shows the OSD for a moment
    char msg[96];
    snprintf(msg, sizeof(msg), "%s\n%s", name, failed?"(image file missing)":"(not saved)");
    menu_draw_dialog("Config", msg);
  }
}

static void menu_cfgsel_selected(sdc_dir_entry_t *entry) {
  const config_cfgsel_t *sel = fsel_state.cfgsel;

  // the last entry of the list creates a new config
  if(entry->name[0] == '/') {
    menu_newcfg_open(sel);
    return;
  }
  
  menu_debugf("Config: '%s' selected", entry->name);
  fsel_state.cfgsel = NULL;

  // the entry is part of a list that is released with the next listing
  char name[strlen(entry->name)+1];
  strcpy(name, entry->name);
  
  // return to the menu, close it right away and switch
  menu_pop();
  osd_enable(OSD_INVISIBLE);
  menu_cfgsel_apply(sel, name, false);
}

// at power up: load the ini that was selected last. The default ini itself has
// been loaded by the init action by now.
static const config_cfgsel_t *menu_find_cfgsel(const config_menu_t *menu) {
  for(config_menu_entry_t *me = menu->entries; me; me = me->next) {
    if(me->type == CONFIG_MENU_ENTRY_CFGSELECTOR) return me->cfgsel;
    if(me->type == CONFIG_MENU_ENTRY_MENU) {
      const config_cfgsel_t *sel = menu_find_cfgsel(me->menu);
      if(sel) return sel;
    }
  }
  return NULL;
}

static void menu_cfgsel_boot(void) {
  const config_cfgsel_t *sel = menu_find_cfgsel(cfg->menu);
  if(!sel || !sel->def || !sel->ext) return;

  char name[FF_LFN_BUF+1];
  if(inifile_selection_get(sel->def, name, sizeof(name)) != 0 || !name[0]) return;

  // only a file meant for this core is accepted
  if(!sdc_cfg_name_ok(name, sel->ext)) {
    menu_debugf("Config: ignoring stored selection '%s'", name);
    return;
  }

  if(strcasecmp(name, sel->def) != 0 && inifile_read(name) != 0) {
    menu_debugf("Config: '%s' not found, using the default", name);
    return;
  }

  menu_debugf("Config: '%s' selected", name);
  menu_cfgsel_set_current(name);
}

// The file a save action has to write. The xml names the default ini for it
// ("amiga.ini"), but while another config is selected the settings belong to
// that config. Saving them into the default ini would overwrite it and the
// default would then contain the settings of the other config.
const char *menu_cfgsel_save_name(const char *file) {
  const config_cfgsel_t *sel = cfg?menu_find_cfgsel(cfg->menu):NULL;
  if(file && sel && sel->def && cfgsel_current && strcasecmp(file, sel->def) == 0)
    return cfgsel_current;
  return file;
}

static void menu_fileselector_select(sdc_dir_entry_t *entry) {
  int drive = fsel_state.index;
  menu_debugf("%s %d: file selected '%s'", drive>=MAX_DRIVES?"IMG":"DRV",
	      drive>=MAX_DRIVES?drive-MAX_DRIVES:drive, entry->name);
    
  // stop any scroll timer that might be running
  menu_timer_enable(false);

  // the list of a config selector
  if(fsel_state.cfgsel) {
    menu_cfgsel_selected(entry);
    return;
  }
    
  if(entry->is_dir) {
    if(entry->name[0] == '/') {
      // User selected the "No Disk" entry
      // return to parent form
      menu_pop();
      // Eject
      sdc_image_open(drive, NULL);
    } else {	
      // check if we are going up one dir and try to select the
      // directory we are coming from
      char *prev = NULL; 
      if(strcmp(entry->name, "..") == 0) {
	prev = strrchr(sdc_get_cwd(drive), '/');
	if(prev) prev++;
      }

      menu_state->selected = 1;   // start by highlighting '..'
      menu_state->scroll = 0;
      menu_state->dir = sdc_readdir(drive, entry->name, (void*)fsel_state.ext);	
      
      // prev is still valid, since sdc_readdir doesn't free the old string when going
      // up one directory. Instead it just terminates it in the middle	
      if(prev) {
	menu_debugf("up to %s", prev);
	
	// try to find previous dir entry in current dir	  
	for(int i=0;i<dir_len(menu_state->dir);i++) {
	  if(dir_entry(menu_state->dir, i)->is_dir && strcmp(dir_entry(menu_state->dir, i)->name, prev) == 0) {
	    // file found, adjust entry and offset
	    menu_state->selected = i+1;

	    if(dir_len(menu_state->dir) > 4 && menu_state->selected > 3) {
	      if(menu_state->selected < dir_len(menu_state->dir) - 1) menu_state->scroll = menu_state->selected - 3;
	      else                                                    menu_state->scroll = menu_state->selected - 5;
	    }
	  }
	}
      }
    }
  } else {
    // request insertion of this image
    sdc_image_open(drive, entry->name);
    
    // return to parent form
    menu_pop();

    // check if this is a drive image selection and run action if yes. Image selectors
    // work differently and do an IRQ driven transfer in the background. The action is there executed
    // ocne the transfer itself is done.
    if(drive<MAX_DRIVES && fsel_state.action)
      sys_run_action(fsel_state.action);
  }
}

void menu_run_current_image_action(void) {
  // Note: The download may also have been triggered when loading config from
  // ini file. We may need to find the right action in that case as well.
  // The same is true for mounted disk images
  
  if(!menu_state) return;

  if(menu_switching) return;

  // the current menu state will be a open file selector if the image transfer
  // has been started through the OSD by the user
  if(menu_state->type != MENU_TYPE_FILESELECTOR) {
    menu_debugf("finished download was ini file triggered during boot");

    if(!sdc_check_for_pending_image_uploads()) {
      menu_debugf("no more image uploads pending");

      sys_run_action_by_name("ready");
    }
      
    return;
  }

  // fsel index should be >= MAX_DRIVES and < MAX_DRIVES+MAX_IMAGES as it
  // should point to an image selector
  if(fsel_state.index < MAX_DRIVES || fsel_state.index >= MAX_DRIVES+MAX_IMAGES) return;

  // run the file selector action if one is present
  if(fsel_state.action)
    sys_run_action(fsel_state.action);

}

// user has pressed esc to go back one level
static void menu_back(void) {
  // when in range edit mode, back leaves edit mode
  config_menu_entry_t *entry = menu_get_selected_entry();
  if(entry && entry->type == CONFIG_MENU_ENTRY_RANGE && entry->range->edit) {
    entry->range->edit = false;
    return;
  }							  

  // stop doing the scroll timer
  menu_timer_enable(false);

  // are we in the root menu?
  if(menu_state->menu == cfg->menu)
    osd_enable(OSD_INVISIBLE);
  else {
    // are we in fileselector?
    if(menu_state->type == MENU_TYPE_FILESELECTOR) {
      // search for ".." in current dir
      sdc_dir_entry_t *entry = NULL;
      for(int i=0;i<dir_len(menu_state->dir);i++)
	if(!strcmp(dir_entry(menu_state->dir, i)->name, ".."))
	  entry = dir_entry(menu_state->dir, i);
      
      // if there was one, go up. Else quit the file selector
      if(entry) menu_fileselector_select(entry);
      else      menu_pop();
    } else
      menu_pop();
  }
}

// user has selected a menu entry
static void menu_select(void) {
  // when in range edit mode, select leaves edit mode
  config_menu_entry_t *entry = menu_get_selected_entry();
  if(entry && entry->type == CONFIG_MENU_ENTRY_RANGE && entry->range->edit) {
    entry->range->edit = false;
    return;
  }							  

  // if the title was selected, then goto parent form
  if(menu_state->selected == 0) {
    menu_pop();
    return;
  }

  // in fileselector
  if(menu_state->type == MENU_TYPE_FILESELECTOR) {
    menu_fileselector_select(dir_entry(menu_state->dir, menu_state->selected-1));
    return;
  }
  
  menu_debugf("Selected: %s '%s'", config_menuentry_get_type_str(entry), menuentry_get_label(entry));

  switch(entry->type) {
  case CONFIG_MENU_ENTRY_FILESELECTOR:
    // user has choosen a file selector
    menu_file_selector_open(entry);
    break;
    
  case CONFIG_MENU_ENTRY_MENU:
    menu_goto(entry->menu);
    break;

  case CONFIG_MENU_ENTRY_LIST: {
    // user has choosen a selection list: go to the next entry in list
    // order and back to the first one after the last. The values need not
    // be 0..n-1, e.g. DIP switch settings like 3, 2, 1
    config_listentry_t *le = entry->list->listentries;
    int value = menu_variable_get(entry->list->id);
    while(le && le->value != value) le = le->next;
    le = (le && le->next) ? le->next : entry->list->listentries;
    if(le) menu_variable_set(entry->list->id, le->value);

    // check if there's an action connected to changing this
    // list. This e.g. happens when changing system settings is
    // meant to trigger a (cold) boot
    if(entry->list->action)
      sys_run_action(entry->list->action);

  } break;

  case CONFIG_MENU_ENTRY_BUTTON:
    if(entry->button->action)
      sys_run_action(entry->button->action);
    break;
	
  case CONFIG_MENU_ENTRY_IMAGE:
    // user has choosen an image selector
    menu_file_selector_open(entry);
    break;
    
  case CONFIG_MENU_ENTRY_TOGGLE:
    menu_variable_set(entry->toggle->id, !menu_variable_get(entry->toggle->id));
    if(entry->toggle->action)
      sys_run_action(entry->toggle->action);
    break;

  case CONFIG_MENU_ENTRY_RANGE:
    // selecting a range entry enables changing its value
    entry->range->edit = true;
    break;

  case CONFIG_MENU_ENTRY_CFGSELECTOR:
    menu_cfgsel_open(entry);
    break;
    
  default:
    menu_debugf("unknown %s", config_menuentry_get_type_str(entry));    
  }
}

// timer implementing key repeat
static TimerHandle_t menu_key_repeat_timer = NULL;  
static int menu_key_last_event = -1;

static void menu_key_repeat(__attribute__((unused)) TimerHandle_t arg) { 
  if(menu_key_last_event >= 0) {
  
    if(menu_key_last_event == MENU_EVENT_UP)     menu_entry_go(-1);
    if(menu_key_last_event == MENU_EVENT_DOWN)   menu_entry_go( 1);

    if(menu_key_last_event == MENU_EVENT_PGUP)   menu_entry_go(-4);
    if(menu_key_last_event == MENU_EVENT_PGDOWN) menu_entry_go( 4);

    if(menu_state->type == MENU_TYPE_MENU)
      menu_draw(menu_state->menu, menu_state->selected, menu_state->scroll);
    else if(menu_state->type == MENU_TYPE_FILESELECTOR)
      menu_fsel_draw(menu_state->label, menu_state->dir, menu_state->selected, menu_state->scroll);
    else if(menu_state->type == MENU_TYPE_CUSTOM)
      menu_custom_draw(menu_state->custom, menu_state->selected, menu_state->scroll);
  
    xTimerChangePeriod( menu_key_repeat_timer, pdMS_TO_TICKS(100), 0);
    xTimerStart( menu_key_repeat_timer, 0 );
  }
}
  
void menu_stop_repeat(void) {
  xTimerStop(menu_key_repeat_timer, 0);
  menu_key_last_event = -1;
}

void menu_do(int event) {
  // see menu_select_swallowed()
  if(event == MENU_EVENT_SELECT && menu_select_swallowed())
    return;

  // The question to delete a config only reacts to y, n and escape. A select (enter,
  // space, fire) would leave it like escape does, so it's ignored.
  if(event == MENU_EVENT_SELECT && menu_delcfg_active() && !menu_dialog_is_open())
    return;

  if(menu_dialog_is_open()) {
    // if the dialog is open, then any key event will close it
    if((event >= MENU_EVENT_UP) && (event <= MENU_EVENT_BACK))
      menu_dialog_close();

    return;
  }

  // -1 is a timer event used to scroll the current file name if it's to long
  // for the OSD
  if(event < 0) {
    if(cfg) {
      if(menu_state->type == MENU_TYPE_FILESELECTOR)
	menu_fs_scroll_entry();
    }
      
    return;
  }
  
  menu_debugf("do %d", event);
  
  if(event)  {
    if(event == MENU_EVENT_TOGGLE) {
      if(!osd_is_visible())
	osd_enable(OSD_VISIBLE);
      else {
	menu_timer_enable(false);
	osd_enable(OSD_INVISIBLE);
	return;  // return now to prevent OSD from being drawn, again
      }
    }

    // a key release event just stops any repeat timer
    if(event == MENU_EVENT_KEY_RELEASE) {
      menu_stop_repeat();
      return;
    }

    // UP/DOWN PGUP and PGDOWN have a repeat
    if(event == MENU_EVENT_UP || event == MENU_EVENT_DOWN ||
       event == MENU_EVENT_PGUP || event == MENU_EVENT_PGDOWN) {
      
      if(menu_key_repeat_timer) {
	menu_key_last_event = event;
	xTimerChangePeriod( menu_key_repeat_timer, pdMS_TO_TICKS(500), 0);
	xTimerStart( menu_key_repeat_timer, 0 );
      }
    }
    
    if(event == MENU_EVENT_UP)     menu_entry_go(-1);
    if(event == MENU_EVENT_DOWN)   menu_entry_go( 1);

    if(event == MENU_EVENT_PGUP)   menu_entry_go(-4);
    if(event == MENU_EVENT_PGDOWN) menu_entry_go( 4);

    if(event == MENU_EVENT_SELECT) menu_select();
    if(event == MENU_EVENT_BACK)   menu_back();
  }

  // if no dialog is open, then draw menu/fsel
  if(!menu_dialog_is_open()) {  
    if(menu_state->type == MENU_TYPE_MENU)
      menu_draw(menu_state->menu, menu_state->selected, menu_state->scroll);
    else if(menu_state->type == MENU_TYPE_FILESELECTOR)
      menu_fsel_draw(menu_state->label, menu_state->dir, menu_state->selected, menu_state->scroll);  
    else if(menu_state->type == MENU_TYPE_CUSTOM)
      menu_custom_draw(menu_state->custom, menu_state->selected, menu_state->scroll);
  }
}

TimerHandle_t menu_timer_handle;
// queue to forward key press events from USB to MENU
QueueHandle_t menu_queue = NULL;

void menu_timer_enable(bool on) {
  if(on) xTimerStart(menu_timer_handle, 0);
  else   xTimerStop(menu_timer_handle, 0);
}

// a 25Hz timer that can be activated by the menu whenever animations
// are displayed and which should be updated constantly
static void menu_timer(__attribute__((unused)) TimerHandle_t pxTimer) {
  static long msg = -1;
  xQueueSendToBack(menu_queue, &msg,  ( TickType_t ) 0);
}

static const config_menu_t system_menu_main;

// check if menu is the system menu or a submenu of it
static bool menu_is_systemmenu(void) {
  menu_state_t *ms = menu_state;
  
  while(ms) {
    if((ms->type == CONFIG_MENU_ENTRY_MENU) &&
       (ms->menu == &system_menu_main))
      return true;

    ms = ms->prev;
  }
  return false;
}  

static void menu_handle_latin1(uint8_t code) {
  // this function receives lagtin1 encoded keyboard events and
  // is meant to be used for complex text input in custom dialogs  
  if(code > 31) menu_debugf("menu_handle_latin1(%d/%c)", code, code);
  else          menu_debugf("menu_handle_latin1(%d)", code);

  if(menu_dialog_is_open()) return;

  if(menu_newcfg_active())                 // typing in the name of a new config
    menu_newcfg_key(code);
  else if(menu_delcfg_active())            // yes or no
    menu_delcfg_key(code);
  else if(code == 127 && menu_state && menu_state->type == MENU_TYPE_FILESELECTOR && fsel_state.cfgsel)
    menu_delcfg_open();                    // delete key in the list of configs
}

static void menu_task(__attribute__((unused)) void *parms) {
  menu_debugf("task running");

  // wait for user events
  while(1) {
    // receive events from usb    
    long cmd;
    xQueueReceive(menu_queue, &cmd, 0xffffffffUL);
    menu_debugf("command %04lx", cmd);

    if(cmd == MENU_EVENT_SYSTEM) {
      menu_debugf("system menu requested");
      menu_dialog_close();
      
      // open osd if it's not open, yet
      if(!osd_is_visible()) osd_enable(OSD_VISIBLE);

      // check if we are already in system menu
      if(!menu_is_systemmenu()) {
	menu_goto(&system_menu_main);
	menu_do(0);   // (re)draw menu
      } else
	menu_debugf("Already in system menu");
    } else

    // catch some non-user controlled events here

#ifdef ENABLE_BLUETOOTH
    if(cmd == MENU_EVENT_BLUETOOTH_CONNECTED) {      
      menu_draw_dialog("Bluetooth", "A device has been connected!");
    } else if(cmd == MENU_EVENT_BLUETOOTH_DISCONNECTED) {
      menu_draw_dialog("Bluetooth", "A device has been disconnected!");
    } else if(cmd == MENU_EVENT_BLUETOOTH_SCAN) {
      menu_draw_dialog("Bluetooth", "Searching for 10 seconds for devices ...");
    } else if(cmd == MENU_EVENT_BLUETOOTH_PIN_CODE_REQUEST) {
      menu_draw_dialog("Bluetooth", "Please enter pin '123456'");
    } else
#endif

    if(cmd == MENU_EVENT_NETWORK_GOT_IP) {
      char message[32];
      snprintf(message, sizeof(message), "IP: %s", network_ipaddr);
      menu_draw_dialog_for("Network", message, pdMS_TO_TICKS(5000));
    } else if(cmd == MENU_EVENT_NETWORK_DISCONNECTED) {
      menu_draw_dialog_for("Network", "disconnected", pdMS_TO_TICKS(3000));
    } else
  if(cmd == MENU_EVENT_USB_MOUNTED) {
      menu_debugf("USB mount event");

      vTaskDelay(100);

      static FATFS usb_fs;
      FRESULT fres;
      if ( (fres = f_mount(&usb_fs, "/usb", 1)) != FR_OK ) menu_debugf("/usb mount failed: %d", fres);
      else {
	menu_debugf("/usb mounted");
	menu_draw_dialog("USB", "A USB mass storage device has been detected!");
      }
    } else if(cmd == MENU_EVENT_USB_UMOUNTED) {
      if (f_unmount("/usb") != FR_OK )	menu_debugf("/usb unmount failed");
      else	                        menu_debugf("/usb unmounted");
      menu_draw_dialog("USB", "A USB mass storage device has been removed!");
    }

    else if((cmd & 0xffffff00) == MENU_EVENT_KEY_LATIN1)
      menu_handle_latin1(cmd & 0xff);

    else
      menu_do(cmd);
  }
}

TaskHandle_t menu_handle = NULL;

void menu_init(void) {
  menu_debugf("Initializing");

  // check if a config was loaded. If no, use the legacy menu
  if(!cfg) {  
    menu_debugf("Warning: No core config found. Is this an old legacy core?");
    return;
  }

  // a config was loaded, use that
  menu_debugf("Using configured menu");

  menu_debugf("Setting up variables");
  menu_setup_variables();
    
  menu_debugf("Processing init action");
  sys_run_action_by_name("init");

  // load the config selected last, if there's a config selector
  menu_cfgsel_boot();

  menu_goto(cfg->menu);    

  // create a one shot timer for key repeat
  menu_key_repeat_timer = xTimerCreate( "Key repeat timer", pdMS_TO_TICKS(500), pdFALSE,
					NULL, menu_key_repeat);
    
  // switch MCU controlled leds off
  sys_set_leds(0x00);
  
  // create a 25 Hz timer that frequently wakes the OSD thread
  // allowing for animations
  menu_timer_handle = xTimerCreate("Menu scroll timer", pdMS_TO_TICKS(40), pdTRUE,
				   NULL, menu_timer);
  
  // message queue from USB to OSD
  menu_queue = xQueueCreate(10, sizeof( long ) );
  
  // start a thread for the on screen display    
  xTaskCreate(menu_task, (char *)"menu_task", 4096, NULL, configMAX_PRIORITIES-3, &menu_handle);

  // At this point, the USB may already be ready. But since the
  // menu task wasn't ready by now, it never had a chance to be mounted properly
  if(mcu_hw_usb_msc_present()) {
    menu_debugf("triggering delayed USB init");
    menu_notify(MENU_EVENT_USB_MOUNTED);
  }
}

// queue an event for the menu task
void menu_notify(unsigned long msg) {
  if(menu_queue) 
    xQueueSendToBackFromISR(menu_queue, &msg,  ( TickType_t ) 0);
  else
    menu_debugf("menu_notify(): queue/menu task not ready!");
}

void menu_notify_ip(const char *ipaddr) {
  snprintf(network_ipaddr, sizeof(network_ipaddr), "%s", ipaddr);
  network_was_connected = true;
  menu_notify(MENU_EVENT_NETWORK_GOT_IP);
}

void menu_notify_network_disconnected(void) {
  // only report a disconnect if we had reported an ip address before
  if(!network_was_connected) return;
  network_was_connected = false;
  menu_notify(MENU_EVENT_NETWORK_DISCONNECTED);
}

void menu_joystick_state(unsigned char state) {
  static unsigned char prev_state = 0;

  if(state != prev_state) {
    static unsigned long msg;
    menu_debugf("Joystick state change to %02x", state);

    msg = 0;
    if(state & 0x08) msg = MENU_EVENT_UP;      
    if(state & 0x04) msg = MENU_EVENT_DOWN;      
    if(state & 0x02) msg = MENU_EVENT_BACK;      
    if(state & 0x10) msg = MENU_EVENT_SELECT;
    if(state & 0x20) msg = MENU_EVENT_BACK;
    if(!msg) msg = MENU_EVENT_KEY_RELEASE;
    menu_notify(msg);

    prev_state = state;
  }
}

// ===========  "single button" menu control ================

// The menu is only controlled through the single OSD hw button
// whenever no keyboard or gamepad is connected.

// three events are supported:
// <200ms: next menu entry
// >200ms and < 3s: select menu entry
// >3s: up one menu level

static TickType_t menu_button_last_event = 0;
static TimerHandle_t menu_button_timer = NULL;

static void menu_button_timer_handler(__attribute__((unused)) TimerHandle_t arg) {
  menu_debugf("button timer event");

  menu_notify(MENU_EVENT_BACK);
  menu_notify(MENU_EVENT_KEY_RELEASE);
  menu_button_last_event = 0;
}

void menu_button_state(unsigned char state) {
  menu_debugf("Button state %02x", state);

  if(mcu_hw_hid_present()) {
    if(state & 0x01) {
      // with keyboard and/or gamepad detected implement the same behaviour as
      // previous versions had
      menu_notify(MENU_EVENT_TOGGLE);
    }
  } else {
    static unsigned char prev_state = 0;

    if(state != prev_state) {
      menu_debugf("Button state change to %02x", state);

      // button has been pressed?
      if(state & 1) {
	// show the osd if it's closed
	if(!osd_is_visible())
	  menu_notify(MENU_EVENT_TOGGLE);
	else {	
	  menu_button_last_event = xTaskGetTickCount();
	  
	  if(!menu_button_timer)
	    menu_button_timer = xTimerCreate( "Button timer", pdMS_TO_TICKS(3000), pdFALSE,
					      NULL, menu_button_timer_handler);
	  
	  xTimerStart(menu_button_timer, 0);
	}
      } else if(menu_button_last_event) {
	xTimerStop(menu_button_timer, 0);

	// button action depends on press duration
	TickType_t len = xTaskGetTickCount() - menu_button_last_event;
	if(len > pdMS_TO_TICKS(200)) {
	  menu_notify(MENU_EVENT_SELECT);
	  menu_notify(MENU_EVENT_KEY_RELEASE);
	} else {
	  menu_notify(MENU_EVENT_DOWN);
	  menu_notify(MENU_EVENT_KEY_RELEASE);
	}
	menu_button_last_event = 0;
      }
	
      prev_state = state;
    }
  }
}

/* ================= system menu =================== */

/*  For simplicity and flexibility, the system menu is
    not strictly bound to use the menu structures. Instead
    it can use programmatically designed dialogs.

    Its about box is an example for this.
*/

static int about_length(void) {
  // return the number of selectable items inside the about dialog.
  return 0;
}

static void about_draw(void) {
  u8g2_DrawStr(&u8g2, 20, MENU_LINE_Y + 1*MENU_ENTRY_H, "FPGA Companion");
  u8g2_DrawStr(&u8g2, 5, MENU_LINE_Y + 2*MENU_ENTRY_H, "(c) 2026 the MiSTle Team");
  u8g2_DrawStr(&u8g2, 0, MENU_LINE_Y + 3*MENU_ENTRY_H, "http://github.com/mistle-dev");
}

const static config_custom_t about = {
  .label = "About",
  .length = about_length,
  .draw = about_draw
};

static void about_func(void) {
  menu_debugf("About");

  // push a custom menu
  menu_push();
  menu_state->type = MENU_TYPE_CUSTOM;
  menu_state->custom = &about;
  menu_state->selected = 0;
  menu_state->scroll = 0;
}

static const config_action_command_t about_exec = {
  .code = CONFIG_ACTION_COMMAND_EXEC,
  .exec = about_func
};

static const config_action_t about_action = {
  .name = "about",
  .commands = (config_action_command_t*)&about_exec
};

static const config_button_t about_btn = {
  .label = "About",
  .action = (config_action_t*)&about_action
};


// the system menu is hard coded as it doesn't need to
// be modified by the running core

#ifdef ENABLE_BLUETOOTH

void btscan(void) {
  menu_notify(MENU_EVENT_BLUETOOTH_SCAN);
  bluetooth_scan();
}

static const config_action_command_t bluetooth_exec = {
  .code = CONFIG_ACTION_COMMAND_EXEC,
  .exec = btscan
};

static const config_action_t bluetooth_action = {
  .name = "btscan",
  .commands = (config_action_command_t*)&bluetooth_exec
};

static const config_button_t bluetooth_btn = {
  .label = "Scan Bluetooth",
  .action = (config_action_t*)&bluetooth_action
};

static const config_menu_entry_t system_menu_bluetooth = {
  .type = CONFIG_MENU_ENTRY_BUTTON,
  .button = (config_button_t*)&bluetooth_btn
};
#endif

// the usb core file selector
static char *core_exts[] = { "fs", "bin", NULL }; 

#if MAX_CORES < 2
#error "Setup at least 2 cores in config.h"
#endif

#ifdef DIRECT_SDC_SUPPORTED
static const config_fsel_t sdc_core_fsel = {
  .index = MAX_DRIVES+MAX_IMAGES+1,
  .label = "Load Core from SD",
  .def = "/sd/core.bin",
  .ext = (char**)core_exts
};

// second entry in main system menu
static const config_menu_entry_t system_menu_sdc_core_fsel = {
  .type = CONFIG_MENU_ENTRY_FILESELECTOR,
  .fsel = (config_fsel_t*)&sdc_core_fsel,
#ifdef ENABLE_BLUETOOTH
  .next = (config_menu_entry_t*)&system_menu_bluetooth
#endif
};
#endif

static const config_fsel_t usb_core_fsel = {
  .index = MAX_DRIVES+MAX_IMAGES+0,
  .label = "Load Core from USB",
  .def = "/usb/core.bin",
  .ext = (char**)core_exts
};

// second entry in main system menu
static const config_menu_entry_t system_menu_usb_core_fsel = {
  .type = CONFIG_MENU_ENTRY_FILESELECTOR,
  .fsel = (config_fsel_t*)&usb_core_fsel,
#ifdef DIRECT_SDC_SUPPORTED
  .next = (config_menu_entry_t*)&system_menu_sdc_core_fsel
#else
#ifdef ENABLE_BLUETOOTH
  .next = (config_menu_entry_t*)&system_menu_bluetooth
#endif
#endif
};

// first entry in main system menu
static const config_menu_entry_t system_menu_about = {
  .type = CONFIG_MENU_ENTRY_BUTTON,
  .button = (config_button_t*)&about_btn,
  .next = (config_menu_entry_t*)&system_menu_usb_core_fsel
};

// the main system menu
static const config_menu_t system_menu_main = {
  .label = "Companion",
  .entries = (config_menu_entry_t*)&system_menu_about
};

