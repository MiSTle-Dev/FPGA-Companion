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

/* new menu state */
typedef struct {
  int type;
  int selected; 
  int scroll;
  union {
    config_menu_t *menu;  
    config_fsel_t *fsel;
  };
  
  // file selector related
  sdc_dir_t *dir;
} menu_state_t;

static menu_state_t *menu_state = NULL;

/* =========== handling of variables ============= */
static menu_variable_t **variables = NULL;

menu_variable_t **menu_get_variables(void) {
  return variables;
}

static int menu_variable_get(char id) {
  for(int i=0;variables[i];i++)
    if(variables[i]->id == id)
      return variables[i]->value;

  return 0;  
}

static void menu_variable_set(char id, int value) {
  for(int i=0;variables[i];i++) {
    if(variables[i]->id == id) {
      if(variables[i]->value != value) {
	variables[i]->value = value;
	// also set this in the core
	sys_set_val(id, value);
      }
    }
  }
}

static void menu_setup_variable(char id, int value) {
  // menu_debugf("setup variable '%c' = %d", id, value);

  // simply return if variable already exists
  int i;
  for(i=0;variables[i];i++)
    if(variables[i]->id == id)
      return;

  // allocate new entry
  menu_variable_t *variable = malloc(sizeof(menu_variable_t));
  variable->id = id;
  variable->value = value;

  // menu_debugf("new variable index %d", i);
  variables = reallocarray(variables, i+2, sizeof(menu_variable_t*));
  variables[i] = variable;
  variables[i+1] = NULL;
}

static void menu_setup_menu_variables(config_menu_t *menu) {
  config_menu_entry_t *me = menu->entries;
  for(int cnt=0;me[cnt].type != CONFIG_MENU_ENTRY_UNKNOWN;cnt++) {
    if(me[cnt].type == CONFIG_MENU_ENTRY_MENU)
      menu_setup_menu_variables(me[cnt].menu);
    
    if(me[cnt].type == CONFIG_MENU_ENTRY_LIST) {
      // setup variable ...
      menu_setup_variable(me[cnt].list->id, me[cnt].list->def);
      // ... and set in core
      sys_set_val(me[cnt].list->id, me[cnt].list->def);
    }
  }
}

static void menu_setup_variables(void) {
  // variables occur in two places:
  // in the set command used in actions
  // in menu items (currently only in lists as buttons use actions)

  // actually variables should always show up in the init action,
  // otherwise they'd be uninitialited (actually set to zero ...)
  
  // add null pointer as end marker
  variables = malloc(sizeof(menu_variable_t *));
  variables[0] = NULL;
  
  // search for variables in all actions
  for(int a=0;cfg->actions[a];a++)
    // search for set commands
    for(int c=0;cfg->actions[a]->commands[c].code != CONFIG_ACTION_COMMAND_IDLE;c++)
      if(cfg->actions[a]->commands[c].code == CONFIG_ACTION_COMMAND_SET)
	menu_setup_variable(cfg->actions[a]->commands[c].set.id, 0);

  // search through menu tree for lists
  menu_setup_menu_variables(cfg->menu);  
}


void menu_set_value(unsigned char id, unsigned char value) {
  menu_variable_set(id, value);
}

// various 8x8 icons
static const unsigned char icn_right_bits[]  = { 0x00,0x04,0x0c,0x1c,0x3c,0x1c,0x0c,0x04 };
static const unsigned char icn_left_bits[]   = { 0x00,0x20,0x30,0x38,0x3c,0x38,0x30,0x20 };
static const unsigned char icn_floppy_bits[] = { 0xff,0x81,0x83,0x81,0xbd,0xad,0x6d,0x3f };
static const unsigned char icn_empty_bits[] =  { 0xc3,0xe7,0x7e,0x3c,0x3c,0x7e,0xe7,0xc3 };

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

static void menu_fs_scroll_entry(void) {
  // no scrolling
  if(fs_scroll_cur < 0) return;
  
  // don't scroll anything else
  if(menu_state->type != CONFIG_MENU_ENTRY_FILESELECTOR) return;
  
  int row = menu_state->selected - 1;
  int y =  MENU_LINE_Y + MENU_ENTRY_H * (row-menu_state->scroll+1);
  int width = u8g2_GetDisplayWidth(&u8g2);

  int swid = u8g2_GetStrWidth(&u8g2, menu_state->dir->files[row].name) + 1;

  // fill the area where the scrolling entry would show
  u8g2_SetClipWindow(&u8g2, FS_ICON_WIDTH, y-MENU_ENTRY_BASE, width, y+MENU_ENTRY_H-MENU_ENTRY_BASE);  
  u8g2_DrawBox(&u8g2, FS_ICON_WIDTH, y-MENU_ENTRY_BASE, width-FS_ICON_WIDTH, MENU_ENTRY_H);
  u8g2_SetDrawColor(&u8g2, 0);

  int scroll = fs_scroll_cur++ - 25;   // 25 means 1 sec delay
  if(fs_scroll_cur > swid-width+FS_ICON_WIDTH+50) fs_scroll_cur = 0;
  if(scroll < 0) scroll = 0;
  if(scroll > swid-width+FS_ICON_WIDTH) scroll = swid-width+FS_ICON_WIDTH;
  
  u8g2_DrawStr(&u8g2, FS_ICON_WIDTH-scroll, y, menu_state->dir->files[row].name);
  
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
  
  char str[strlen(entry->name)+1];
  int y =  MENU_LINE_Y + MENU_ENTRY_H * (row+1);

  // ignore leading / used by special entries
  if(entry->name[0] == '/') strcpy(str, entry->name+1);
  else                      strcpy(str, entry->name);
  
  int width = u8g2_GetDisplayWidth(&u8g2);
  
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
  
  // draw folder icon in front of directories
  if(entry->is_dir)
    u8g2_DrawXBM(&u8g2, 1, y-8, 8, 8,
		 (entry->name[0] == '/')?empty_icon:
		 strcmp(entry->name, "..")?folder_icon:
		 up_icon);

  if(menu_state->selected == row+menu_state->scroll+1)
    u8g2_DrawButtonFrame(&u8g2, 0, y, U8G2_BTN_INV, width, 1, 1);
}

static void menu_push(void) {
  // count existing state entries as the last one would
  // have the menu pointer being the root menu
  // pointer
  int i=0;
  if(menu_state) {
    while(menu_state[i].menu != cfg->menu) i++;
    i++;
  }

  menu_debugf("stack depth %d", i);
  
  menu_state = reallocarray(menu_state, i+1, sizeof(menu_state_t));
  // move all existing entries up one
  if(i) {
    for(int j=i-1;j>=0;j--) {
      debugf("move state %d to %d", j, j+1);
      menu_state[j+1] = menu_state[j];
    }
  }
}

static int menu_count_entries(void) {
  int entries = 0;

  if(menu_state->type == CONFIG_MENU_ENTRY_MENU)
    while(menu_state->menu->entries[entries].type != CONFIG_MENU_ENTRY_UNKNOWN)
      entries++;
  else if(menu_state->type == CONFIG_MENU_ENTRY_FILESELECTOR)
    entries = menu_state->dir->len;
    
  return entries+1;  // title is also an entry
}

static bool menu_is_root(void) {
  return menu_state->menu == cfg->menu;
}

static int menu_entry_is_usable(void) {
  // not root menu? Then all entries are usable
  if(!menu_is_root()) return 1;

  // in root menu only the title is unusable
  return menu_state->selected != 0;
}

static void menu_entry_go(int step) {
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

static void menu_draw_title(const char *s, bool arrow, bool selected) {
  int x = 1;

  // draw left arrow for submenus
  if(arrow) {
    u8g2_DrawXBM(&u8g2, 0, 1, 8, 8, icn_left_bits);    
    x = 8;
  }

  // draw title in bold and seperator line
  u8g2_SetFont(&u8g2, u8g2_font_helvB08_tr);
  u8g2_DrawStr(&u8g2, x, MENU_ENTRY_BASE, s);
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
  
  return NULL;
}

static int menu_get_list_length(config_menu_entry_t *entry) {
  int len = 0;  
  for(;entry->list->listentries[len];len++);
  return len-1;
}
  
static char *menu_get_listentry(config_menu_entry_t *entry, int value) {
  if(!entry || entry->type != CONFIG_MENU_ENTRY_LIST) return NULL;

  for(int i=0;entry->list->listentries[i];i++)
    if(entry->list->listentries[i]->value == value)
      return entry->list->listentries[i]->label;

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
  int hl_x = 0;
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
		  
    hl_x = width/2;
    hl_w = width/2;
  }
  
  // some entries have a small icon to the right    
  if(entry->type == CONFIG_MENU_ENTRY_MENU)
    u8g2_DrawXBM(&u8g2, hl_w-8, ypos-8, 8, 8, icn_right_bits);
  if(entry->type == CONFIG_MENU_ENTRY_FILESELECTOR) {
    // icon depends if floppy is inserted
    u8g2_DrawXBM(&u8g2, hl_w-MENU_ENTRY_BASE, ypos-8, 8, 8,
	 sdc_get_image_name(entry->fsel->index)?icn_floppy_bits:icn_empty_bits);
  }
  
  if(selected)
    u8g2_DrawButtonFrame(&u8g2, hl_x, ypos, U8G2_BTN_INV, hl_w, 1, 1);
}

static int menu_wrap_text(int y_in, const char *msg) {  
  // fetch words until the width is exceeded
  const char *p = msg;
  char *b = NULL;
  int y = y_in;
  
  u8g2_SetFont(&u8g2, font_helvR08_te);
  while(*msg && *p) {
    // search for end of word
    while(*p && *p != ' ') p++;
    
    // allocate substring
    b = realloc(b, p-msg+1);
    strncpy(b, msg, p-msg);
    b[p-msg]='\0';
    
    // check if this is now too long for screen
    if((u8g2_GetStrWidth(&u8g2, b) >  u8g2_GetDisplayWidth(&u8g2))) {
      // cut last word to fit to screen
      while(*p == ' ') p--;
      while(*p != ' ') p--;
      b[p-msg]='\0';

      if(y_in) u8g2_DrawStr(&u8g2, (u8g2_GetDisplayWidth(&u8g2)-u8g2_GetStrWidth(&u8g2, b))/2, y, b);
      y+=11;
      
      msg = ++p;
    }
    while(*p == ' ') p++;
  }
  
  if(y_in) u8g2_DrawStr(&u8g2, (u8g2_GetDisplayWidth(&u8g2)-u8g2_GetStrWidth(&u8g2, b))/2, y, b);
  y+=11;

  free(b);

  return y;
}

// draw a dialog box
void menu_draw_dialog(const char *title,  const char *msg) {
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

void menu_draw(void) {
  // draw a test dialog box
  //  menu_draw_dialog("Title", "This is a rather long text which needs to wrap!");  return;
  
  u8g2_ClearBuffer(&u8g2);
 
  if(menu_state->type == CONFIG_MENU_ENTRY_MENU) {
    // =============== draw a regular menu =================
    menu_debugf("drawing '%s'", menu_state->menu->label);  
    
    // draw the title
    menu_draw_title(menu_state->menu->label, !menu_is_root(), menu_state->selected == 0);

    // draw up to four entries
    config_menu_entry_t *entry = menu_state->menu->entries;
    for(int i=0;i<4 && entry[i].type != CONFIG_MENU_ENTRY_UNKNOWN;i++)
      menu_draw_entry(entry+i+menu_state->scroll, i, menu_state->selected == menu_state->scroll+i+1);    
  } else {
    // =============== draw a fileselector =================    
    menu_debugf("drawing '%s'", menu_state->fsel->label);
    
    menu_draw_title(menu_state->fsel->label, true, menu_state->selected == 0);
    menu_timer_enable(false);
    fs_scroll_cur = -1;

    // draw up to four entries
    for(int i=0;i<4 && i<menu_state->dir->len-menu_state->scroll;i++) {            
      debugf("file %s", menu_state->dir->files[i+menu_state->scroll].name);

      menu_fs_draw_entry(i, &menu_state->dir->files[i+menu_state->scroll]);
    }
  }
    
  u8g2_SendBuffer(&u8g2);
}

void menu_goto(config_menu_t *menu) {
  menu_push();
  
  // prepare menu state ...
  menu_state->menu = menu;
  menu_state->selected = 1;
  menu_state->scroll = 0;
  menu_state->type = CONFIG_MENU_ENTRY_MENU;
}

static void menu_file_selector_open(config_menu_entry_t *entry) {
  menu_push();
  menu_state->fsel = entry->fsel;
  menu_state->selected = 1;
  menu_state->scroll = 0;
  menu_state->type = CONFIG_MENU_ENTRY_FILESELECTOR;
  
  // scan file system
  menu_state->dir = sdc_readdir(entry->fsel->index, NULL, (void*)entry->fsel->ext);
  // try to jump to current file. Get the current image name and path
  char *name = sdc_get_image_name(entry->fsel->index);
  if(name) {
    debugf("trying to jump to %s", name);
    // try to find name in file list
    for(int i=0;i<menu_state->dir->len;i++) {
      if(strcmp(menu_state->dir->files[i].name, name) == 0) {
	debugf("found preset entry %d", i);
	
	// file found, adjust entry and offset
	menu_state->selected = i+1;
	
	if(menu_state->dir->len > 4 && menu_state->selected > 3) {
	  debugf("more than 4 files an selected is > 3");
	  if(menu_state->selected < menu_state->dir->len-1) menu_state->scroll = menu_state->selected - 3;
	  else                                              menu_state->scroll = menu_state->dir->len-4;
	}
      }
    }
  }  
}

// all other entries in step down
static void menu_pop(void) {
  // this should never happen ...
  if(!menu_state) return;

  // neither should this as we never really close the
  // root menu
  if(menu_state->menu == cfg->menu) {
    free(menu_state);
    menu_state = NULL;
    return;
  }
    
  // count number of entries
  int i=1; while(menu_state[i-1].menu != cfg->menu) i++;
  menu_debugf("pop stack depth %d", i);

  if(i>10) exit(0);
  
  // move all existing entries down one
  for(int j=0;j<i-1;j++) {
    debugf("move state %d to %d", j+1, j);
    menu_state[j] = menu_state[j+1];
  }  

  menu_state = reallocarray(menu_state, i-1, sizeof(menu_state_t));
}

static void menu_fileselector_select(sdc_dir_entry_t *entry) {
  int drive = menu_state->fsel->index;
  debugf("drive %d, file selected '%s'", drive, entry->name);
    
  // stop any scroll timer that might be running
  menu_timer_enable(false);
    
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
      menu_state->dir = sdc_readdir(drive, entry->name, (void*)menu_state->fsel->ext);	
      
      // prev is still valid, since sdc_readdir doesn't free the old string when going
      // up one directory. Instead it just terminates it in the middle	
      if(prev) {
	menu_debugf("up to %s", prev);
	
	// try to find previous dir entry in current dir	  
	for(int i=0;i<menu_state->dir->len;i++) {
	  if(menu_state->dir->files[i].is_dir && strcmp(menu_state->dir->files[i].name, prev) == 0) {
	    // file found, adjust entry and offset
	    menu_state->selected = i+1;

	    if(menu_state->dir->len > 4 && menu_state->selected > 3) {
	      if(menu_state->selected < menu_state->dir->len - 1) menu_state->scroll = menu_state->selected - 3;
	      else                                                menu_state->scroll = menu_state->selected - 5;
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
  }
}

// user has pressed esc to go back one level
static void menu_back(void) {
  // stop doing the scroll timer
  menu_timer_enable(false);

  // are we in the root menu?
  if(menu_state->menu == cfg->menu)
    osd_enable(OSD_INVISIBLE);
  else {
    // are we in fileselector?
    if(menu_state->type == CONFIG_MENU_ENTRY_FILESELECTOR) {
      // search for ".." in current dir
      sdc_dir_entry_t *entry = NULL;
      for(int i=0;i<menu_state->dir->len;i++)
	if(!strcmp(menu_state->dir->files[i].name, ".."))
	  entry = &(menu_state->dir->files[i]);
      
      // if there was one, go up. Else quit the file selector
      if(entry) menu_fileselector_select(entry);
      else      menu_pop();
    } else
      menu_pop();
  }
}

// user has selected a menu entry
static void menu_select(void) {
  // if the title was selected, then goto parent form
  if(menu_state->selected == 0) {
    menu_pop();
    return;
  }

  // in fileselector
  if(menu_state->type == CONFIG_MENU_ENTRY_FILESELECTOR) {
    menu_fileselector_select(&menu_state->dir->files[menu_state->selected-1]);
    return;
  }
  
  config_menu_entry_t *entry = menu_state->menu->entries + menu_state->selected - 1;
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
    // user has choosen a selection list
    int value = menu_variable_get(entry->list->id) + 1;
    int max_value = menu_get_list_length(entry);
    if(value > max_value) value = 0;    
    menu_variable_set(entry->list->id, value);

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

    menu_draw();
  
    xTimerChangePeriod( menu_key_repeat_timer, pdMS_TO_TICKS(100), 0);
    xTimerStart( menu_key_repeat_timer, 0 );
  }
}
  
void menu_stop_repeat(void) {
  xTimerStop(menu_key_repeat_timer, 0);
  menu_key_last_event = -1;
}

void menu_do(int event) {
  // -1 is a timer event used to scroll the current file name if it's to long
  // for the OSD
  if(event < 0) {
    if(cfg) {
      if(menu_state->type == CONFIG_MENU_ENTRY_FILESELECTOR)
	menu_fs_scroll_entry();
    }
      
    return;
  }
  
  menu_debugf("do %d", event);
  
  if(event)  {
    if(event == MENU_EVENT_SHOW)
      osd_enable(OSD_VISIBLE);
      
    if(event == MENU_EVENT_HIDE) {
      menu_timer_enable(false);
      osd_enable(OSD_INVISIBLE);
      return;  // return now to prevent OSD from being drawn, again
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
  menu_draw();
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

static void menu_task(__attribute__((unused)) void *parms) {
  menu_debugf("task running");

  // wait for user events
  while(1) {
    // receive events from usb    
    long cmd;
    xQueueReceive(menu_queue, &cmd, 0xffffffffUL);
    menu_debugf("command %ld", cmd);
    menu_do(cmd);
  }
}

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

  menu_goto(cfg->menu);    

  // ready to run core
  sys_run_action_by_name("ready");

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
  xTaskCreate(menu_task, (char *)"menu_task", 4096, NULL, configMAX_PRIORITIES-3, NULL);
}
  
void menu_notify(unsigned long msg) {
  xQueueSendToBackFromISR(menu_queue, &msg,  ( TickType_t ) 0);
}

