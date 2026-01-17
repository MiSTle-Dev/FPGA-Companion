/*
  config.c - parse configuration as requested by the FPGA or read from SD card
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"
#include "debug.h"
#include "xml.h"

#ifdef ESP_PLATFORM
#include <freertos/FreeRTOS.h>
#else
#include <FreeRTOS.h>
#endif

#define CONFIG_XML_ELEMENT_ERROR         -1
#define CONFIG_XML_ELEMENT_ROOT           0
#define CONFIG_XML_ELEMENT_CONFIG         1
#define CONFIG_XML_ELEMENT_ACTIONS        2
#define CONFIG_XML_ELEMENT_ACTION         3
#define CONFIG_XML_ELEMENT_COMMAND_LOAD   4
#define CONFIG_XML_ELEMENT_COMMAND_SET    5
#define CONFIG_XML_ELEMENT_COMMAND_SAVE   6
#define CONFIG_XML_ELEMENT_COMMAND_DELAY  7
#define CONFIG_XML_ELEMENT_COMMAND_HIDE   8
#define CONFIG_XML_ELEMENT_COMMAND_LINK   9
#define CONFIG_XML_ELEMENT_MENU          10
#define CONFIG_XML_ELEMENT_FILSELECTOR   11
#define CONFIG_XML_ELEMENT_LIST          12
#define CONFIG_XML_ELEMENT_LISTENTRY     13
#define CONFIG_XML_ELEMENT_BUTTON        14
#define CONFIG_XML_ELEMENT_IMAGE         15
#define CONFIG_XML_ELEMENT_TOGGLE        16

static int config_element;
static int config_depth;

config_t *cfg = NULL;

void config_init(void) {
  xml_init();  // reset xml parser

  // init configuration structure
  config_element = CONFIG_XML_ELEMENT_ROOT;
  config_depth = 0;
  cfg = pvPortMalloc(sizeof(config_t));
  cfg->name = NULL;  
  cfg->version = -1;  
  cfg->menu = NULL;  
  cfg->actions = NULL;
}

// a FreeRTOS'd version of strdup
static inline char *StrDup(const char *s) {
  char *res = pvPortMalloc(strlen(s)+1);
  strcpy(res, s);
  return res;
}

/* ============================================================================= */
/* ================================== root ===================================== */
/* ============================================================================= */
 
static int config_xml_root_element(char *name) {
  // expecting config element
  if(strcasecmp(name, "config") == 0) {
    config_element = CONFIG_XML_ELEMENT_CONFIG;
    return 0;
  } else
    debugf("WARNING: Unexpected element %s in state %d", name, config_element);

  return -1;
}

static config_menu_t *config_xml_new_menu(config_menu_t *parent);
static void config_xml_new_fileselector(config_menu_t *menu);
static void config_xml_new_list(config_menu_t *menu);
static void config_xml_new_button(config_menu_t *menu);
static void config_xml_new_image(config_menu_t *menu);
static void config_xml_new_toggle(config_menu_t *menu);

/* ============================================================================= */
/* ================================ config ===================================== */
/* ============================================================================= */
 
static int config_xml_config_element(char *name) {
  // expecting actions element
  if(strcasecmp(name, "actions") == 0) {
    config_element = CONFIG_XML_ELEMENT_ACTIONS;
    return 0;
  } else if(strcasecmp(name, "menu") == 0) {
    // root level menu
    cfg->menu = config_xml_new_menu(NULL);      
    config_element = CONFIG_XML_ELEMENT_MENU;      
    return 0;
  } else
    debugf("WARNING: Unexpected config element %s in state %d", name, config_element);

  return -1;
}

static void config_xml_config_attribute(char *name, char *value) {    
  if(strcasecmp(name, "name") == 0 && !cfg->name)
    cfg->name = StrDup(value);
  else if(strcasecmp(name, "version") == 0)
    cfg->version = atoi(value);
  else
    debugf("WARNING: Unused config attribute '%s'", name);
}


/* ========================================================================= */
/* ===========================  actions ==================================== */
/* ========================================================================= */

static void config_xml_new_action(config_t *cfg) {
  config_action_t *action = pvPortMalloc(sizeof(config_action_t));
  action->name = NULL;
  action->commands = NULL;
  action->next = NULL;
  
  // attach new action to chain
  if(!cfg->actions) cfg->actions = action;
  else {
    config_action_t *a = cfg->actions;
    while(a->next) a = a->next;
    a->next = action;
  }
}

static config_action_t *config_xml_get_last_action(config_t *cfg) {
  config_action_t *action = cfg->actions;
  if(!action) return NULL;
  while(action->next) action = action->next;  
  return action;
}

config_action_t *config_get_action(const char *str) {
  config_action_t *action = cfg->actions;

  while(action) {
    if(strcmp(str, action->name) == 0)
      return action;
    
    action = action->next;
  }
  return NULL;  
}

static void config_xml_action_attribute(char *name, char *value) {
  // get current action
  config_action_t *action = config_xml_get_last_action(cfg);
  if(action && strcasecmp(name, "name") == 0 && !action->name)
    action->name = StrDup(value);
  
  else
    debugf("WARNING: Unused action attribute '%s'", name);    
}


static void config_xml_new_action_command(config_action_t *action, int code) {
  // allocate memory for a new command
  config_action_command_t *command = pvPortMalloc(sizeof(config_action_command_t));
  memset(command, 0, sizeof(config_action_command_t));
  command->code = code;
  command->next = NULL;

  // append new command to chain of commands
  if(!action->commands) action->commands = command;
  else {
    config_action_command_t *c = action->commands;
    while(c->next) c = c->next;
    c->next = command;
  }
}

static config_action_command_t *config_xml_get_last_action_command(config_action_t *action) {
  if(!action) return NULL;
  if(!action->commands) return NULL;

  config_action_command_t *command = action->commands;
  while(command->next) command = command->next;

  return command;
}

static void config_xml_command_attribute(int config_element, char *name, char *value) {
  config_action_command_t *command =  config_xml_get_last_action_command(config_xml_get_last_action(cfg));
  if(command) {
    if(config_element == CONFIG_XML_ELEMENT_COMMAND_LOAD && strcasecmp(name, "file") == 0 && !command->filename) {
      command->filename = StrDup(value);
    } else if(config_element == CONFIG_XML_ELEMENT_COMMAND_SET && strcasecmp(name, "id") == 0)
      command->set.id = value[0];
    else if(config_element == CONFIG_XML_ELEMENT_COMMAND_SET && strcasecmp(name, "value") == 0)
      command->set.value = atoi(value);
    else if(config_element == CONFIG_XML_ELEMENT_COMMAND_SAVE && strcasecmp(name, "file") == 0 && !command->filename)
      command->filename = StrDup(value);
    else if(config_element == CONFIG_XML_ELEMENT_COMMAND_DELAY && strcasecmp(name, "ms") == 0)
      command->delay.ms = atoi(value);
    else if(config_element == CONFIG_XML_ELEMENT_COMMAND_LINK && strcasecmp(name, "action") == 0 && !command->action)
      command->action = config_get_action(value);
	
    else
      debugf("WARNING: Unused action/command/<...> attribute '%s'", name);    
  }
}

static int config_xml_actions_element(char *name) {    
  // expecting action element
  if(strcasecmp(name, "action") == 0) {      
    // append a new action entry      
    config_xml_new_action(cfg);
    config_element = CONFIG_XML_ELEMENT_ACTION;
    return 0;
  } else
    debugf("WARNING: Unexpected actions element %s in state %d", name, config_element);

  return -1;
}
  
static int config_xml_action_element(char *name) {
  // get current action
  config_action_t *action = config_xml_get_last_action(cfg);
  if(action) {
    // add a new command entry
    if(strcasecmp(name, "load") == 0) {
      config_xml_new_action_command(action, CONFIG_ACTION_COMMAND_LOAD);
      config_element = CONFIG_XML_ELEMENT_COMMAND_LOAD;
      return 0;
    } else if(strcasecmp(name, "set") == 0) {
      config_xml_new_action_command(action, CONFIG_ACTION_COMMAND_SET);
      config_element = CONFIG_XML_ELEMENT_COMMAND_SET;
      return 0;
    } else if(strcasecmp(name, "save") == 0) {
      config_xml_new_action_command(action, CONFIG_ACTION_COMMAND_SAVE);
      config_element = CONFIG_XML_ELEMENT_COMMAND_SAVE;
      return 0;
    } else if(strcasecmp(name, "delay") == 0) {
      config_xml_new_action_command(action, CONFIG_ACTION_COMMAND_DELAY);
      config_element = CONFIG_XML_ELEMENT_COMMAND_DELAY;
      return 0;
    } else if(strcasecmp(name, "hide") == 0) {
      config_xml_new_action_command(action, CONFIG_ACTION_COMMAND_HIDE);
      config_element = CONFIG_XML_ELEMENT_COMMAND_HIDE;
      return 0;
    } else if(strcasecmp(name, "link") == 0) {
      config_xml_new_action_command(action, CONFIG_ACTION_COMMAND_LINK);
      config_element = CONFIG_XML_ELEMENT_COMMAND_LINK;
      return 0;
    } else
      debugf("WARNING: Unexpected command element %s in state %d", name, config_element);

  }
  return -1;
}

static void config_dump_action(config_action_t *act) {
  debugf("Action, name=\"%s\"", act->name);
  config_action_command_t *command = act->commands;

  while(command) {
    switch(command->code) {
    case CONFIG_ACTION_COMMAND_LOAD:
      debugf("  Load %s", command->filename);
      break;
    case CONFIG_ACTION_COMMAND_SAVE:
      debugf("  Save %s", command->filename);
      break;
    case CONFIG_ACTION_COMMAND_SET:
      debugf("  Set %c=%u", command->set.id, command->set.value);
      break;
    case CONFIG_ACTION_COMMAND_DELAY:
      debugf("  Delay %ums", command->delay.ms);
      break;
    case CONFIG_ACTION_COMMAND_HIDE:
      debugf("  Hide OSD");
      break;
    }
    command = command->next;
  }
}

/* ========================================================================= */
/* ============================== menu ===================================== */
/* ========================================================================= */
 
static config_menu_entry_t *config_xml_new_menu_entry(config_menu_t *menu) {
  // Allocate space for one more entry. There need to be two more enries than
  // used by now. One for the new entry and one for the end marker.
  config_menu_entry_t *entry = pvPortMalloc(sizeof(config_menu_entry_t));
  entry->type = CONFIG_MENU_ENTRY_UNKNOWN;
  entry->next = NULL;

  // append new menu entry to chain of menu entries
  if(!menu->entries) menu->entries = entry;
  else {
    config_menu_entry_t *e = menu->entries;
    while(e->next) e = e->next;
    e->next = entry;
  }

  return entry;
}

static config_menu_t *config_xml_new_menu(config_menu_t *parent) {
  config_menu_t *menu = pvPortMalloc(sizeof(config_menu_t));
  menu->label = NULL;
  menu->entries = NULL;
  
  if(parent) {
    config_menu_entry_t *me = config_xml_new_menu_entry(parent);
    me->type = CONFIG_MENU_ENTRY_MENU;
    me->menu = menu;
  }
  
  return menu;
}

static config_menu_t *config_xml_get_menu(config_menu_t *menu, int depth) {
  // walk over menu tree to return last menu entry
  config_menu_t *last = menu;
  config_menu_entry_t *me = menu->entries;
  while(me) {
    if(me->type == CONFIG_MENU_ENTRY_MENU && depth)
      last = config_xml_get_menu(me->menu, depth-1);
    me = me->next;
  }    
  return last;
}

static config_menu_entry_t *config_xml_get_last_menu_entry(config_menu_t *menu, int depth) {
  menu = config_xml_get_menu(menu, depth);
  config_menu_entry_t *me = menu->entries;
  while(me->next) me = me->next;
  return me;
}

const char *config_menuentry_get_type_str(config_menu_entry_t *entry) {
  const char *names[] = { "unknown", "menu", "fileselector", "list", "button", "image", "toggle" };

  if(entry->type == CONFIG_MENU_ENTRY_MENU)         return names[1];
  if(entry->type == CONFIG_MENU_ENTRY_FILESELECTOR) return names[2];
  if(entry->type == CONFIG_MENU_ENTRY_LIST)         return names[3];
  if(entry->type == CONFIG_MENU_ENTRY_BUTTON)       return names[4];
  if(entry->type == CONFIG_MENU_ENTRY_IMAGE)        return names[5];
  if(entry->type == CONFIG_MENU_ENTRY_TOGGLE)       return names[6];
  return names[0];  
}

static int config_xml_menu_element(char *name) {
  config_menu_t *menu = config_xml_get_menu(cfg->menu, config_depth-3);
  if(strcasecmp(name, "fileselector") == 0) {
    config_xml_new_fileselector(menu);
    config_element = CONFIG_XML_ELEMENT_FILSELECTOR;
    return 0;
  } else if(strcasecmp(name, "menu") == 0) {
    config_xml_new_menu(menu);      	
    config_element = CONFIG_XML_ELEMENT_MENU;
    return 0;
  } else if(strcasecmp(name, "list") == 0) {
    config_xml_new_list(menu);      	
    config_element = CONFIG_XML_ELEMENT_LIST;
    return 0;
  } else if(strcasecmp(name, "button") == 0) {
    config_xml_new_button(menu);      	
    config_element = CONFIG_XML_ELEMENT_BUTTON;
    return 0;
  } else if(strcasecmp(name, "image") == 0) {
    config_xml_new_image(menu);
    config_element = CONFIG_XML_ELEMENT_IMAGE;
    return 0;
  } else if(strcasecmp(name, "toggle") == 0) {
    config_xml_new_toggle(menu);
    config_element = CONFIG_XML_ELEMENT_TOGGLE;
    return 0;
  } else
    debugf("WARNING: Unexpected menu element %s in state %d", name, config_element);
    
  return -1;
}
    
static void config_xml_menu_attribute(char *name, char *value) {
  config_menu_t *menu = config_xml_get_menu(cfg->menu, config_depth-2);  
  if(menu && strcasecmp(name, "label") == 0 && !menu->label)
    menu->label = StrDup(value);
  
  else
    debugf("WARNING: Unused menu attribute '%s'", name);    
}

static void config_dump_fileselector(config_fsel_t *fs);
static void config_dump_button(config_button_t *btn);
static void config_dump_image(config_image_t *img);
static void config_dump_toggle(config_toggle_t *btn);
static void config_dump_list(config_list_t *ls);

static void config_dump_menu(config_menu_t *mnu) {
  debugf("Menu, label=\"%s\"", mnu->label);

  config_menu_entry_t *me = mnu->entries;
  while(me) {
    switch(me->type) {
    case CONFIG_MENU_ENTRY_MENU:
      config_dump_menu(me->menu);
      break;
    case CONFIG_MENU_ENTRY_FILESELECTOR:
      config_dump_fileselector(me->fsel);
      break;
    case CONFIG_MENU_ENTRY_LIST:
      config_dump_list(me->list);
      break;
    case CONFIG_MENU_ENTRY_BUTTON:
      config_dump_button(me->button);
      break;
    case CONFIG_MENU_ENTRY_IMAGE:
      config_dump_image(me->image);
      break;
    case CONFIG_MENU_ENTRY_TOGGLE:
      config_dump_toggle(me->toggle);
      break;
    }
    me = me->next;
  }
}

/* ============================================================================= */
/* ============================= fileselector ================================== */
/* ============================================================================= */
 
static void config_xml_new_fileselector(config_menu_t *menu) {
  config_fsel_t *fsel = pvPortMalloc(sizeof(config_fsel_t));
  fsel->index = -1;
  fsel->label = NULL;
  fsel->ext = NULL;
  fsel->def = NULL;
  fsel->action = NULL;

  config_menu_entry_t *me = config_xml_new_menu_entry(menu);
  me->type = CONFIG_MENU_ENTRY_FILESELECTOR;
  me->fsel = fsel;
}

static char **config_parse_strlist(const char *str, char sep) {
  // determine number of strings inside
  int i=1;
  for(const char *s=str;(s=strchr(s,sep));i++,s++);
  
  // allocate array of pointers incl terminating 0 pointer
  char **ptr = pvPortMalloc((i+1)*sizeof(char*));

  i=0;
  char *s;
  while((s = strchr(str, sep))) {
    ptr[i] = pvPortMalloc(s-str+1);
    strncpy(ptr[i], str, s-str+1);
    ptr[i][s-str] = '\0';
    str = s+1;
    i++;
  }

  // copy last string and append a null pointer to mark the end of the list
  ptr[i] = StrDup(str);
  ptr[i+1] = NULL;

  return ptr;
}

static void config_xml_fsel_attribute(char *name, char *value) {
  config_menu_entry_t *me = config_xml_get_last_menu_entry(cfg->menu, config_depth-2);
  if(me && me->type == CONFIG_MENU_ENTRY_FILESELECTOR) {    
    if(me->fsel && strcasecmp(name, "label") == 0 && !me->fsel->label)
      me->fsel->label = StrDup(value);	  
    else if(me->fsel && strcasecmp(name, "ext") == 0 && !me->fsel->ext)
      me->fsel->ext = config_parse_strlist(value, ';');
    else if(me->fsel && strcasecmp(name, "index") == 0)
      me->fsel->index = atoi(value);
    else if(me->fsel && strcasecmp(name, "default") == 0)
      me->fsel->def = StrDup(value);
    else if(strcasecmp(name, "action") == 0)
      me->fsel->action = config_get_action(value);
    
    else
      debugf("WARNING: Unused file selector attribute '%s'", name);
  }
}

static void config_dump_fileselector(config_fsel_t *fs) {
  debugf("Fileselector, index=%d, label=\"%s\" ext=[%s], default=\"%s\"",
	 fs->index, fs->label, fs->ext[0], fs->def?fs->def:"<none>");
  for(int i=1;fs->ext[i];i++) debugf("  further ext: \"%s\"", fs->ext[i]);
  if(fs->action) config_dump_action(fs->action);
}
  
/* ============================================================================= */
/* ================================== list ===================================== */
/* ============================================================================= */
 
static void config_xml_new_list(config_menu_t *menu) {
  config_list_t *list = pvPortMalloc(sizeof(config_list_t));
  list->id = -1;
  list->def = -1;
  list->label = NULL;
  list->action = NULL;
  list->listentries = NULL;

  config_menu_entry_t *me = config_xml_new_menu_entry(menu);
  me->type = CONFIG_MENU_ENTRY_LIST;
  me->list = list;
}

static void config_xml_new_listentry(config_list_t *list) {
  config_listentry_t *listentry = pvPortMalloc(sizeof(config_listentry_t));
  listentry->value = 0;
  listentry->label = NULL;
  listentry->next = NULL;

  // append new menu entry to chain of menu entries
  if(!list->listentries) list->listentries = listentry;
  else {
    config_listentry_t *e = list->listentries;
    while(e->next) e = e->next;
    e->next = listentry;
  }
}

static int config_xml_list_element(char *name) {
  config_menu_entry_t *me = config_xml_get_last_menu_entry(cfg->menu, config_depth-4);
  if(me && me->type == CONFIG_MENU_ENTRY_LIST) {    
    if(strcasecmp(name, "listentry") == 0) {
      config_xml_new_listentry(me->list);
      config_element = CONFIG_XML_ELEMENT_LISTENTRY;
      return 0;
    } else
      debugf("WARNING: Unexpected list element %s in state %d", name, config_element);
  }

  return -1;
}

static void config_xml_list_attribute(char *name, char *value) {
  config_menu_entry_t *me = config_xml_get_last_menu_entry(cfg->menu, config_depth-3);
  if(me && me->type == CONFIG_MENU_ENTRY_LIST) {    
    if(me->list && strcasecmp(name, "label") == 0 && !me->list->label)
      me->list->label = StrDup(value);      
    else if(me->list && strcasecmp(name, "id") == 0)
      me->list->id = value[0];
    else if(me->list && strcasecmp(name, "default") == 0)
      me->list->def = atoi(value);
    else if(me->list && strcasecmp(name, "action") == 0)
      me->list->action = config_get_action(value);
    
    else
      debugf("WARNING: Unused list attribute '%s'", name);
  }
}

static void config_xml_listentry_attribute(char *name, char *value) {
  // get corresponding list
  config_menu_entry_t *me = config_xml_get_last_menu_entry(cfg->menu, config_depth-4);
  if(me && me->list && me->type == CONFIG_MENU_ENTRY_LIST) {    
    // get last listentry
    config_listentry_t *le = me->list->listentries;
    while(le->next) le = le->next;

    if(strcasecmp(name, "label") == 0 && !le->label)
      le->label = StrDup(value);      
    else if(strcasecmp(name, "value") == 0)
      le->value = atoi(value);      
    
    else
      debugf("WARNING: Unused listentry attribute '%s'", name);
  }
}

static void config_dump_list(config_list_t *ls) {
  debugf("List, id='%c', label=\"%s\", default=\"%d\"", ls->id, ls->label, ls->def);
  config_listentry_t *le = ls->listentries;
  while(le) {
    debugf("  Listentry, label=\"%s\", value=\"%d\"", le->label, le->value);
    le = le->next;
  }
  if(ls->action) config_dump_action(ls->action);
}
  
/* ============================================================================= */
/* ================================= button ==================================== */
/* ============================================================================= */

static void config_xml_new_button(config_menu_t *menu) {
  config_button_t *button = pvPortMalloc(sizeof(config_button_t));
  button->label = NULL;
  button->action = NULL;

  config_menu_entry_t *me = config_xml_new_menu_entry(menu);
  me->type = CONFIG_MENU_ENTRY_BUTTON;
  me->button = button;
}
 
static void config_xml_button_attribute(char *name, char *value) {
  // get corresponding button
  config_menu_entry_t *me = config_xml_get_last_menu_entry(cfg->menu, config_depth-3);
  if(me && me->type == CONFIG_MENU_ENTRY_BUTTON) {    
    if(me->button && strcasecmp(name, "label") == 0 && !me->button->label)
      me->button->label = StrDup(value);      
    else if(me->button && strcasecmp(name, "action") == 0)
      me->button->action = config_get_action(value);
    
    else
      debugf("WARNING: Unused button attribute '%s'", name);
  }
}
    
static void config_dump_button(config_button_t *btn) {
  debugf("Button, label=\"%s\"", btn->label);
  if(btn->action) config_dump_action(btn->action);
}

/* ============================================================================= */
/* ================================= image ===================================== */
/* ============================================================================= */

static void config_xml_new_image(config_menu_t *menu) {
  config_image_t *image = pvPortMalloc(sizeof(config_image_t));
  image->index = -1;
  image->label = NULL;
  image->none_str = NULL;
  image->none_icn = NULL;
  image->def = NULL;
  image->action = NULL;
  image->ext = NULL;

  config_menu_entry_t *me = config_xml_new_menu_entry(menu);
  me->type = CONFIG_MENU_ENTRY_IMAGE;
  me->image = image;
}
 
static void config_xml_image_attribute(char *name, char *value) {
  // get corresponding image
  config_menu_entry_t *me = config_xml_get_last_menu_entry(cfg->menu, config_depth-3);
  if(me && me->type == CONFIG_MENU_ENTRY_IMAGE) {    
    if(me->image && strcasecmp(name, "label") == 0 && !me->image->label)
      me->image->label = StrDup(value);      
    else if(me->image && strcasecmp(name, "ext") == 0 && !me->image->ext)
      me->image->ext = config_parse_strlist(value, ';');
    else if(me->image && strcasecmp(name, "index") == 0)
      me->image->index = atoi(value);
    else if(me->image && strcasecmp(name, "default") == 0)
      me->image->def = StrDup(value);
    else if(me->image && strcasecmp(name, "none_str") == 0 && !me->image->none_str)
      me->image->none_str = StrDup(value);      
    else if(me->image && strcasecmp(name, "none_icn") == 0 && !me->image->none_icn) {
      if(strlen(value) >= 16) {
	// convert 16 bytes hex string into 8 byte bitmap
	me->image->none_icn = pvPortMalloc(8);
	for(int i=0;i<16;i++) {
	  int nibble =
	    (value[i] >= '0' && value[i] <= '9')?(value[i]-'0'):
	    (value[i] >= 'a' && value[i] <= 'f')?(value[i]-'a'+10):
	    (value[i] >= 'A' && value[i] <= 'F')?(value[i]-'A'+10):
	    0;
	  
	  me->image->none_icn[i/2] = (i&1)?(me->image->none_icn[i/2] | nibble):(nibble<<4);
	}
      }
    } else if(me->image && strcasecmp(name, "action") == 0)
      me->image->action = config_get_action(value);

    else
      debugf("WARNING: Unused image attribute '%s'", name);
  }
}
    
static void config_dump_image(config_image_t *img) {
  debugf("Image, index=%d, label=\"%s\", none_str=\"%s\", none_icn=%p, ext=[%s], default=\"%s\"",
	 img->index, img->label, img->none_str?img->none_str:"<unset>", img->none_icn,
	 img->ext[0], img->def?img->def:"<none>");
  for(int i=1;img->ext[i];i++) debugf("  further ext: \"%s\"", img->ext[i]);
  if(img->action) config_dump_action(img->action);
}

/* ============================================================================= */
/* ================================= toggle ==================================== */
/* ============================================================================= */

static void config_xml_new_toggle(config_menu_t *menu) {
  config_toggle_t *toggle = pvPortMalloc(sizeof(config_toggle_t));
  toggle->label = NULL;
  toggle->action = NULL;
  toggle->id = 0;
  toggle->def = 0;

  config_menu_entry_t *me = config_xml_new_menu_entry(menu);
  me->type = CONFIG_MENU_ENTRY_TOGGLE;
  me->toggle = toggle;
}
 
static void config_xml_toggle_attribute(char *name, char *value) {
  // get corresponding toggle
  config_menu_entry_t *me = config_xml_get_last_menu_entry(cfg->menu, config_depth-3);
  if(me && me->type == CONFIG_MENU_ENTRY_TOGGLE) {    
    if(me->toggle && strcasecmp(name, "label") == 0 && !me->toggle->label)
      me->toggle->label = StrDup(value);      
    else if(me->toggle && strcasecmp(name, "id") == 0)
      me->toggle->id = value[0];
    else if(me->toggle && strcasecmp(name, "default") == 0)
      me->toggle->def = atoi(value);
    else if(me->toggle && strcasecmp(name, "action") == 0)
      me->toggle->action = config_get_action(value);

    else
      debugf("WARNING: Unused toggle attribute '%s'", name);
  }
}
    
static void config_dump_toggle(config_toggle_t *toggle) {
  debugf("Toggle, id='%c', label=\"%s\", default=\"%d\"", toggle->id, toggle->label, toggle->def);
  if(toggle->action) config_dump_action(toggle->action);
}

void config_dump(void) {
  debugf("========================== Config ==========================");

  debugf("Name: %s", cfg->name);
  debugf("Version: %d.%d", cfg->version/100, cfg->version%100);

  // check if an init or ready action exists
  config_action_t *action = config_get_action("init");
  if(action) { debugf("On init:"); config_dump_action(action); }
  action = config_get_action("ready");
  if(action) { debugf("On ready:"); config_dump_action(action); }
  
  if(cfg->menu)
    config_dump_menu(cfg->menu);
}
  
/* ============================================================================= */
/* =========================== xml parser callbacks ============================ */
/* ============================================================================= */

int xml_element_start_cb(char *name) {
  int retval = -1;
  
  // debugf("%d Element start: %s", config_element, name);
  config_depth++;

  switch(config_element) {
  case CONFIG_XML_ELEMENT_ROOT:  // initial state
    retval = config_xml_root_element(name);
    break;
    
  case CONFIG_XML_ELEMENT_CONFIG: // inside config
    retval = config_xml_config_element(name);
    break;

  case CONFIG_XML_ELEMENT_ACTIONS: // config/actions
    retval = config_xml_actions_element(name);
    break;
    
  case CONFIG_XML_ELEMENT_ACTION: // config/actions/action/<...>
    retval = config_xml_action_element(name);
    break;
    
  case CONFIG_XML_ELEMENT_MENU: // menu
    retval = config_xml_menu_element(name);
    break;

  case CONFIG_XML_ELEMENT_LIST:  // list    
    retval = config_xml_list_element(name);
    break;
    
  default:
    debugf("WARNING: Unexpected element state");
  }

  // parsing failed?
  if(retval != 0) {
    config_depth--;
    return -1;
  }
  
  return 0;
}

void xml_element_end_cb(void) {
  // debugf("%d Element end", config_element);
  
  switch(config_element) {
  case CONFIG_XML_ELEMENT_ROOT:  // initial state
    debugf("WARNING: Unexpected element close in state %d", config_element);
    config_element = CONFIG_XML_ELEMENT_ERROR;
    break;

  case CONFIG_XML_ELEMENT_CONFIG:  // config
    config_element = CONFIG_XML_ELEMENT_ROOT;
    break;
    
  case CONFIG_XML_ELEMENT_ACTIONS:  // config/action
    config_element = CONFIG_XML_ELEMENT_CONFIG; // -> config
    break;
    
  case CONFIG_XML_ELEMENT_ACTION:  // config/action/command
    config_element = CONFIG_XML_ELEMENT_ACTIONS;
    break;
    
  case CONFIG_XML_ELEMENT_COMMAND_LOAD:
  case CONFIG_XML_ELEMENT_COMMAND_SET:
  case CONFIG_XML_ELEMENT_COMMAND_SAVE:
  case CONFIG_XML_ELEMENT_COMMAND_DELAY:
  case CONFIG_XML_ELEMENT_COMMAND_HIDE:
  case CONFIG_XML_ELEMENT_COMMAND_LINK:
    config_element = CONFIG_XML_ELEMENT_ACTION;
    break;
    
  case CONFIG_XML_ELEMENT_MENU:
    // returning from a menu usually returns to the parent menu. Except
    // in depth 2, then it returns to the config
    if(config_depth == 2) config_element = CONFIG_XML_ELEMENT_CONFIG;
    break;
    
  case CONFIG_XML_ELEMENT_FILSELECTOR:
  case CONFIG_XML_ELEMENT_LIST:
  case CONFIG_XML_ELEMENT_BUTTON:
  case CONFIG_XML_ELEMENT_IMAGE:
  case CONFIG_XML_ELEMENT_TOGGLE:
    config_element = CONFIG_XML_ELEMENT_MENU;
    break;
    
  case CONFIG_XML_ELEMENT_LISTENTRY:
    config_element = CONFIG_XML_ELEMENT_LIST;
    break;
  }
  config_depth--;
}

void xml_attribute_cb(char *name, char *value) {
  // debugf("%d Attribute: %s = '%s'", config_element, name, value);

  switch(config_element) {
  case CONFIG_XML_ELEMENT_CONFIG:
    config_xml_config_attribute(name, value);
    break;
    
  case CONFIG_XML_ELEMENT_ACTION:
    config_xml_action_attribute(name, value);
    break;
    
  case CONFIG_XML_ELEMENT_COMMAND_LOAD:
  case CONFIG_XML_ELEMENT_COMMAND_SET:
  case CONFIG_XML_ELEMENT_COMMAND_SAVE:
  case CONFIG_XML_ELEMENT_COMMAND_DELAY:
  case CONFIG_XML_ELEMENT_COMMAND_HIDE:
  case CONFIG_XML_ELEMENT_COMMAND_LINK:
    config_xml_command_attribute(config_element, name, value);
    break;

  case CONFIG_XML_ELEMENT_MENU:
    config_xml_menu_attribute(name, value);
    break;
    
  case CONFIG_XML_ELEMENT_FILSELECTOR:
    config_xml_fsel_attribute(name, value);
    break;
    
  case CONFIG_XML_ELEMENT_LIST:
    config_xml_list_attribute(name, value);
    break;
    
  case CONFIG_XML_ELEMENT_LISTENTRY:
    config_xml_listentry_attribute(name, value);
    break;
    
  case CONFIG_XML_ELEMENT_BUTTON:
    config_xml_button_attribute(name, value);
    break;
    
  case CONFIG_XML_ELEMENT_IMAGE:
    config_xml_image_attribute(name, value);
    break;
    
  case CONFIG_XML_ELEMENT_TOGGLE:
    config_xml_toggle_attribute(name, value);
    break;
  }
}
