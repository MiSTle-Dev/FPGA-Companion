/*
  config.c - parse configuration as requested by the FPGA or read from SD card
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"
#include "debug.h"
#include "xml.h"

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
  cfg = malloc(sizeof(config_t));
  cfg->name = NULL;  
  cfg->version = -1;  
  cfg->menu = NULL;  

  // initialize empty actions list
  cfg->actions = malloc(sizeof(config_action_t *));
  cfg->actions[0] = NULL;
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
    cfg->name = strdup(value);
  else if(strcasecmp(name, "version") == 0)
    cfg->version = atoi(value);
  else
    debugf("WARNING: Unused config attribute '%s'", name);
}


/* ========================================================================= */
/* ===========================  actions ==================================== */
/* ========================================================================= */

static void config_xml_new_action(config_t *cfg) {
  int a=0; for(;cfg->actions[a];a++);
  cfg->actions = reallocarray(cfg->actions, a+2, sizeof(config_action_t*));

  cfg->actions[a] = malloc(sizeof(config_action_t));
  cfg->actions[a]->name = NULL;
  cfg->actions[a]->commands = malloc(sizeof(config_action_command_t));
  cfg->actions[a]->commands[0].code = CONFIG_ACTION_COMMAND_IDLE;

  cfg->actions[a+1] = NULL;
}

static config_action_t *config_xml_get_last_action(config_t *cfg) {
  int a=0; for(;cfg->actions[a];a++);
  return cfg->actions[a-1];
}

config_action_t *config_get_action(const char *str) {
  for(int i=0;cfg->actions[i];i++)
    if(strcmp(str, cfg->actions[i]->name) == 0)
      return cfg->actions[i];

  return NULL;  
}

static void config_xml_action_attribute(char *name, char *value) {
  // get current action
  config_action_t *action = config_xml_get_last_action(cfg);
  if(action && strcasecmp(name, "name") == 0 && !action->name)
    action->name = strdup(value);
  
  else
    debugf("WARNING: Unused action attribute '%s'", name);    
}


static void config_xml_new_action_command(config_action_t *action, int code) {
  int c=0; for(;action->commands[c].code!=CONFIG_ACTION_COMMAND_IDLE;c++);

  action->commands = reallocarray(action->commands, c+2, sizeof(config_action_command_t));
  memset(&action->commands[c], 0, sizeof(config_action_command_t));
  action->commands[c].code = code;
  action->commands[c+1].code = CONFIG_ACTION_COMMAND_IDLE;
}

static config_action_command_t *config_xml_get_last_action_command(config_action_t *action) {
  if(!action) return NULL;
  
  int c; for(c=0;action->commands[c].code != CONFIG_ACTION_COMMAND_IDLE;c++);
  return &action->commands[c-1];
}

static void config_xml_command_attribute(int config_element, char *name, char *value) {
  config_action_command_t *command =  config_xml_get_last_action_command(config_xml_get_last_action(cfg));
  if(command) {
    if(config_element == CONFIG_XML_ELEMENT_COMMAND_LOAD && strcasecmp(name, "file") == 0 && !command->filename) {
      command->filename = strdup(value);
    } else if(config_element == CONFIG_XML_ELEMENT_COMMAND_SET && strcasecmp(name, "id") == 0)
      command->set.id = value[0];
    else if(config_element == CONFIG_XML_ELEMENT_COMMAND_SET && strcasecmp(name, "value") == 0)
      command->set.value = atoi(value);
    else if(config_element == CONFIG_XML_ELEMENT_COMMAND_SAVE && strcasecmp(name, "file") == 0 && !command->filename)
      command->filename = strdup(value);
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
  for(int i=0;act->commands[i].code != CONFIG_ACTION_COMMAND_IDLE;i++) {  
    switch(act->commands[i].code) {
    case CONFIG_ACTION_COMMAND_LOAD:
      debugf("  Load %s", act->commands[i].filename);
      break;
    case CONFIG_ACTION_COMMAND_SAVE:
      debugf("  Save %s", act->commands[i].filename);
      break;
    case CONFIG_ACTION_COMMAND_SET:
      debugf("  Set %c=%u", act->commands[i].set.id, act->commands[i].set.value);
      break;
    case CONFIG_ACTION_COMMAND_DELAY:
      debugf("  Delay %ums", act->commands[i].delay.ms);
      break;
    case CONFIG_ACTION_COMMAND_HIDE:
      debugf("  Hide OSD");
      break;
    }
  }
}

/* ========================================================================= */
/* ============================== menu ===================================== */
/* ========================================================================= */
 
static config_menu_entry_t *config_xml_new_menu_entry(config_menu_t *menu) {
  config_menu_entry_t *me = menu->entries;
  int cnt; for(cnt=0;me[cnt].type != CONFIG_MENU_ENTRY_UNKNOWN;cnt++);

  // Allocate space for one more entry. There need to be two more enries than
  // used by now. One for the new entry and one for the end marker.
  menu->entries = reallocarray(menu->entries, cnt+2, sizeof(config_menu_entry_t));
  menu->entries[cnt+1].type = CONFIG_MENU_ENTRY_UNKNOWN;
  return &menu->entries[cnt];
}

static config_menu_t *config_xml_new_menu(config_menu_t *parent) {
  config_menu_t *menu = malloc(sizeof(config_menu_t));
  menu->label = NULL;
  menu->entries = malloc(sizeof(config_menu_entry_t));
  menu->entries[0].type = CONFIG_MENU_ENTRY_UNKNOWN;
  
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
  while(me->type != CONFIG_MENU_ENTRY_UNKNOWN) {
    if(me->type == CONFIG_MENU_ENTRY_MENU && depth)
      last = config_xml_get_menu(me->menu, depth-1);
    me++;
  }    
  return last;
}

static config_menu_entry_t *config_xml_get_last_menu_entry(config_menu_t *menu, int depth) {
  menu = config_xml_get_menu(menu, depth);
  config_menu_entry_t *me = menu->entries;
  while(me->type != CONFIG_MENU_ENTRY_UNKNOWN) me++;
  return me-1;
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
    menu->label = strdup(value);    
  
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

  for(int i=0;mnu->entries[i].type != CONFIG_MENU_ENTRY_UNKNOWN;i++) {
    switch(mnu->entries[i].type) {
    case CONFIG_MENU_ENTRY_MENU:
      config_dump_menu(mnu->entries[i].menu);
      break;
    case CONFIG_MENU_ENTRY_FILESELECTOR:
      config_dump_fileselector(mnu->entries[i].fsel);
      break;
    case CONFIG_MENU_ENTRY_LIST:
      config_dump_list(mnu->entries[i].list);
      break;
    case CONFIG_MENU_ENTRY_BUTTON:
      config_dump_button(mnu->entries[i].button);
      break;
    case CONFIG_MENU_ENTRY_IMAGE:
      config_dump_image(mnu->entries[i].image);
      break;
    case CONFIG_MENU_ENTRY_TOGGLE:
      config_dump_toggle(mnu->entries[i].toggle);
      break;
    }    
  }
}

/* ============================================================================= */
/* ============================= fileselector ================================== */
/* ============================================================================= */
 
static void config_xml_new_fileselector(config_menu_t *menu) {
  config_fsel_t *fsel = malloc(sizeof(config_fsel_t));
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
  char *s, **ptr = NULL;
  int i=0;

  while((s = strchr(str, sep))) {
    ptr = reallocarray(ptr, sizeof(char*), i+1);
    ptr[i] = malloc(s-str+1);
    strncpy(ptr[i], str, s-str+1);
    ptr[i][s-str] = '\0';
    str = s+1;
    i++;
  }

  // copy last string and append a null pointer to mark the end of the list
  ptr = reallocarray(ptr, sizeof(char*), i+2);
  ptr[i] = strdup(str);
  ptr[i+1] = NULL;

  return ptr;
}

static void config_xml_fsel_attribute(char *name, char *value) {
  config_menu_entry_t *me = config_xml_get_last_menu_entry(cfg->menu, config_depth-2);
  if(me && me->type == CONFIG_MENU_ENTRY_FILESELECTOR) {    
    if(me->fsel && strcasecmp(name, "label") == 0 && !me->fsel->label)
      me->fsel->label = strdup(value);	  
    else if(me->fsel && strcasecmp(name, "ext") == 0 && !me->fsel->ext)
      me->fsel->ext = config_parse_strlist(value, ';');
    else if(me->fsel && strcasecmp(name, "index") == 0)
      me->fsel->index = atoi(value);
    else if(me->fsel && strcasecmp(name, "default") == 0)
      me->fsel->def = strdup(value);
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
  config_list_t *list = malloc(sizeof(config_list_t));
  list->id = -1;
  list->def = -1;
  list->label = NULL;
  list->action = NULL;
  list->listentries = malloc(sizeof(config_listentry_t *));
  list->listentries[0] = NULL;

  config_menu_entry_t *me = config_xml_new_menu_entry(menu);
  me->type = CONFIG_MENU_ENTRY_LIST;
  me->list = list;
}

static void config_xml_new_listentry(config_list_t *list) {
  config_listentry_t *listentry = malloc(sizeof(config_listentry_t));
  listentry->value = 0;
  listentry->label = NULL;

  int cnt;
  for(cnt=0;list->listentries[cnt];cnt++);
  list->listentries = reallocarray(list->listentries, cnt+2, sizeof(config_listentry_t));
  list->listentries[cnt] = listentry;
  list->listentries[cnt+1] = NULL;
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
      me->list->label = strdup(value);      
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
    int cnt;
    for(cnt=0;me->list->listentries[cnt];cnt++);
    cnt--;
    
    if(strcasecmp(name, "label") == 0 && !me->list->listentries[cnt]->label)
      me->list->listentries[cnt]->label = strdup(value);      
    else if(strcasecmp(name, "value") == 0)
      me->list->listentries[cnt]->value = atoi(value);      
    
    else
      debugf("WARNING: Unused listentry attribute '%s'", name);
  }
}

static void config_dump_list(config_list_t *ls) {
  debugf("List, id='%c', label=\"%s\", default=\"%d\"", ls->id, ls->label, ls->def);
  for(int i=0;ls->listentries[i];i++)
    debugf("  Listentry, label=\"%s\", value=\"%d\"",
	   ls->listentries[i]->label, ls->listentries[i]->value);
  if(ls->action) config_dump_action(ls->action);
}
  
/* ============================================================================= */
/* ================================= button ==================================== */
/* ============================================================================= */

static void config_xml_new_button(config_menu_t *menu) {
  config_button_t *button = malloc(sizeof(config_button_t));
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
      me->button->label = strdup(value);      
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
  config_image_t *image = malloc(sizeof(config_image_t));
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
      me->image->label = strdup(value);      
    else if(me->image && strcasecmp(name, "ext") == 0 && !me->image->ext)
      me->image->ext = config_parse_strlist(value, ';');
    else if(me->image && strcasecmp(name, "index") == 0)
      me->image->index = atoi(value);
    else if(me->image && strcasecmp(name, "default") == 0)
      me->image->def = strdup(value);
    else if(me->image && strcasecmp(name, "none_str") == 0 && !me->image->none_str)
      me->image->none_str = strdup(value);      
    else if(me->image && strcasecmp(name, "none_icn") == 0 && !me->image->none_icn) {
      if(strlen(value) >= 16) {
	// convert 16 bytes hex string into 8 byte bitmap
	me->image->none_icn = malloc(8);
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
  config_toggle_t *toggle = malloc(sizeof(config_toggle_t));
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
      me->toggle->label = strdup(value);      
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
