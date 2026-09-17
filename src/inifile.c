/*
  inifile.c  
 */

#include "inifile.h"
#include "ff.h"
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include "sdc.h"      // for CARD_MOUNTPOINT
#include "menu.h"     // to access menu variables
#include "config.h"
#include "debug.h"

static int iswhite(char c) {
  return c == ' ' || c == '\r' || c == '\n' || c == '\t';
}

static const struct option_S { char *name; char *info; int index; } option_ids[] = {
  {"hotkey", "; HID key code of OSD/menu hotkey\n",  INIFILE_OPTION_HOTKEY },
  {"led",    "; led state (0=blink, 1=on, 2=off)\n", INIFILE_OPTION_LED },
  {NULL,     NULL,                                   -1 }
};

static int options[2] = { 0x45, 0 };  // default options: hotkey=F12, led=blink
static void inifile_parse_option(char *id, char *value) {
  for(const struct option_S *oid = option_ids;oid->name;oid++) {
    if(!strcasecmp(oid->name, id)) {
      options[oid->index] = atoi(value);
      ini_debugf("option %s: %d", id, options[oid->index]);
    }
  }
}

int inifile_option_get(int id) {
  if((id < 0) || (id > 1)) return -1;
  return options[id];
}

int inifile_read(char *name) {
  if(!name) {
    ini_debugf("Unable to load core specific setting as no core has been identified");
    return -1;
  }

  char *filename = pvPortMalloc(strlen(CARD_MOUNTPOINT) + strlen(name) + 2);  // MP+'/'+name+'\0'
  strcpy(filename, CARD_MOUNTPOINT);
  strcat(filename, "/");
  strcat(filename, name);
  
  ini_debugf("Reading settings from '%s'", filename);

  sdc_lock();  // get exclusive access to the file system

  FIL fil;
  if(f_open(&fil, filename, FA_OPEN_EXISTING | FA_READ) == FR_OK) {
    char buffer[FF_LFN_BUF+10];

    ini_debugf("Settings file opened");
    // read file line by line
    while(f_gets(buffer, sizeof(buffer), &fil) != NULL) {
      // ignore everything after semicolon
      char *pos = strchr(buffer, ';');
      if(pos) *pos = '\0';

      // also skip all trailing white space
      while(strlen(buffer) > 0 && iswhite(buffer[strlen(buffer)-1]))
	buffer[strlen(buffer)-1] = 0;

      // ini_debugf("Line = '%s'\n", buffer);
      // check for drives or images
      if((strncasecmp(buffer, "drive", 5) == 0) ||
	 (strncasecmp(buffer, "image", 5) == 0) ) {
	char is_drive = buffer[0] == 'd' || buffer[0] == 'D';
	char * p = buffer+5;  // skip 'drive'/'image'
	while(*p && iswhite(*p)) p++;
	if(*p) {
	  int drive = *p-'0';
	  // skip after '='
	  while(*p && *p != '=') p++;
	  p++;
	  if(*p) {
	    // skip to begin of filename
	    while(*p && iswhite(*p)) p++;

#ifdef INIFILE_PREFIX
	    // skip an extra file name prefix that may have been added for
	    // compatibility between the different MCUs. See sdc.h for more details.
	    if(strncasecmp(p, INIFILE_PREFIX, strlen(INIFILE_PREFIX)) == 0)
	      p += strlen(INIFILE_PREFIX);
#endif
	    
	    if(*p) {
	      // tell SDC layer what images to use as default
	      ini_debugf("%s %d = %s", is_drive?"drive":"image",drive, p);
	      sdc_set_default(is_drive?drive:(drive+MAX_DRIVES), p);
	    }
	  }
	}
      }
      
      // check for variables 
      if(strncasecmp(buffer, "var ", 4) == 0) {
	
	// --- parse 'var x=0` style lines ---
	// skip "var"
	char *p = buffer+4;
	// skip to first char
	while(*p && iswhite(*p)) p++;
	if(*p) {	  
	  char id = *p++;
	  // skip until '='
	  while(*p && *p != '=') p++;
	  p++;  // skip =
	  if(*p) {
	    // skip all whites
	    while(*p && iswhite(*p)) p++;
	    if(*p) {
	      int value = atoi(p);
	      ini_debugf("var %c = %d", id, value);
	      
	      // save values
	      menu_set_value(id, value);
	    }
	  }
	}
      }

      // check for firmware options
      if(strncasecmp(buffer, "option ", 7) == 0) {
	// skip "option"
	char *p = buffer+7;
	// skip to first char
	while(*p && iswhite(*p)) p++;
	if(*p) {	  
	  char *id = p;
	  // skip to end of if
	  while(*p && !iswhite(*p) && *p != '=') p++;
	  if(p && *p == '=') {
	    *p++ = '\0'; // terminate id (overwrites the '=')
	  } else {
	    *p++ = '\0'; // terminate id
	    // skip until '='
	    while(*p && *p != '=') p++;
	    p++;  // skip =
	  }
	    
	  if(*p) {
	    // skip all whites
	    while(*p && iswhite(*p)) p++;
	    if(*p)
	      inifile_parse_option(id, p);
	  }
	}
      }      
    }
    f_close(&fil);
  } else {
    ini_debugf("Error opening file %s", filename);
    vPortFree(filename);
    sdc_unlock();
    return -1;
  }
  vPortFree(filename);
  sdc_unlock();
  return 0;
}

void inifile_write(char *name) {
  if(!name) {
    ini_debugf("Unable to write core specific setting as no core has been identified");
    return;
  }
    
  char *filename = pvPortMalloc(strlen(CARD_MOUNTPOINT) + strlen(name) + 2);  // MP+'/'+name+'\0'
  strcpy(filename, CARD_MOUNTPOINT);
  strcat(filename, "/");
  strcat(filename, name);

  ini_debugf("Write settings to %s", filename);
  
  sdc_lock();  // get exclusive access to the file system
  
  // saving does not work, yet, as there is no SD card write support by now
  FIL file;
  if(f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS) == FR_OK) {    
    f_puts("; FPGA Companion settings\n", &file);

    // write variable values
    f_puts("\n; variables\n", &file);

    menu_variable_t *vars = menu_get_variables();
    while(vars) {
      char str[10];
      sprintf(str, "var %c=%d\n", vars->id, vars->value);
      f_puts(str, &file);
      vars = vars->next;
    }

    // write options
    f_puts("\n; firmware options\n", &file);
    for(const struct option_S *oid = option_ids;oid->name;oid++) {
      char str[32];
      f_puts(oid->info, &file);
      sprintf(str, "option %s=%d\n", oid->name, options[oid->index]);
      f_puts(str, &file);
    }
    
    // write disk and ROM image file names
    f_puts("\n; image files\n", &file);

    // disk images
    for(int drive=0;drive<MAX_DRIVES+MAX_IMAGES;drive++) {
      char *cwd = sdc_get_cwd(drive);
      char *image = sdc_get_image_name(drive);

      if(cwd && image) {
	char str[strlen(cwd) + strlen(image) + 12];
#ifdef INIFILE_PREFIX
	// add prefix for compatibility, see sdc.h for details
	sprintf(str, "%s%d=%s%s/%s\n", (drive<MAX_DRIVES)?"drive":"image",
		(drive<MAX_DRIVES)?drive:(drive-MAX_DRIVES), INIFILE_PREFIX, cwd, image);
#else
	sprintf(str, "%s%d=%s/%s\n", (drive<MAX_DRIVES)?"drive":"image",
		(drive<MAX_DRIVES)?drive:(drive-MAX_DRIVES), cwd, image);
#endif
	f_puts(str, &file);
      }      
    }

    f_puts("\n", &file);
    
    f_close(&file);  
  } else
    ini_debugf("Error opening file");
  
  vPortFree(filename);
  sdc_unlock();
}

// --------------------------------------------------------------------
// ------------ parse the global (not core specific) config -----------
// --------------------------------------------------------------------

#define CONFIG_INI  "config.ini"

enum {
  CONFIG_TYPE_UNKNOWN = 0,
  CONFIG_TYPE_STRING,
  CONFIG_TYPE_IP,
  CONFIG_TYPE_INT,
  CONFIG_TYPE_CHOICE
};

struct config_value_s {
  union {
    char *str;
    uint32_t ip;
    int val;
  } data;

  struct config_value_s *next;
};

struct config_s {
  char *name;
  int type;
  struct config_value_s *value;  
  struct config_s *next;
};

struct config_section_s {
  char *name;
  struct config_s *cfg;
  struct config_section_s *next;
};

static struct config_section_s *config_sections = NULL;

static const struct {
  char *section;
  char *name;
  int type;
  char *choices;
} keys[] = {
  // IP/internet settings
  { "NETWORK", "MODE", CONFIG_TYPE_CHOICE, "manual,dhcp" },
  { "NETWORK", "IP", CONFIG_TYPE_IP, NULL },
  { "NETWORK", "MASK", CONFIG_TYPE_IP, NULL },
  { "NETWORK", "GW", CONFIG_TYPE_IP, NULL },

  // wifi settings
  { "WIFI", "SSID", CONFIG_TYPE_STRING, NULL },
  { "WIFI", "PASS", CONFIG_TYPE_STRING, NULL },

  // (S)NTP
  { "NTP", "IP", CONFIG_TYPE_IP, NULL },
  { "NTP", "TIMEZONE", CONFIG_TYPE_INT, NULL },
  
  { NULL, NULL, 0, NULL }
};

static bool inifile_config_parse_ip(char *value, uint32_t *addr) {	
  // parse ip address
  *addr = 0;
  int dots = 0;
  while(strchr(value, '.')) {
    *addr = (*addr << 8) | atoi(value);
    value = strchr(value, '.')+1;
    dots++;
  }
  *addr = (*addr << 8) | atoi(value);

  return dots == 3;
}	

static void inifile_config_dump(void) {
  ini_debugf("Config:");

  struct config_section_s *sec = config_sections;
  while(sec) {
    struct config_s *cfg = sec->cfg;
    while(cfg) {
      ini_debugf("  %s/%s", sec->name, cfg->name);
      
      struct config_value_s *val = cfg->value;
      
      while(val) {    
	switch(cfg->type) {
	case CONFIG_TYPE_STRING:
	  ini_debugf("    %s", val->data.str);	
	  break;

	case CONFIG_TYPE_CHOICE:
	  ini_debugf("    %d", val->data.val);	
	  break;
	  
	case CONFIG_TYPE_INT:
	  ini_debugf("    %d", val->data.val);	
	  break;
	  
	case CONFIG_TYPE_IP:
	  ini_debugf("    %lu.%lu.%lu.%lu",
		     (val->data.ip>>24)&0xff, (val->data.ip>>16)&0xff,
		     (val->data.ip>>8)&0xff,  (val->data.ip>>0)&0xff);	
	  break;
	  
	default:
	  ini_debugf("    <?>");
	  break;
	}
	val = val->next;
      }      
      cfg = cfg->next;
    }
    sec = sec->next;
  }
}

// a FreeRTOS'd version of strdup
static inline char *StrDup(const char *s) {
  char *res = pvPortMalloc(strlen(s)+1);
  strcpy(res, s);
  return res;
}

static struct config_s *inifile_config_get(char *section, char *name) {
  struct config_section_s *sec = config_sections;
  while(sec) {
    struct config_s *cfg = sec->cfg;
    while(cfg) {
      if(!strcasecmp(section, sec->name) &&
	 !strcasecmp(name, cfg->name))
	return cfg;

      cfg = cfg->next;
    }
    sec = sec->next;
  }
  return NULL;
}

bool inifile_config_has(char *section, char *name) {
  return inifile_config_get(section, name) != NULL;
}

int inifile_config_num_values(char *section, char *name) {
  struct config_s *cfg = inifile_config_get(section, name);
  if(!cfg) return 0;

  int num = 0;
  struct config_value_s *val = cfg->value;
  while(val) {
    num++;
    val = val->next;
  }

  return num;
}

static struct config_value_s *inifile_config_get_value_n(char *section, char *name, int n) {
  struct config_s *cfg = inifile_config_get(section, name);
  if(!cfg) return NULL;

  struct config_value_s *val = cfg->value;
  if(!val) return NULL;

  while(n) {
    val = val->next;
    if(!val) return NULL;
  }

  return val;
}

uint32_t inifile_config_get_ip_n(char *section, char *name, uint32_t def, int n) {
  struct config_value_s *val = inifile_config_get_value_n(section, name, n);
  if(!val) return def;

  return val->data.ip;
}

uint32_t inifile_config_get_ip(char *section, char *name, uint32_t def) {
  return inifile_config_get_ip_n(section, name, def, 0);
}

int inifile_config_get_int(char *section, char *name, int def) {
  struct config_value_s *val = inifile_config_get_value_n(section, name, 0);
  if(!val) return def;

  return val->data.val;
}

char *inifile_config_get_str(char *section, char *name) {
  struct config_value_s *val = inifile_config_get_value_n(section, name, 0);
  if(!val) return NULL;

  return val->data.str;
}

static struct config_value_s *inifile_config_append_value(struct config_s **cfg) {
  // get pointer to end of value chain
  struct config_value_s **val = &((*cfg)->value);
  while(*val) val = &((*val)->next);
  
  // create a new value entry
  *val = pvPortMalloc(sizeof(struct config_value_s));
  (*val)->next = NULL;

  return *val;
}	

static void inifile_config_parse_line(char *line) {
  // check if this is a section marker
  if(strlen(line) > 2 && line[0] == '[' && line[strlen(line)-1] == ']') {
    line++;                        // skip [
    line[strlen(line)-1] = '\0';   // skip ]
    
    // search end of section list ...
    struct config_section_s **sec = &config_sections;
    while(*sec) sec = &((*sec)->next);

    // create a new section
    *sec = pvPortMalloc(sizeof(struct config_section_s));
    (*sec)->name = StrDup(line);
    (*sec)->cfg = NULL;
    (*sec)->next = NULL;
    
    return;
  }
    
  // cut at '=', return if none present
  if(!strchr(line, '=')) return;

  char *value = strchr(line, '=');
  *value++ = '\0';

  // cut trailing and leading whitespace from name ...
  while(iswhite(*line)) line++;
  while(strlen(line) > 0 && iswhite(line[strlen(line)-1])) line[strlen(line)-1] = 0;

  // ... and from value
  while(iswhite(*value)) value++;
  while(strlen(value) > 0 && iswhite(value[strlen(value)-1])) value[strlen(value)-1] = 0;

  for(int i=0;keys[i].name;i++) {
    if(!strcasecmp(keys[i].name, line)) {
      // search end of section list ...
      struct config_section_s *sec = config_sections;
      if(!sec) return;  // don't allow entries outside section      
      while(sec->next) sec = sec->next;

      // ... and there end of entry list ...
      struct config_s **cfg = &(sec->cfg);
      while(*cfg) cfg = &((*cfg)->next);
      
      // ... and create a new entry
      *cfg = pvPortMalloc(sizeof(struct config_s));
      (*cfg)->name = StrDup(line);
      (*cfg)->type = keys[i].type;
      (*cfg)->value = NULL;
      (*cfg)->next = NULL;
      
      switch(keys[i].type) {
      case CONFIG_TYPE_STRING:
	// just copy the string	
	inifile_config_append_value(cfg)->data.str = StrDup(value);
	break;

      case CONFIG_TYPE_INT:
	// parse integer	
	inifile_config_append_value(cfg)->data.val = atoi(value);
	break;
	
      case CONFIG_TYPE_CHOICE:
	// check if the choice is actually a valid one
	if(!keys[i].choices) return;

	// choices are comma seperated
	char *choice = keys[i].choices;
	int j=0;
	while(strchr(choice, ',')) {
	  if(!strncasecmp(choice, value, strchr(choice, ',')-choice)) {
	    inifile_config_append_value(cfg)->data.val = j;
	    return;
	  }	  
	  choice = strchr(choice, ',')+1;
	  j++;
	}

	if(!strcasecmp(choice, value)) {
	  inifile_config_append_value(cfg)->data.val = j;
	  return;
	}

	ini_debugf("Ignoring unexpected choice %s '%s'", line, value);
	break;	

      case CONFIG_TYPE_IP:
	// IP may actually be a comma seperated list of IPs
	uint32_t addr;
	while(strchr(value, ',')) {
	  *strchr(value, ',') = '\0';   // terminate first address string
	  if(inifile_config_parse_ip(value, &addr)) {
	    (*cfg)->type = keys[i].type;
	    inifile_config_append_value(cfg)->data.ip = addr;	    
	  }
	  
	  // we can just skip ahead since we known that there once was a comma
	  value += strlen(value)+1;
	}

	if(inifile_config_parse_ip(value, &addr))
	  inifile_config_append_value(cfg)->data.ip = addr;
	break;	
      }
      
      return;
    }
  }

  ini_debugf("Ignoring unknown config entry '%s'", line);
}
  
// read the system config.ini
void inifile_config_read(void) {
  ini_debugf("inifile_config_read()");

  char *filename = pvPortMalloc(strlen(CARD_MOUNTPOINT) + strlen(CONFIG_INI) + 2);  // MP+'/'+name+'\0'
  strcpy(filename, CARD_MOUNTPOINT);
  strcat(filename, "/");
  strcat(filename, CONFIG_INI);
  
  ini_debugf("Reading settings from '%s'", filename);

  sdc_lock();  // get exclusive access to the file system

  FIL fil;
  if(f_open(&fil, filename, FA_OPEN_EXISTING | FA_READ) == FR_OK) {
    char buffer[64];

    ini_debugf("Settings file %s opened", filename);
    bool no_eol = false;
    // read file line by line
    while(f_gets(buffer, sizeof(buffer), &fil) != NULL) {
      // check if last line was incomplete and if yes, ignore
      // this continuation of that line
      if(no_eol) {
	// this continuation may still not be the end of the
	// line.
	if(!strlen(buffer) || buffer[strlen(buffer)-1] == '\n') 
	  no_eol = false;
	
      } else {      
	// check if line ends with newline. If not, then it's
	// been truncated
	if(strlen(buffer) && buffer[strlen(buffer)-1] != '\n')
	  no_eol = true;
	
	// a truncated line may still be valid/usable if the truncated
	// part is only a comment. So process the line if its either
	// not truncated or if contains a semicolon
	if(!no_eol || strchr(buffer, ';')) {
	  
	  // ignore everything after semicolon
	  char *pos = strchr(buffer, ';');
	  if(pos) *pos = '\0';

	  // also skip all trailing white space
	  while(strlen(buffer) > 0 && iswhite(buffer[strlen(buffer)-1]))
	    buffer[strlen(buffer)-1] = 0;

	  // skip all leading whitespace
	  char *p = buffer;
	  while(iswhite(p[0])) p++;
	  
	  if(strlen(p))
	    inifile_config_parse_line(p);
	} else
	  ini_debugf("Warning, skipping incomplete line '%s'", buffer);
      }
    }
    f_close(&fil);
  } else
    ini_debugf("Error opening file %s", filename);

  vPortFree(filename);
  sdc_unlock();

  inifile_config_dump();
}
