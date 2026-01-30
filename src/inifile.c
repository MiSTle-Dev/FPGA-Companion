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
