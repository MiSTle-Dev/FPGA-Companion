#include <stdlib.h>
#include <string.h>

#ifdef ESP_PLATFORM
#include <freertos/FreeRTOS.h>
#else
#include <FreeRTOS.h>
#endif

#include "xml.h"
#include "debug.h"

/* ======================== very simple xml stream parser ======================== */

static int state = 0;
static int depth = 0;

void xml_init(void) {
  state = 0;  // wait for open tag  
  depth = 0;  // start in root level 
}

static int xml_iswhite(unsigned char c) {
  return (c == ' ') || (c == '\t') || (c == '\r') || (c == '\n');  
}

// append char to string
static char *xml_str_expand(char *s, char c) {
  int len = 0;
  
  if(!s) s = pvPortMalloc(2);
  else {
    len = strlen(s);

    // reallocate with space for term char and new char
    char *n = pvPortMalloc(len+2); 
    strcpy(n, s);
    vPortFree(s);
    s = n;
  }
    
  s[len] = c;
  s[len+1] = '\0';
  return s;
}

int xml_parse(char c) {
  static char *name = NULL;
  static char *value = NULL;
  static int skip = 0;
  
  // debugf("S%d: %d (%c)", state, c, (c>=32)?c:'.');

  switch(state) {
  case 0: // waiting for element start
    if(c == '<') state = 1;
    break;
    
  case 1: // first char after element start
    if(c == '?' || c == '!') state = 2;
    else if(c == '/') state = 10;
    else {
      // first char of element name, start buffering
      name = xml_str_expand(NULL, c);
      state = 3;
    }
    break;

  case 2: // in <? ... > or <! ... >, bot are ignored
    if(c == '>') state = 0;
    break;

  case 3: // reading element name
    if(c == '/') {
      if(!skip) {
	if(xml_element_start_cb(name) != 0) skip = 1;
      } else skip++;
      if(name) vPortFree(name);
      name = NULL;
      if(!skip) xml_element_end_cb();
      else      skip--;
      state = 2;
    } else if(!xml_iswhite(c) && c != '>') name = xml_str_expand(name, c);
    else {      
      if(!skip) {
	if(xml_element_start_cb(name) != 0) skip = 1;
      } else skip++;
      
      if(c == '>') state = 0;
      else         state = 4;
    }
    break;

  case 4: // search for attributes (if present)
    if(c == '>') state = 0;
    else if(c == '/') {
      if(!skip) xml_element_end_cb();
      else      skip--;
      if(name) vPortFree(name);
      name = NULL;
      state = 0;
    } else if(!xml_iswhite(c)) {      
      name = xml_str_expand(NULL, c);
      state = 5;      
    }
    break;

  case 5: // reading attribute name
    if(c == '>') {
      debugf("unexpected '>' after attribute name");
      state = -1;
    } else if(!xml_iswhite(c) && c != '=') name = xml_str_expand(name, c);
    else if(c == '=') state = 7;
    else state = 6;
    break;

  case 6: // search for equal sign
    if(c == '=') state = 7;
    else if(!xml_iswhite(c)) {
      debugf("expected '='");
      state = -1;
    }
    break;
    
  case 7: // search for attribute value
    if(c == '\"') state = 8;
    else if(c == '\'') state = 9;
    else if(!xml_iswhite(c)) {
      debugf("attribute value does not start with ' or \"");
      state = -1;
    }
    break;
    
  case 8:  // attribute value started with "
  case 9:  // attribute value started with '
    if((state == 8 && c == '\"') ||
       (state == 9 && c == '\'')) {
      if(!skip) xml_attribute_cb(name, value);
      if(name) vPortFree(name);
      name = NULL;
      if(value) vPortFree(value);
      value = NULL;
      state = 4;     // -> search for next attribute
    } else {
      value = xml_str_expand(value, c);

      // check if string now ends with escaped char
      if(strlen(value)>=5 && !strcasecmp("&amp;", value+strlen(value)-5))
	strcpy(value+strlen(value)-5, "&");
      else if(strlen(value)>=4 && !strcasecmp("&lt;", value+strlen(value)-4))
	strcpy(value+strlen(value)-4, "<");
      else if(strlen(value)>=4 && !strcasecmp("&gt;", value+strlen(value)-4))
	strcpy(value+strlen(value)-4, ">");
    }
    break;
    
  case 10: // closing element
    if(c == '>') {
      if(!skip) xml_element_end_cb();
      else      skip--;
      state = 0;
    }
    break;
  }
  
  return 0;
}

