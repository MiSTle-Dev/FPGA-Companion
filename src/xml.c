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
static int skip = 0;

// a name or value is built up char by char in a buffer that doubles when it is full and
// is kept for the next one: the heap is not touched for every char
typedef struct {
  char *str;
  int len, size;
} xml_buf_t;

static xml_buf_t name, value;
// chars at the start of value whose entities are decoded already
static int decoded = 0;

static void xml_buf_clear(xml_buf_t *b) {
  b->len = 0;
  if(b->str) b->str[0] = '\0';
}

static void xml_buf_free(xml_buf_t *b) {
  if(b->str) vPortFree(b->str);
  b->str = NULL;
  b->len = b->size = 0;
}

void xml_init(void) {
  state = 0;  // wait for open tag  
  depth = 0;  // start in root level 
  skip = 0;   // no element skipped
  decoded = 0;
  xml_buf_free(&name);
  xml_buf_free(&value);
}

static int xml_iswhite(unsigned char c) {
  return (c == ' ') || (c == '\t') || (c == '\r') || (c == '\n');  
}

// append char to buffer (a NUL char is not stored)
static void xml_buf_add(xml_buf_t *b, char c) {
  // space for the new char and the term char
  if(b->len + 2 > b->size) {
    int size = b->size ? 2 * b->size : 32;
    char *n = pvPortMalloc(size);
    if(b->str) {
      memcpy(n, b->str, b->len);
      vPortFree(b->str);
    }
    b->str = n;
    b->size = size;
  }
  if(c) b->str[b->len++] = c;
  b->str[b->len] = '\0';
}

// replace an entity at the end of the buffer with its char if it begins at or after start
static int xml_buf_unescape(xml_buf_t *b, int start, const char *entity, char c) {
  int n = strlen(entity);
  if(b->len - n < start || strcasecmp(entity, b->str + b->len - n)) return 0;
  b->len -= n - 1;
  b->str[b->len - 1] = c;
  b->str[b->len] = '\0';
  return 1;
}

int xml_parse(char c) {
  
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
      xml_buf_clear(&name);
      xml_buf_add(&name, c);
      state = 3;
    }
    break;

  case 2: // in <? ... > or <! ... >, bot are ignored
    if(c == '>') state = 0;
    break;

  case 3: // reading element name
    if(c == '/') {
      if(!skip) {
	if(xml_element_start_cb(name.str) != 0) skip = 1;
      } else skip++;
      xml_buf_clear(&name);
      if(!skip) xml_element_end_cb();
      else      skip--;
      state = 2;
    } else if(!xml_iswhite(c) && c != '>') xml_buf_add(&name, c);
    else {      
      if(!skip) {
	if(xml_element_start_cb(name.str) != 0) skip = 1;
      } else skip++;
      // the name is not needed anymore, the first attribute would overwrite it
      xml_buf_clear(&name);
      
      if(c == '>') state = 0;
      else         state = 4;
    }
    break;

  case 4: // search for attributes (if present)
    if(c == '>') state = 0;
    else if(c == '/') {
      if(!skip) xml_element_end_cb();
      else      skip--;
      xml_buf_clear(&name);
      state = 0;
    } else if(!xml_iswhite(c)) {      
      xml_buf_clear(&name);
      xml_buf_add(&name, c);
      state = 5;      
    }
    break;

  case 5: // reading attribute name
    if(c == '>') {
      debugf("unexpected '>' after attribute name");
      state = -1;
    } else if(!xml_iswhite(c) && c != '=') xml_buf_add(&name, c);
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
      // an empty value is passed as an empty string
      if(!skip) xml_attribute_cb(name.str, value.str ? value.str : "");
      xml_buf_clear(&name);
      xml_buf_clear(&value);
      decoded = 0;
      state = 4;     // -> search for next attribute
    } else {
      xml_buf_add(&value, c);

      // check if the string now ends with an entity. A decoded char never starts another one.
      if(c == ';' &&
	 (xml_buf_unescape(&value, decoded, "&amp;", '&') ||
	  xml_buf_unescape(&value, decoded, "&lt;", '<') ||
	  xml_buf_unescape(&value, decoded, "&gt;", '>')))
	decoded = value.len;
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

