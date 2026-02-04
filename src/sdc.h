#ifndef SDC_H
#define SDC_H

#include "config.h"
#include <ff.h>

// fatfs mounts the card under /sd
#define CARD_MOUNTPOINT "/sd"

typedef struct sdc_dir {
  char *name;
  unsigned long len;
  int is_dir;
  struct sdc_dir *next;
} sdc_dir_entry_t;

int sdc_init(void);
int sdc_image_open(int drive, char *name);
sdc_dir_entry_t *sdc_readdir(int drive, char *name, const char *exts);
int sdc_handle_event(void);
void sdc_lock(void);
void sdc_unlock(void);
char *sdc_get_image_name(int drive);
char *sdc_get_cwd(int drive);
void sdc_set_cwd(int drive, char *path);
void sdc_set_default(int drive, const char *name);
void sdc_mount_defaults(void);

#endif // SDC_H
