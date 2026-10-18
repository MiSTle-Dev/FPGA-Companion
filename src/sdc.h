#ifndef SDC_H
#define SDC_H

#include <stdbool.h>
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
int sdc_mount_defaults(void);   // returns number of images not mounted
bool sdc_wait_idle(void);
sdc_dir_entry_t *sdc_readdir_cfg(char **exts);
bool sdc_cfg_name_ok(const char *name, char **exts);
bool sdc_image_upload_in_progress(void);
bool sdc_check_for_pending_image_uploads(void);

#endif // SDC_H
