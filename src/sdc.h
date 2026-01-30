#ifndef SDC_H
#define SDC_H

#include "config.h"
#include <ff.h>
#include <diskio.h>   // for DEV_SD

// fatfs mounts the card under /sd
#ifdef DEV_SD
// DEV_SD supported fatfs setups mount the sd card under /sd. This
// is then also used when storing file names in the ini file. Thus
// no additional prefix is needed on sd card
#define CARD_MOUNTPOINT "/sd"
#else
// non DEV_SD setups mount the sd card directly as the root fs.
// To be compatible in the ini files with the DEV_SD setups
// an additional PREFIX is added when storing the settings in the
// ini file. This allows to use the same ini file with all MCUs
// and e.g. swap cards between BL616 and rp2040 based setups,
// especially when mixing internal BL616 and external rp2040.
#define CARD_MOUNTPOINT ""
#define INIFILE_PREFIX "/sd"
#endif

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
void sdc_set_default(int drive, const char *name);
void sdc_mount_defaults(void);

#endif // SDC_H
