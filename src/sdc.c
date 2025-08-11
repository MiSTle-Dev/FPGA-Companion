//
// sdc.c - sd card access 
//
//


#include <ff.h>
#include <stdio.h>
#include <stdlib.h>
#include <diskio.h>
#include <string.h>

#include "sdc.h"
#include "sysctrl.h"

#ifdef ESP_PLATFORM
#ifndef CONFIG_FATFS_USE_FASTSEEK
#error "Please enable FATFS_USE_FASTSEEK!"
#error "Use 'idf.py menuconfig' and then navigate to"
#error "Component Config -> FAT Filesystem Support"
#error "and set 'Enable fast seek algorithm ...'"
#endif
#endif

// enable to use old way to determine cluster position
// #define USE_FSEEK

#include "debug.h"
#include "config.h"
#include "mcu_hw.h"

static SemaphoreHandle_t sdc_sem;

static FATFS fs;

static FIL fil[MAX_DRIVES];
static DWORD *lktbl[MAX_DRIVES];

static void sdc_spi_begin(void) {
  mcu_hw_spi_begin();  
  mcu_hw_spi_tx_u08(SPI_TARGET_SDC);
}

static LBA_t clst2sect(DWORD clst) {
  clst -= 2;
  if (clst >= fs.n_fatent - 2)   return 0;
  return fs.database + (LBA_t)fs.csize * clst;
}

int sdc_read_sector(unsigned long sector, unsigned char *buffer) {
  // check if sd card is still busy as it may
  // be reading a sector for the core. Forcing a MCU read
  // may change the data direction from core to mcu while
  // the core is still reading
  unsigned char status;
  do {
    sdc_spi_begin();  
    mcu_hw_spi_tx_u08(SPI_SDC_STATUS);
    status = mcu_hw_spi_tx_u08(0);
    mcu_hw_spi_end();  
  } while(status & 0x02);   // card busy?

  sdc_spi_begin();  
  mcu_hw_spi_tx_u08(SPI_SDC_MCU_READ);
  mcu_hw_spi_tx_u08((sector >> 24) & 0xff);
  mcu_hw_spi_tx_u08((sector >> 16) & 0xff);
  mcu_hw_spi_tx_u08((sector >> 8) & 0xff);
  mcu_hw_spi_tx_u08(sector & 0xff);

  // todo: add timeout
  while(mcu_hw_spi_tx_u08(0));  // wait for ready

  // read 512 bytes sector data
  for(int i=0;i<512;i++) buffer[i] = mcu_hw_spi_tx_u08(0);

  mcu_hw_spi_end();

  //  sdc_debugf("sector %ld", sector);
  //  hexdump(buffer, 512);

  return 0;
}

int sdc_write_sector(unsigned long sector, const unsigned char *buffer) {
  // check if sd card is still busy as it may
  // be reading a sector for the core.
  unsigned char status;
  do {
    sdc_spi_begin();  
    mcu_hw_spi_tx_u08(SPI_SDC_STATUS);
    status = mcu_hw_spi_tx_u08(0);
    mcu_hw_spi_end();  
  } while(status & 0x02);   // card busy?

  sdc_spi_begin();  
  mcu_hw_spi_tx_u08(SPI_SDC_MCU_WRITE);
  mcu_hw_spi_tx_u08((sector >> 24) & 0xff);
  mcu_hw_spi_tx_u08((sector >> 16) & 0xff);
  mcu_hw_spi_tx_u08((sector >> 8) & 0xff);
  mcu_hw_spi_tx_u08(sector & 0xff);

  // write sector data
  for(int i=0;i<512;i++) mcu_hw_spi_tx_u08(buffer[i]);  

  // todo: add timeout
  while(mcu_hw_spi_tx_u08(0));  // wait for ready

  mcu_hw_spi_end();

  return 0;
}

// -------------------- fatfs read/write interface to sd card connected to fpga -------------------

#ifdef DEV_SD
#define SDC_RESULT int
#else
#define SDC_RESULT DRESULT
#endif

static SDC_RESULT sdc_read(BYTE *buff, LBA_t sector, UINT count) {
  sdc_debugf("sdc_read(%p,%lu,%u)", buff, sector, count);  
  sdc_read_sector(sector, buff);
  return 0;
}

static SDC_RESULT sdc_write(const BYTE *buff, LBA_t sector, UINT count) {
  sdc_debugf("sdc_write(%p,%lu,%u)", buff, sector, count);  
  sdc_write_sector(sector, buff);
  return 0;
}

static SDC_RESULT sdc_ioctl(BYTE cmd, void *buff) {
  sdc_debugf("sdc_ioctl(%d,%p)", cmd, buff);

  switch(cmd) {
  case GET_SECTOR_SIZE:
    *((WORD*) buff) = 512;
    return RES_OK;
    break;
  }
  
  return RES_ERROR;
}

#ifdef ESP_PLATFORM

#if !FF_USE_STRFUNC
#error "FatFS string functions are not enabled!"
#error "Please set FF_USE_STRFUNC to 1 in esp-idf/components/fatfs/src/ffconf.h"
#endif

#if !FF_FS_EXFAT
#error "FatFS exFAT support is not enabled!"
#error "Please set FF_FS_EXFAT to 1 in esp-idf/components/fatfs/src/ffconf.h"
#endif

#if CONFIG_WL_SECTOR_SIZE != 512
#error "Please set wear levelling sector size to 512!"
#endif

#include <diskio_impl.h>
DRESULT sdc_disk_ioctl(__attribute__((unused)) BYTE pdrv, BYTE cmd, void *buff) { return sdc_ioctl(cmd, buff); }
DRESULT sdc_disk_read(__attribute__((unused)) BYTE pdrv, BYTE *buff, LBA_t sector, UINT count) { return sdc_read(buff, sector, count); }
DRESULT sdc_disk_write(__attribute__((unused)) BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count) { return sdc_write(buff, sector, count); }
DSTATUS sdc_disk_status(BYTE pdrv) { return 0; }
DSTATUS sdc_disk_initialize(BYTE pdrv) { sdc_debugf("sdc_initialize(%d)", pdrv); return 0; }

#else
#ifndef DEV_SD  // bouffalo sdk sets DEV_SD
// FatFS variant in bouffalo SDK defines DEV_SD
DRESULT disk_ioctl(__attribute__((unused)) BYTE pdrv, BYTE cmd, void *buff) { return sdc_ioctl(cmd, buff); }
DRESULT disk_read(__attribute__((unused)) BYTE pdrv, BYTE *buff, LBA_t sector, UINT count) { return sdc_read(buff, sector, count); }
DRESULT disk_write(__attribute__((unused)) BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count) { return sdc_write(buff, sector, count); }
DSTATUS disk_status(__attribute__((unused)) BYTE pdrv) { return 0; }
DSTATUS disk_initialize(__attribute__((unused)) BYTE pdrv) { return 0; }
#else
static int sdc_status() { return 0; }
static int sdc_initialize() { return 0; }
static DSTATUS Translate_Result_Code(int result) { return result; }
#endif
#endif

static int fs_init() {
  FRESULT res_msc;

  for(int i=0;i<MAX_DRIVES;i++)
    lktbl[i] = NULL;

#ifdef DEV_SD
  FATFS_DiskioDriverTypeDef MSC_DiskioDriver = { NULL };
  MSC_DiskioDriver.disk_status = sdc_status;
  MSC_DiskioDriver.disk_initialize = sdc_initialize;
  MSC_DiskioDriver.disk_write = sdc_write;
  MSC_DiskioDriver.disk_read = sdc_read;
  MSC_DiskioDriver.disk_ioctl = sdc_ioctl;
  MSC_DiskioDriver.error_code_parsing = Translate_Result_Code;
  
  disk_driver_callback_init(DEV_SD, &MSC_DiskioDriver);
#endif

#ifdef ESP_PLATFORM
  const ff_diskio_impl_t sdc_impl = {
    .init = sdc_disk_initialize,
    .status = sdc_disk_status,
    .read = sdc_disk_read,
    .write = sdc_disk_write,
    .ioctl = sdc_disk_ioctl,
  };
  
  ff_diskio_register(0, &sdc_impl);
#endif
    
  // wait for SD card to become available
  // TODO: display error in OSD
  unsigned char status;
  int timeout = 200;
  do {
    sdc_spi_begin();  
    mcu_hw_spi_tx_u08(SPI_SDC_STATUS);
    status = mcu_hw_spi_tx_u08(0);
    mcu_hw_spi_end();

    if((status & 0xf0) != 0x80) {
      timeout--;
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  } while(timeout && ((status & 0xf0) != 0x80));
  // getting here with a timeout either means that there
  // is no matching core on the FPGA or that there is no
  // SD card inserted
  
  // switch rgb led to green
  if(!timeout) {
    sdc_debugf("SD not ready, status = %d", status);
    sys_set_rgb(0x400000);  // red, failed
    return -1;
  }
  
  char *type[] = { "UNKNOWN", "SDv1", "SDv2", "SDHCv2" };
  sdc_debugf("SDC status: %02x", status);
  sdc_debugf("  card status: %d", (status >> 4)&15);
  sdc_debugf("  card type: %s", type[(status >> 2)&3]);

  res_msc = f_mount(&fs, CARD_MOUNTPOINT, 1);
  if (res_msc != FR_OK) {
    sdc_debugf("mount fail,res:%d", res_msc);
    sys_set_rgb(0x400000);  // red, failed
    return -1;
  }
  
  sys_set_rgb(0x004000);  // green, ok
  return 0;
}

// -------------- higher layer routines provided to the firmware ----------------
 
// keep track of working directory for each drive
static char *cwd[MAX_DRIVES];
static char *image_name[MAX_DRIVES];

void sdc_set_default(int drive, const char *name) {
  sdc_debugf("set default %d: %s", drive, name);
  
  // A valid filename will currently always begin with the mount point
  // This is actually handled differently with different firmware variants (bl616/pico/...)
  // and the settings are thus not portable
  if(strncasecmp(name, CARD_MOUNTPOINT, strlen(CARD_MOUNTPOINT)) == 0) {
    // name should consist of path and image name
    char *p = strrchr(name+strlen(CARD_MOUNTPOINT), '/');
    if(p && *p) {
      if(cwd[drive]) free(cwd[drive]);
      cwd[drive] = strndup(name, p-name);
      if(image_name[drive]) free(image_name[drive]);
      image_name[drive] = strdup(p+1);
    }
  }
}

char *sdc_get_image_name(int drive) {
  return image_name[drive];
}

char *sdc_get_cwd(int drive) {
  return cwd[drive];
}

#ifndef USE_FSEEK
// this function has been taken from fatfs ff.c as it's static there
static DWORD clmt_clust(FIL *fp, FSIZE_t ofs) {
  DWORD cl, ncl;
  DWORD *tbl;
  FATFS *fs = fp->obj.fs;
  
  tbl = fp->cltbl + 1;                    /* Top of CLMT */
  cl = (DWORD)(ofs / FF_MAX_SS / fs->csize); /* Cluster order from top of the file */
  for (;;) {
    ncl = *tbl++; /* Number of cluters in the fragment */
    if (ncl == 0)
      return 0; /* End of table? (error) */
    if (cl < ncl)
      break; /* In this fragment? */
    cl -= ncl;
    tbl++; /* Next fragment */
  }
  return cl + *tbl; /* Return the cluster number */
}
#endif

int sdc_handle_event(void) {  
  // read sd status
  sdc_spi_begin();  
  mcu_hw_spi_tx_u08(SPI_SDC_STATUS);
  mcu_hw_spi_tx_u08(0);
  unsigned char request = mcu_hw_spi_tx_u08(0);
  unsigned long rsector = 0;
  for(int i=0;i<4;i++) rsector = (rsector << 8) | mcu_hw_spi_tx_u08(0); 
  mcu_hw_spi_end();

  if(request) {
    int drive = 0;
    while(!(request & (1<<drive))) drive++;

    if(!fil[drive].flag) {
      // no file selected
      // this should actually never happen as the core won't request
      // data if it hasn't been told that an image is inserted
      return -1;
    }
    
    // ---- figure out which physical sector to use ----
  
    // translate sector into a cluster number inside image
    sdc_lock();
#ifdef USE_FSEEK
    f_lseek(&fil[drive], (rsector+1)*512);
    // and add sector offset within cluster    
    unsigned long dsector = clst2sect(fil[drive].clust) + rsector%fs.csize;    
#else
    // derive cluster directly from table
    unsigned long dsector = clst2sect(clmt_clust(&fil[drive], rsector*512)) + rsector%fs.csize;
#endif
    
    sdc_debugf("DRV %d: lba %lu = %lu", drive, rsector, dsector);

    // send sector number to core, so it can read or write the right
    // sector from/to its local sd card
    sdc_spi_begin();  
    mcu_hw_spi_tx_u08(SPI_SDC_CORE_RW);
    mcu_hw_spi_tx_u08((dsector >> 24) & 0xff);
    mcu_hw_spi_tx_u08((dsector >> 16) & 0xff);
    mcu_hw_spi_tx_u08((dsector >> 8) & 0xff);
    mcu_hw_spi_tx_u08(dsector & 0xff);

    // wait while core is busy to make sure we don't start
    // requesting data for ourselves while the core is still
    // doing its own io
    while(mcu_hw_spi_tx_u08(0) & 1);
    
    mcu_hw_spi_end();

    sdc_unlock();
  }

  return 0;
}

static void sdc_image_enable_direct(char drive, unsigned long start) {
  sdc_debugf("DRV %d: enable direct mapping @%lu", drive, start);
  
  sdc_spi_begin();
  mcu_hw_spi_tx_u08(SPI_SDC_DIRECT);
  mcu_hw_spi_tx_u08(drive);
  
  // send start sector
  mcu_hw_spi_tx_u08((start >> 24) & 0xff);
  mcu_hw_spi_tx_u08((start >> 16) & 0xff);
  mcu_hw_spi_tx_u08((start >> 8) & 0xff);
  mcu_hw_spi_tx_u08(start & 0xff);
  
  mcu_hw_spi_end();
}

static int sdc_image_inserted(char drive, FSIZE_t size) {
  // report the size of the inserted image to the core. This is needed
  // to guess sector/track/side information for floppy disk images, so the
  // core can translate from floppy disk to LBA

  if(size) {
    if(sizeof(FSIZE_t) == 8) sdc_debugf("DRV %d: inserted. Size = %llu", drive, size);
    else                     sdc_debugf("DRV %d: inserted. Size = %lu", drive, (uint32_t)size);
  } else                     sdc_debugf("DRV %d: ejected", drive);
  
  sdc_spi_begin();

  // files over 4GB size are reported using an extra command. This also prevents
  // cores not coping with big files from messing them up
  if( (sizeof(FSIZE_t) == 8) && (size >= 0x100000000llu))
    mcu_hw_spi_tx_u08(SPI_SDC_INS_LARGE);
  else
    mcu_hw_spi_tx_u08(SPI_SDC_INSERTED);
  
  mcu_hw_spi_tx_u08(drive);

  // send upper 32 bits of large file
  if((sizeof(FSIZE_t) == 8) && (size >= 0x100000000llu)) {
    mcu_hw_spi_tx_u08((size >> 56) & 0xff);
    mcu_hw_spi_tx_u08((size >> 48) & 0xff);
    mcu_hw_spi_tx_u08((size >> 40) & 0xff);
    mcu_hw_spi_tx_u08((size >> 32) & 0xff);
  }
    
  // send file length
  mcu_hw_spi_tx_u08((size >> 24) & 0xff);
  mcu_hw_spi_tx_u08((size >> 16) & 0xff);
  mcu_hw_spi_tx_u08((size >> 8) & 0xff);
  mcu_hw_spi_tx_u08(size & 0xff);

  mcu_hw_spi_end();

  return 0;
}

int sdc_image_open(int drive, char *name) {
  unsigned long start_sector = 0;
  
  // tell core that the "disk" has been removed
  sdc_image_inserted(drive, 0);

  // forget about any previous name
  if(image_name[drive]) {
    free(image_name[drive]);
    image_name[drive] = NULL;
  }
  
  // nothing to be inserted? Do nothing!
  if(!name) return 0;

  // assemble full name incl. path
  char fname[strlen(cwd[drive]) + strlen(name) + 2];
  strcpy(fname, cwd[drive]);
  strcat(fname, "/");
  strcat(fname, name);
  
  sdc_lock();
  
  // close any previous image, especially free the link table
  if(fil[drive].cltbl) {
    sdc_debugf("DRV %d: freeing link table", drive);
    free(lktbl[drive]);
    lktbl[drive] = NULL;
    fil[drive].cltbl = NULL;
  }
  
  sdc_debugf("DRV %d: Mounting %s", drive, fname);

  if(f_open(&fil[drive], fname, FA_OPEN_EXISTING | FA_READ) != 0) {
    sdc_debugf("DRV %d: file open failed", drive);
    sdc_unlock();
    return -1;
  } else {
    sdc_debugf("DRV %d: file opened, cl=%lu(%lu)", drive,
	   fil[drive].obj.sclust, clst2sect(fil[drive].obj.sclust));
    sdc_debugf("DRV %d: File len = %ld, spc = %d, clusters = %lu", drive,
	   (unsigned long)fil[drive].obj.objsize, fs.csize,
	   (unsigned long)fil[drive].obj.objsize / 512 / fs.csize);      
    
    // try with a 16 entry link table
    lktbl[drive] = malloc(16 * sizeof(DWORD));    
    fil[drive].cltbl = lktbl[drive];
    lktbl[drive][0] = 16;
    
    if(f_lseek(&fil[drive], CREATE_LINKMAP)) {
      // this isn't really a problem. But sector access will
      // be slower
      sdc_debugf("DRV %d: Short link table creation failed, "
		 "required size: %lu", drive, lktbl[drive][0]);

      // re-alloc sufficient memory
      lktbl[drive] = realloc(lktbl[drive], sizeof(DWORD) * lktbl[drive][0]);

      // and retry link table creation
      if(f_lseek(&fil[drive], CREATE_LINKMAP)) {
	sdc_debugf("DRV %d: Link table creation finally failed, "
		   "required size: %lu", drive, lktbl[drive][0]);
	free(lktbl[drive]);
	lktbl[drive] = NULL;
	fil[drive].cltbl = NULL;

	sdc_unlock();
	return -1;
      } else 
	sdc_debugf("DRV %d: Link table ok with %ld entries", drive, lktbl[drive][0]);
    } else {
      sdc_debugf("DRV %d: Short link table ok with %ld entries", drive, lktbl[drive][0]);

      // A link table length of 4 means, that  there's only one entry in it. This
      // in turn means that the file is continious. The start sector can thus be
      // sent to the core which can then access any sector without further help
      // by the MCU.
      if(lktbl[drive][0] == 4 && lktbl[drive][3] == 0)
	start_sector = clst2sect(lktbl[drive][2]);
    }
  }

  sdc_unlock();

  // remember current image name
  image_name[drive] = strdup(name);

  // image has successfully been opened, so report image size to core
  sdc_image_inserted(drive, fil[drive].obj.objsize);

  // allow direct mapping if possible
  if(start_sector) sdc_image_enable_direct(drive, start_sector);
  
  return 0;
}

sdc_dir_t *sdc_readdir(int drive, char *name, const char *ext) {
  static sdc_dir_t sdc_dir = { 0, NULL };

  int dir_compare(const void *p1, const void *p2) {
    sdc_dir_entry_t *d1 = (sdc_dir_entry_t *)p1;
    sdc_dir_entry_t *d2 = (sdc_dir_entry_t *)p2;

    // comparing directory with re111gular file?
    if(d1->is_dir != d2->is_dir)
      return d2->is_dir - d1->is_dir;

    return strcasecmp(d1->name, d2->name);    
  }

  void append(sdc_dir_t *dir, FILINFO *fno) {
    if(!(dir->len%8))
      // allocate room for 8 more entries
      dir->files = reallocarray(dir->files, dir->len + 8, sizeof(sdc_dir_entry_t));
      
    dir->files[dir->len].name = strdup(fno->fname);
    dir->files[dir->len].len = fno->fsize;
    dir->files[dir->len].is_dir = (fno->fattrib & AM_DIR)?1:0;
    dir->len++;
  }
  
  // check if a file name matches any of the extensions given
  char ext_match(char *name, const char *exts) {
    // check if name has an extension at all
    char *dot = strrchr(name, '.');
    if(!dot) return 0;

    if(cfg) {
      char **ext = (char**)exts;
      
      for(int i=0;ext[i];i++)
	if(!strcasecmp(dot+1, ext[i]))
	  return 1;

    } else {    
      // iterate over all extensions
      const char *ext = exts;
      while(1) {
	const char *p = ext;
	while(*p && *p != '+' && *p != ';') p++;  // search of end of ext
	unsigned int len = p-ext;
	
	// check if length would match
	if(strlen(dot+1) == len)
	  if(!strncasecmp(dot+1, ext, len))
	    return 1;  // it's a match
	
	// end of extension string reached: nothing found
	if(!*p) return 0;
	
	ext = p+1;
      }
    }
    return 0;
  }

#ifdef ESP_PLATFORM
  FF_DIR dir;
#else
  DIR dir;
#endif
  
  FILINFO fno;

  // assemble name before we free it
  if(name) {
    if(strcmp(name, "..")) {
      // alloc a longer string to fit new cwd
      char *n = malloc(strlen(cwd[drive])+strlen(name)+2);  // both strings + '/' and '\0'
      strcpy(n, cwd[drive]); strcat(n, "/"); strcat(n, name);
      free(cwd[drive]);
      cwd[drive] = n;
    } else {
      // no real need to free here, the unused parts will be free'd
      // once the cwd length increases. The menu relies on this!!!!!
      strrchr(cwd[drive], '/')[0] = 0;
    }
  }
  
  // free existing file names
  if(sdc_dir.files) {
    for(int i=0;i<sdc_dir.len;i++)
      free(sdc_dir.files[i].name);

    free(sdc_dir.files);
    sdc_dir.len = 0;
    sdc_dir.files = NULL;
  }

  // add "<UP>" entry for anything but root
  if(strcmp(cwd[drive], CARD_MOUNTPOINT) != 0) {
    strcpy(fno.fname, "..");
    fno.fattrib = AM_DIR;
    append(&sdc_dir, &fno);
  } else {
    // the root also gets a special entry for "eject" or No Disk
    // It's identified by the leading /, so the name can be changed
    strcpy(fno.fname, "/No Disk");
    fno.fattrib = AM_DIR;
    append(&sdc_dir, &fno);    
  }

  sdc_debugf("max name len = %d", FF_LFN_BUF);

  sdc_lock();
  
  int ret = f_opendir(&dir, cwd[drive]);
  sdc_debugf("opendir(%s)=%d", cwd[drive], ret);
  
  if(ret == 0) {  
    do {
      f_readdir(&dir, &fno);
      if(fno.fname[0] != 0 && !(fno.fattrib & (AM_HID|AM_SYS)) ) {
	sdc_debugf("%s %s, len=%lld", (fno.fattrib & AM_DIR) ? "dir: ":"file:", fno.fname, fno.fsize);

	// only accept directories or .ST/.HD files
	if((fno.fattrib & AM_DIR) || ext_match(fno.fname, ext)) 
	  append(&sdc_dir, &fno);
      }
    } while(fno.fname[0] != 0);

    f_closedir(&dir);

    qsort(sdc_dir.files, sdc_dir.len, sizeof(sdc_dir_entry_t), dir_compare);
  }
    
  sdc_unlock();

  return &sdc_dir;
}

int sdc_init(void) {
  sdc_sem = xSemaphoreCreateMutex();

  sdc_debugf("---- SDC init ----");

  if(fs_init() == 0) {
    // setup paths
    for(int d=0;d<MAX_DRIVES;d++) {
      cwd[d] = strdup(CARD_MOUNTPOINT);
      image_name[d] = NULL;
    }
    sdc_debugf("SD card is ready");
  }
    
  return 0;
}

void sdc_mount_defaults(void) {
  sdc_debugf("Mounting all default images ...");

  // try to mount (default) images
  for(int drive=0;drive<MAX_DRIVES;drive++) {
    char *name = sdc_get_image_name(drive);
    sdc_debugf("Processing drive %d: %s", drive, name?name:"<no image>");
    
    if(name) {
      // create a local copy as sdc_image_open frees its own copy
      char local_name[strlen(name)+1];
      strcpy(local_name, name);
      
      if(sdc_image_open(drive, local_name) != 0) {
	// open failed, also reset the path
	if(cwd[drive]) free(cwd[drive]);
	cwd[drive] = strdup(CARD_MOUNTPOINT);
      }
    }
  }
}

// use a locking mechanism to make sure the file system isn't modified
// by two threads at the same time
void sdc_lock(void) {
  xSemaphoreTake(sdc_sem, 0xffffffffUL); // wait forever
}

void sdc_unlock(void) {
  xSemaphoreGive(sdc_sem);
}
