#ifndef SDC_DIRECT_H
#define SDC_DIRECT_H

bool sdc_direct_init(void);
void sdc_direct_release(void);

// these are called by sdc.c 
bool sdc_direct_write(uint32_t lba, const uint8_t *buffer);
bool sdc_direct_read(uint32_t lba, uint8_t *buffer);

// called by main if accessing the FPGA times out
void sdc_boot(void);
bool sdc_direct_upload_core_bin(const char *name);

#endif // SDC_DIRECT_H
