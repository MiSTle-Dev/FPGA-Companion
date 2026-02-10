#ifndef SDIO_H
#define SDIO_H

// these are called by sdc.c 
bool sdio_sector_write(uint32_t lba, const uint8_t *buffer, int count);
bool sdio_sector_read(uint32_t lba, uint8_t *buffer, int count);

void sdio_take_over(void);

// called by main if accessing the FPGA times out
void sdc_boot(void);

#endif // SDIO_H
