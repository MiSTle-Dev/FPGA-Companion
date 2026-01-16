#ifndef SDC_DIRECT_H
#define SDC_DIRECT_H

bool sdc_direct_init(void);
void sdc_direct_release(void);

// called by main if accessing the FPGA times out
void sdc_boot(void);

#endif // SDC_DIRECT_H
