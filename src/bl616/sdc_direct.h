#ifndef SDC_DIRECT_H
#define SDC_DIRECT_H

bool sdc_direct_init(void);
void sdc_direct_release(void);
bool sdc_direct_upload_core_bin(const char *name);

#endif // SDC_DIRECT_H
