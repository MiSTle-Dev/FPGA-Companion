#ifndef _BLUETOOTH_H
#define _BLUETOOTH_H

void bluetooth_init(void);  // set up and power on, returns
void bluetooth_run(void);   // runs the btstack loop, never returns
void bluetooth_scan(void);

#endif // _BLUETOOTH_H
