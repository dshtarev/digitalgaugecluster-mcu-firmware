#ifndef __H_OBD_TASK
#define __H_OBD_TASK

#define OBD_MAX_FAULTS 20

void obd_task( void * );
void obd_read_dtc();
void obd_clear_dtc();

#endif
