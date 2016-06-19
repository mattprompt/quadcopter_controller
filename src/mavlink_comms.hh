#ifndef MAVLINK_COMMS_HH

#define MAVLINK_COMMS_HH

int mavlink_comms_init(int mav_number_in);
int mavlink_comms_end(void);
int mavlink_comms_receive(void);
int mavlink_comms_send(void);
uint64_t microsSinceEpoch(void);

#endif