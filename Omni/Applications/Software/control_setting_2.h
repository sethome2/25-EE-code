#ifndef CONTROL_SETTING_H
#define CONTROL_SETTING_H

void remote_control_task(void);
extern float auto_yaw,auto_pitch,last_yaw,last_pitch,visual_yaw;
#define YAW_DATA_HIS 1
#define PITCH_DATA_HIS 0

#endif /* CONTROL_SETTING_H */
