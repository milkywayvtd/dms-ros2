uint8 NORMAL=0
uint8 EYES_CLOSED=1
uint8 DISTRACTED=2
uint8 OUT_OF_FRAME=3
uint8 SYSTEM_FAULT=4

builtin_interfaces/Time stamp
uint8 state
bool eyes_closed_alarm
bool distraction_alarm
bool out_of_frame_alarm
float32 eyes_closed_duration
float32 distraction_duration
float32 out_of_frame_duration
bool camera_fault
