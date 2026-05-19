
#ifndef	__LINE_H__
#define __LINE_H__

#define ADC_N 8

void track_zhixian1(void);
void track_zhixian2(void);
void track_PID1(int pwm,float P);
void track_PID2(int pwm,float P);
void track_PID3(int pwm,float P);
char Check_BlackLine(void);

#endif

