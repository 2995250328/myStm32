#ifndef __BEEP_H
#define __BEEP_H     
#include "sys.h"

#define BEEP PAout(3)  

void BEEP_Init(void);         
void Sound(u16 frq);
void Sound2(u16 time);
void play_music(u8 music[],u8 time[],u16 length);
void play_successful(void);
void play_failed(void);

#endif
