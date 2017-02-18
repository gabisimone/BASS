/*
 * ledsManager.h
 *
 *  Created on: 7/12/2016
 *      Author: marco
 */

#ifndef EXAMPLES_BASS_INC_LEDSMANAGER_H_
#define EXAMPLES_BASS_INC_LEDSMANAGER_H_

static void boardGpiosInit(void);
static void shiftInit(void);
static void shiftInitT(void);
static void shiftClock(void);
static void shiftClockT(void);
static void shiftData(uint8_t dataToSend);
void ledsControl(uint8_t* data);
void shiftBoardInit(void);


#endif /* EXAMPLES_BASS_INC_LEDSMANAGER_H_ */
