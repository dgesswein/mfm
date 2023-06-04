/*
 * msg.h
 *
 *  Created on: Dec 20, 2013
 *      Author: djg
 *  11/09/14 DJG Added new function
 *  09/06/14 DJG Added extra class of messages
 */

#ifndef MSG_H_
#define MSG_H_


#define MSG_DEBUG_DATA 0x001
#define MSG_DEBUG 0x002
#define MSG_INFO 0x004
#define MSG_PROGRESS 0x008
#define MSG_ERR 0x010
#define MSG_INFO_SUMMARY 0x020
#define MSG_ERR_SERIOUS 0x040
#define MSG_ERR_SUMMARY 0x080
#define MSG_FATAL 0x100
#define MSG_STATS 0x200
#define MSG_FORMAT 0x400

void msg(uint32_t level, char *format, ...);
uint32_t msg_set_err_mask(uint32_t mask);
uint32_t msg_get_err_mask(void);
void *msg_malloc(size_t size, char *msgstr);
void msg_set_logfile(FILE *file, uint32_t mask);
#endif /* MSG_H_ */
