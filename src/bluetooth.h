#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdbool.h>

// Initialize Bluetooth A2DP sink and HFP
void bluetooth_init(void);

// Get connection status
bool bluetooth_is_connected(void);
bool bluetooth_is_playing_audio(void);

// HFP (Hands-Free Profile) functions
bool bluetooth_is_call_active(void);
void bluetooth_answer_call(void);
void bluetooth_reject_call(void);
void bluetooth_hangup_call(void);

#endif // BLUETOOTH_H
