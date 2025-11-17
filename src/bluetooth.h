#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdbool.h>

// Initialize Bluetooth A2DP sink and HFP
void bluetooth_init(void);

// Initialize microphone for HFP (call after audio_manager_init)
void bluetooth_init_microphone(void);

// Force safe HFP mSBC encode (no zero-copy). Useful for testing and debugging.
void bluetooth_force_hfp_safe_encode(bool enable);
// Disable encoder for debug tests (encoder writes will be skipped)
void bluetooth_disable_hfp_encoder(bool disable);
// Enable debug-level logs for HFP encoder flow
void bluetooth_set_hfp_debug_logs(bool enable);
// Reduce log output: set global log level to WARN but keep mic/Bluetooth traces
void bluetooth_enable_mic_trace_only(bool enable);
// Toggle HFP send mode: use queue instead of ringbuffer (helps debug corruption)
void bluetooth_use_hfp_queue(bool enable);

// Get connection status
bool bluetooth_is_connected(void);
bool bluetooth_is_playing_audio(void);

// HFP (Hands-Free Profile) functions
bool bluetooth_is_call_active(void);
void bluetooth_answer_call(void);
void bluetooth_reject_call(void);
void bluetooth_hangup_call(void);

#endif // BLUETOOTH_H
