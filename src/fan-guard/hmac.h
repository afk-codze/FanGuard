#ifndef HMAC_AUTH_H
#define HMAC_AUTH_H

#include <Arduino.h>
#include "mbedtls/md.h"
#include "esp_attr.h"
#include <Preferences.h>

// HMAC Configuration
#define HMAC_KEY_LENGTH 32
#define HMAC_OUTPUT_LENGTH 32 // SHA-256 produces 32 bytes

// Preferences namespace and key names
#define PREF_NAMESPACE "iot_auth"
#define PREF_SESSION_ID "session_id"

// Structure for authentication data
typedef struct {
    uint32_t session_id;   // Persists across power cycles, stored in flash
    uint32_t seq_number;   // Monotonically increasing sequence number
    uint64_t timestamp;    // Message timestamp
} auth_data_t;

// Global authentication state
extern uint32_t g_session_id;        // Stored in flash memory
extern RTC_DATA_ATTR uint32_t g_sequence_number; // Persists through deep sleep

/**
 * @brief Initialize the HMAC authentication system
 * @note Loads or creates new session ID
 */
void hmac_auth_init();

/**
 * @brief Calculate HMAC for a message
 * @param payload Message payload
 * @param payload_len Length of the payload
 * @param auth Authentication data (session ID, sequence number, timestamp)
 * @param hmac_result Buffer to store resulting HMAC (must be at least HMAC_OUTPUT_LENGTH bytes)
 * @return true if successful, false otherwise
 */
bool calculate_hmac(const char* payload, size_t payload_len, 
                   auth_data_t* auth, uint8_t* hmac_result);

/**
 * @brief Convert binary HMAC to hex string representation
 * @param hmac_bin Binary HMAC value
 * @param hmac_hex Output buffer for hex string (must be at least 2*HMAC_OUTPUT_LENGTH+1 bytes)
 */
void hmac_to_hex_string(const uint8_t* hmac_bin, char* hmac_hex);

/**
 * @brief Get next sequence number for messages
 * @return Current sequence number (increments after call)
 */
uint32_t get_next_sequence_number();

#endif // HMAC_AUTH_H