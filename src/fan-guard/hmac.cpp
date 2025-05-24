#include "hmac.h"
#include "secrets.h" // Contains HMAC_KEY
#include <string.h>
#include <Preferences.h>
#include "shared-defs.h"
#include "secrets.h"

// Global state variables
uint32_t g_session_id = 0;           // Stored in flash memory
uint32_t g_sequence_number = 0;

// Preferences instance for non-volatile storage
Preferences preferences;

void hmac_auth_init() {
    // Open preferences in read/write mode
    preferences.begin(PREF_NAMESPACE, false);
    
    // Check if this is wakeup from deep sleep
    // Increment session ID on fresh boot only
    uint32_t stored_session_id = preferences.getUInt(PREF_SESSION_ID);
    stored_session_id++; // Increment the counter
    preferences.putUInt(PREF_SESSION_ID, stored_session_id);
    
    // Reset sequence number for new session
    g_sequence_number = 0;
    
    Serial.printf("[AUTH] Fresh boot detected. Incremented session ID to: %u\n", stored_session_id);
    
    // Load the current session ID value into memory
    g_session_id = preferences.getUInt(PREF_SESSION_ID);
    
    // Close preferences
    preferences.end();
    
    Serial.printf("[AUTH] Current session ID: %u\n", g_session_id);
}

bool calculate_hmac(const char* payload, size_t payload_len, 
                   auth_data_t* auth, uint8_t* hmac_result) {
    // Check parameters
    if (!payload || !auth || !hmac_result) {
        return false;
    }

    // Create message buffer with authentication data + payload
    // Format: session_id (4 bytes) + seq_number (4 bytes) + timestamp (8 bytes) + payload
    size_t auth_size = sizeof(uint32_t) * 2 + sizeof(uint64_t);
    size_t total_len = auth_size + payload_len;
    uint8_t* message = (uint8_t*)malloc(total_len);
    
    if (!message) {
        Serial.println("[AUTH] Memory allocation failed");
        return false;
    }
    
    // Populate the message buffer
    uint8_t* ptr = message;
    
    // Add session ID
    memcpy(ptr, &auth->session_id, sizeof(uint32_t));
    ptr += sizeof(uint32_t);
    
    // Add sequence number
    memcpy(ptr, &auth->seq_number, sizeof(uint32_t));
    ptr += sizeof(uint32_t);
    
    // Add timestamp
    memcpy(ptr, &auth->timestamp, sizeof(uint64_t));
    ptr += sizeof(uint64_t);
    
    // Add payload
    memcpy(ptr, payload, payload_len);
    
    // Initialize mbedTLS context
    mbedtls_md_context_t ctx;
    mbedtls_md_type_t md_type = MBEDTLS_MD_SHA256;
    const mbedtls_md_info_t *md_info = mbedtls_md_info_from_type(md_type);
    
    mbedtls_md_init(&ctx);
    int ret = mbedtls_md_setup(&ctx, md_info, 1); // 1 for HMAC (not just hash)
    
    if (ret != 0) {
        Serial.printf("[AUTH] mbedTLS setup failed: %d\n", ret);
        free(message);
        return false;
    }
    
    // Compute HMAC
    ret = mbedtls_md_hmac_starts(&ctx, HMAC_KEY, HMAC_KEY_LENGTH);
    if (ret == 0) {
        ret = mbedtls_md_hmac_update(&ctx, message, total_len);
    }
    if (ret == 0) {
        ret = mbedtls_md_hmac_finish(&ctx, hmac_result);
    }
    
    // Cleanup
    mbedtls_md_free(&ctx);
    free(message);
    
    return (ret == 0);
}

void hmac_to_hex_string(const uint8_t* hmac_bin, char* hmac_hex) {
    for (int i = 0; i < HMAC_OUTPUT_LENGTH; i++) {
        sprintf(&hmac_hex[i * 2], "%02x", hmac_bin[i]);
    }
    hmac_hex[HMAC_OUTPUT_LENGTH * 2] = '\0';
}

uint32_t get_next_sequence_number() {
    return g_sequence_number++;
}