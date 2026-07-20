#pragma once

/**
 * app_config.h - Application Configuration Schema Registration
 * 
 * Registers all 34 configuration keys per protocol_design.md Revision 2 (C5).
 * Keys registered:
 *   - comm.* (9 keys: communication settings)
 *   - mode.work (1 key: work mode)
 *   - led.* (3 keys: LED settings)
 *   - bind.map00..bind.map33 (34 keys: binding area to channel mapping)
 * Total: 47 keys
 */

/**
 * Initialize and register all application configuration schema.
 * Must be called BEFORE ConfigManager::initialize().
 * Idempotent - safe to call multiple times.
 */
void app_config_register_schema();
