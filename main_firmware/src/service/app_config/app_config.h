#pragma once

/**
 * app_config.h - Application Configuration Schema Registration
 * 
 * Registers application configuration keys used by protocol, input, LEDs, bindings, and keyboard mappings.
 */

/**
 * Initialize and register all application configuration schema.
 * Must be called BEFORE ConfigManager::initialize().
 * Idempotent - safe to call multiple times.
 */
void app_config_register_schema();
