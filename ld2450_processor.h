#pragma once
#include "zone.h"

void process_ld2450_data(
    const std::vector<uint8_t>& bytes,
    unsigned long& last_update,
    unsigned long& update_counter,
    unsigned long& last_rate_calc,
    unsigned long& packet_error_count,
    bool& init_zone_publish,
    template_::TemplateNumber* update_interval_ms,
    template_::TemplateNumber* position_threshold,
    template_::TemplateNumber* speed_threshold,
    template_::TemplateNumber* wall_angle,
    template_::TemplateSensor* update_rate,
    template_::TemplateSensor* packet_errors,
    template_::TemplateTextSensor* radar_status,
    template_::TemplateSwitch* zone_fn_enable,
    template_::TemplateSwitch* target_fn_enable,
    template_::TemplateSwitch* debug_mode,
    template_::TemplateNumber* zone_x[3],
    template_::TemplateNumber* zone_y[3],
    template_::TemplateNumber* zone_height[3],
    template_::TemplateNumber* zone_width[3],
    template_::TemplateSensor* zone_target_count[3],
    template_::TemplateBinarySensor* zone_target_exist[3],
    template_::TemplateSwitch* zone_ex_enable[1],
    template_::TemplateNumber* zone_ex_x[1],
    template_::TemplateNumber* zone_ex_y[1],
    template_::TemplateNumber* zone_ex_height[1],
    template_::TemplateNumber* zone_ex_width[1],
    template_::TemplateSensor* zone_ex_target_count[1],
    template_::TemplateBinarySensor* zone_ex_target_exist[1],
    template_::TemplateSensor* target_angle[3],
    template_::TemplateTextSensor* target_position[3],
    template_::TemplateTextSensor* target_direction[3],
    template_::TemplateSensor* target_x[3],
    template_::TemplateSensor* target_y[3],
    template_::TemplateSensor* target_speed[3],
    template_::TemplateSensor* target_resolution[3],
    template_::TemplateSensor* all_target_count,
    template_::TemplateBinarySensor* any_target_exist
) {
    const int NUM_ZONES = 3;
    const int NUM_ZONES_EX = 1;
    const int NUM_TARGETS = 3;
    const int MIN_PACKET_SIZE = 30;
    const int EXPECTED_PACKET_SIZE = 48;
    static std::vector<uint8_t> packet_buffer;

    // Append new bytes to buffer
    packet_buffer.insert(packet_buffer.end(), bytes.begin(), bytes.end());

    // Only process if we have at least a full packet
    const int FULL_PACKET_SIZE = 48;
    if (packet_buffer.size() < FULL_PACKET_SIZE) {
        // Not enough data yet; exit and wait for next update
        return;
    }

    // Take first full packet for processing
    std::vector<uint8_t> packet(packet_buffer.begin(), packet_buffer.begin() + FULL_PACKET_SIZE);

    // Remove the processed bytes from the buffer
    packet_buffer.erase(packet_buffer.begin(), packet_buffer.begin() + FULL_PACKET_SIZE);

    unsigned long current_time = millis();
    if ((current_time - last_update) <= update_interval_ms->state) { 
        return;
    }
    last_update = current_time;
    
    update_counter = update_counter + 1;
    if ((current_time - last_rate_calc) >= 1000) {
        float rate = update_counter * 1000.0 / (current_time - last_rate_calc);
        update_rate->publish_state(rate);
        update_counter = 0;
        last_rate_calc = current_time;
    }

    if (bytes.size() < MIN_PACKET_SIZE) {
        packet_error_count = packet_error_count + 1;
        packet_errors->publish_state(packet_error_count);
        radar_status->publish_state("Packet Error");
        ESP_LOGW("ld2450", "Invalid packet size: %d", bytes.size());
        return;
    }
    
    float angle = wall_angle->state;
    float pos_threshold = position_threshold->state;
    float speed_thresh = speed_threshold->state;
    
    struct Position p[NUM_TARGETS];
    struct Zone zone_ex[NUM_ZONES_EX], zone[NUM_ZONES];
    
    // Parse target data
    int b = 0;
    for (int i = 0; i < NUM_TARGETS; i++) {
        // Parse X coordinate
        p[i].x = (uint16_t((bytes[b+5] << 8) | bytes[b+4]));
        if ((bytes[b+5] & 0x80) >> 7) {
            p[i].x -= 32768;
        } else {
            p[i].x = 0 - p[i].x;
        }
        p[i].x = p[i].x * -1;
    
        // Parse Y coordinate
        p[i].y = (uint16_t((bytes[b+7] << 8) | bytes[b+6]));
        if ((bytes[b+7] & 0x80) >> 7) {
            p[i].y -= 32768;
        } else {
            p[i].y = 0 - p[i].y;
        }
    
        // Parse speed
        p[i].speed = (bytes[b+9] << 8 | bytes[b+8]);
        if ((bytes[b+9] & 0x80) >> 7) {
            p[i].speed -= 32768;
        } else {
            p[i].speed = 0 - p[i].speed;
        }
        
        // Parse distance resolution - THIS IS UNSIGNED, NO SIGN CONVERSION
        p[i].distance_resolution = (uint16_t((bytes[b+11] << 8) | bytes[b+10]));
        
        // Target is valid if it has non-zero coordinates or positive Y
        p[i].valid = (p[i].x != 0 || p[i].y > 0);
        
        // Reset exclusion flag
        p[i].zone_ex_enter = false;
        
        b += 8; // Move to next target data block
    }
    
    // Process exclusion zones
    for (int i = 0; i < NUM_ZONES_EX; i++) {
        zone_ex[i].x = zone_ex_x[i]->state;
        zone_ex[i].y = zone_ex_y[i]->state;
        zone_ex[i].width = zone_ex_width[i]->state;
        zone_ex[i].height = zone_ex_height[i]->state;
        zone_ex[i].resetCounts();

        if (zone_ex_enable[i]->state && zone_ex[i].isConfigured()) {
            for (int j = 0; j < NUM_TARGETS; j++) {
                if (p[j].valid) {
                    if (check_targets_in_zone(zone_ex[i], p[j], angle)) {
                        zone_ex[i].target_count++;
                        p[j].zone_ex_enter = true;
                    } else { 
                        zone_ex[i].outside_target_count++;
                    }
                }
            }
        }
        zone_ex[i].has_target = (zone_ex[i].target_count > 0);
        zone_ex[i].has_target_outside = (zone_ex[i].outside_target_count > 0);
    }
    
    // Count all valid targets
    int16_t all_target_counts = 0;
    for (int i = 0; i < NUM_TARGETS; i++) {
        if (p[i].valid && !p[i].zone_ex_enter) {
            all_target_counts++;
        }
    }
    bool has_target_in_zone_all = (all_target_counts > 0);

    // Process detection zones
    if (zone_fn_enable->state) {
        for (int i = 0; i < NUM_ZONES; i++) {
            zone[i].x = zone_x[i]->state;
            zone[i].y = zone_y[i]->state;
            zone[i].width = zone_width[i]->state;
            zone[i].height = zone_height[i]->state;
            zone[i].resetCounts();
            
            if (zone[i].isConfigured()) {
                for (int j = 0; j < NUM_TARGETS; j++) {
                    if (p[j].valid && !p[j].zone_ex_enter) {
                        if (check_targets_in_zone(zone[i], p[j], angle)) {
                            zone[i].target_count++;
                        } else { 
                            zone[i].outside_target_count++;
                        }
                    }
                }
            }
            zone[i].has_target = (zone[i].target_count > 0);
            zone[i].has_target_outside = (zone[i].outside_target_count > 0);
        }
    }

    // Calculate target attributes
    for (int i = 0; i < NUM_TARGETS; i++) {
        if (p[i].valid) {
            p[i].angle = calculate_target_angle(p[i].x, p[i].y);
            p[i].position = calculate_target_position(p[i].speed, speed_thresh);
            p[i].direction = calculate_target_direction(p[i].x, p[i].y, 100);
        }
    }

    // Publish target data
    if (id(target_fn_enable).state) {
        for (int i = 0; i < NUM_TARGETS; i++) {
            if (p[i].valid) {
                // Always publish for valid targets (for debugging)
                target_x[i]->publish_state(p[i].x);
                target_y[i]->publish_state(p[i].y);
                
                float speed_ms = p[i].speed / 100.0f;
                target_speed[i]->publish_state(speed_ms);
                
                target_resolution[i]->publish_state(p[i].distance_resolution);
                target_angle[i]->publish_state(p[i].angle);
                target_position[i]->publish_state(p[i].position);
                target_direction[i]->publish_state(p[i].direction);
            } else {
                // Publish 0 for invalid targets
                target_x[i]->publish_state(0);
                target_y[i]->publish_state(0);
                target_speed[i]->publish_state(0);
                target_angle[i]->publish_state(0);
            }
        }
    }
    
    ESP_LOGD("ld2450", "T1: valid=%d x=%d y=%d speed=%d", p[0].valid, p[0].x, p[0].y, p[0].speed);
    ESP_LOGD("ld2450", "T2: valid=%d x=%d y=%d speed=%d", p[1].valid, p[1].x, p[1].y, p[1].speed);
    ESP_LOGD("ld2450", "T3: valid=%d x=%d y=%d speed=%d", p[2].valid, p[2].x, p[2].y, p[2].speed);

    // Publish overall presence
    if (all_target_count->state != all_target_counts) {
        all_target_count->publish_state(all_target_counts);
        any_target_exist->publish_state(has_target_in_zone_all);
    } else if (any_target_exist->state != has_target_in_zone_all) {
        any_target_exist->publish_state(has_target_in_zone_all);
    }

    // Publish zone data
    for (int i = 0; i < NUM_ZONES; i++) {
        if (zone_target_count[i]->state != zone[i].target_count) {
            zone_target_count[i]->publish_state(zone[i].target_count);
            zone_target_exist[i]->publish_state(zone[i].has_target);
        }
    }
    
    // Publish exclusion zone data
    for (int i = 0; i < NUM_ZONES_EX; i++) {
        if (zone_ex_target_count[i]->state != zone_ex[i].target_count) {
            zone_ex_target_count[i]->publish_state(zone_ex[i].target_count);
        }
        if (zone_ex_target_exist[i]->state != zone_ex[i].has_target) {
            zone_ex_target_exist[i]->publish_state(zone_ex[i].has_target);
        }
    }

    // Initialization
    if (!init_zone_publish) {
        init_zone_publish = true;
        radar_status->publish_state("Ready");
        ESP_LOGI("ld2450", "Radar initialized and publishing");
    }
    
    // Debug logging
    if (debug_mode->state) {
        for (int i = 0; i < NUM_TARGETS; i++) {
            if (p[i].valid) {
                debug_print_target(p[i], i + 1);
            }
        }
        for (int i = 0; i < NUM_ZONES; i++) {
            if (zone[i].isConfigured()) {
                debug_print_zone(zone[i], i + 1);
            }
        }
    }
}