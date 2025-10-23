#pragma once

#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>
#include <algorithm>
#include <cctype>
#include <limits>

// Mathematical constants
namespace ZoneConstants {
    constexpr float TAU = 2.0f * M_PI;  // 6.283185...
    constexpr float EPSILON = 1e-6f;     // For floating point comparisons
    constexpr int16_t MAX_COORDINATE = 4000;
    constexpr int16_t MAX_DISTANCE = 8000;
    constexpr int16_t MIN_Y = -500;
}

// Position data structure for detected targets
struct Position {
    int16_t x = 0; 
    int16_t y = 0;
    int16_t speed = 0;
    int16_t distance_resolution = 0;
    bool valid = false;  // Changed from 'valide' for correct English
    bool zone_ex_enter = false;
    float angle = 0.0f;
    std::string position = "Static";
    std::string direction = "None";
    
    // Calculate distance from origin
    float getDistance() const {
        return std::sqrt(static_cast<float>(x * x + y * y));
    }
    
    // Check if position is within valid bounds
    bool isWithinBounds() const {
        return (x >= -ZoneConstants::MAX_COORDINATE && 
                x <= ZoneConstants::MAX_COORDINATE &&
                y >= ZoneConstants::MIN_Y && 
                y <= ZoneConstants::MAX_DISTANCE);
    }
    
    // Reset to default state
    void reset() {
        x = 0;
        y = 0;
        speed = 0;
        distance_resolution = 0;
        valid = false;
        zone_ex_enter = false;
        angle = 0.0f;
        position = "Static";
        direction = "None";
    }
};

// Zone definition structure
struct Zone {
    int16_t x = 0;
    int16_t y = 0;
    int16_t height = 0;
    int16_t width = 0;
    int16_t target_count = 0;
    int16_t outside_target_count = 0;
    bool has_target = false;
    bool has_target_outside = false;
    
    // Check if zone configuration is valid
    bool isValid() const {
        return (width > 0 && height > 0 &&
                x >= -ZoneConstants::MAX_COORDINATE && 
                x <= ZoneConstants::MAX_COORDINATE &&
                y >= ZoneConstants::MIN_Y && 
                y <= ZoneConstants::MAX_DISTANCE);
    }
    
    // Get zone area
    int32_t getArea() const {
        return static_cast<int32_t>(width) * static_cast<int32_t>(height);
    }
    
    // Reset zone counts
    void resetCounts() {
        target_count = 0;
        outside_target_count = 0;
        has_target = false;
        has_target_outside = false;
    }
    
    // Check if zone is configured (non-zero dimensions)
    bool isConfigured() const {
        return (width != 0 || height != 0);
    }
};

// Point structure for zone corner calculations
struct Pxy {
    float x = 0.0f;  // Use float for better precision in calculations
    float y = 0.0f;
    
    Pxy() = default;
    Pxy(float px, float py) : x(px), y(py) {}
    
    // Calculate distance to another point
    float distanceTo(const Pxy& other) const {
        float dx = x - other.x;
        float dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    // Calculate distance to a position
    float distanceTo(const Position& pos) const {
        float dx = x - pos.x;
        float dy = y - pos.y;
        return std::sqrt(dx * dx + dy * dy);
    }
};

// Utility Functions

// Convert degrees to radians (inline for performance)
inline float toRadians(float degrees) {
    return degrees * M_PI / 180.0f;
}

// Convert radians to degrees
inline float toDegrees(float radians) {
    return radians * 180.0f / M_PI;
}

// Safe arc cosine calculation with bounds checking
inline float safeAcos(float value) {
    // Clamp value to [-1, 1] to avoid NaN from acos
    if (value > 1.0f) return 0.0f;
    if (value < -1.0f) return M_PI;
    return std::acos(value);
}

// Calculate zone corners with rotation
struct ZoneCorners {
    Pxy p1, p2, p3, p4;
    
    ZoneCorners(const Zone& z, float angle_deg) {
        float angle_rad = toRadians(angle_deg);
        float cos_angle = std::cos(angle_rad);
        float sin_angle = std::sin(angle_rad);
        float cos_angle_90 = std::cos(toRadians(angle_deg - 90.0f));
        float sin_angle_90 = std::sin(toRadians(angle_deg + 90.0f));
        
        // Corner 1: Origin point
        p1.x = z.x;
        p1.y = z.y;
        
        // Corner 2: Width direction
        p2.x = z.x - z.width * cos_angle;
        p2.y = z.y + z.width * sin_angle;
        
        // Corner 3: Opposite corner (width + height)
        p3.x = z.x - z.width * cos_angle + z.height * cos_angle_90;
        p3.y = z.y + z.width * sin_angle + z.height * sin_angle_90;
        
        // Corner 4: Height direction
        p4.x = z.x + z.height * cos_angle_90;
        p4.y = z.y + z.height * sin_angle_90;
    }
};

/**
 * Check if a target is inside a zone using the sum of angles method
 * This method calculates the sum of angles from the target to each corner
 * If the sum equals 2π (TAU), the point is inside the quadrilateral
 * 
 * @param z The zone to check
 * @param t The target position
 * @param angle The rotation angle of the zone in degrees
 * @return true if target is inside the zone
 */
bool check_targets_in_zone(const Zone& z, const Position& t, float angle) {
    // Quick validation checks
    if (!z.isValid() || !t.valid) {
        return false;
    }
    
    // Skip if target is excluded
    if (t.zone_ex_enter) {
        return false;
    }
    
    // Calculate zone corners
    ZoneCorners corners(z, angle);
    Pxy target(t.x, t.y);
    
    // Calculate distances from target to each corner
    float d15 = corners.p1.distanceTo(target);
    float d25 = corners.p2.distanceTo(target);
    float d35 = corners.p3.distanceTo(target);
    float d45 = corners.p4.distanceTo(target);
    
    // Quick rejection: if target is too far from all corners, it's outside
    float max_zone_diagonal = std::sqrt(z.width * z.width + z.height * z.height);
    if (d15 > max_zone_diagonal && d25 > max_zone_diagonal && 
        d35 > max_zone_diagonal && d45 > max_zone_diagonal) {
        return false;
    }
    
    // Calculate distances between corners
    float d12 = corners.p1.distanceTo(corners.p2);
    float d14 = corners.p1.distanceTo(corners.p4);
    float d23 = corners.p2.distanceTo(corners.p3);
    float d34 = corners.p3.distanceTo(corners.p4);
    
    // Check for degenerate cases (zero distances)
    if (d15 < ZoneConstants::EPSILON || d25 < ZoneConstants::EPSILON ||
        d35 < ZoneConstants::EPSILON || d45 < ZoneConstants::EPSILON) {
        return true; // Target is essentially on a corner
    }
    
    // Calculate angles using law of cosines with safety checks
    float cos_a152 = (d15*d15 + d25*d25 - d12*d12) / (2.0f * d15 * d25);
    float cos_a154 = (d15*d15 + d45*d45 - d14*d14) / (2.0f * d15 * d45);
    float cos_a253 = (d25*d25 + d35*d35 - d23*d23) / (2.0f * d25 * d35);
    float cos_a354 = (d35*d35 + d45*d45 - d34*d34) / (2.0f * d35 * d45);
    
    float a152 = safeAcos(cos_a152);
    float a154 = safeAcos(cos_a154);
    float a253 = safeAcos(cos_a253);
    float a354 = safeAcos(cos_a354);
    
    // Sum of angles
    float a_sum = a152 + a154 + a253 + a354;
    
    // Check if sum is approximately 2π (with small tolerance)
    return (a_sum >= (ZoneConstants::TAU - 0.01f));
}

/**
 * Alternative fast rectangular zone check (no rotation)
 * Use this for axis-aligned zones for better performance
 */
bool check_targets_in_rect_zone(const Zone& z, const Position& t) {
    if (!z.isValid() || !t.valid || t.zone_ex_enter) {
        return false;
    }
    
    int16_t x_min = std::min(z.x, static_cast<int16_t>(z.x - z.width));
    int16_t x_max = std::max(z.x, static_cast<int16_t>(z.x - z.width));
    int16_t y_min = z.y;
    int16_t y_max = z.y + z.height;
    
    return (t.x >= x_min && t.x <= x_max && t.y >= y_min && t.y <= y_max);
}

/**
 * Convert string to boolean (case-insensitive)
 */
bool to_bool(const std::string& str) {
    if (str.empty()) return false;
    
    std::string lower_str = str;
    std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(), 
                   [](unsigned char c){ return std::tolower(c); });
    
    return (lower_str == "true" || lower_str == "1" || 
            lower_str == "yes" || lower_str == "on");
}

/**
 * Validate and update zone info text
 */
void check_zone_valid(int x, int y, int width, int height, 
                      template_::TemplateTextSensor* tips_conf) {
    if (!tips_conf) return;
    
    // Check if zone is unconfigured
    if (x == 0 && width == 0 && y == 0 && height == 0) {
        tips_conf->publish_state("Configure below");
        return;
    }
    
    // Validate boundaries
    bool valid = true;
    std::string status;
    
    if (width <= 0 || height <= 0) {
        status = "Invalid: Width/Height must be > 0";
        valid = false;
    } else if (abs(x) > ZoneConstants::MAX_COORDINATE) {
        status = "Invalid: X out of bounds";
        valid = false;
    } else if (y < ZoneConstants::MIN_Y || y > ZoneConstants::MAX_DISTANCE) {
        status = "Invalid: Y out of bounds";
        valid = false;
    } else if (abs(x) + width > ZoneConstants::MAX_COORDINATE) {
        status = "Warning: Zone extends beyond X boundary";
        valid = true; // Warning, not error
    } else if (y + height > ZoneConstants::MAX_DISTANCE) {
        status = "Warning: Zone extends beyond Y boundary";
        valid = true; // Warning, not error
    } else {
        // Calculate zone area in square meters for reference
        float area_m2 = (width * height) / 1000000.0f;
        char buffer[80];
        snprintf(buffer, sizeof(buffer), "Size: %dx%d mm (%.2f m²)", 
                 width, height, area_m2);
        status = buffer;
    }
    
    tips_conf->publish_state(status.c_str());
}

/**
 * Validate and update exclusion zone info text
 */
void check_zout_valid(int zone_num, template_::TemplateTextSensor* tips_conf) {
    if (!tips_conf) return;
    
    char buffer[50];
    snprintf(buffer, sizeof(buffer), "Exclusion Zone %d", zone_num);
    tips_conf->publish_state(buffer);
}

/**
 * Calculate if target is approaching or moving away
 */
std::string calculate_target_position(int16_t speed, float speed_threshold = 0.05f) {
    float speed_ms = speed / 100.0f; // Convert to m/s
    
    if (speed_ms > speed_threshold) {
        return "Moving away";
    } else if (speed_ms < -speed_threshold) {
        return "Approaching";
    } else {
        return "Static";
    }
}

/**
 * Calculate target direction based on X coordinate
 */
std::string calculate_target_direction(int16_t x, int16_t y, int16_t threshold = 100) {
    if (x > threshold) {
        return "Right";
    } else if (x < -threshold) {
        return "Left";
    } else if (y > 0) {
        return "Center";
    } else {
        return "None";
    }
}

/**
 * Calculate angle from Y-axis (sensor forward direction)
 */
float calculate_target_angle(int16_t x, int16_t y) {
    if (y == 0) return 0.0f;
    
    // Calculate angle from Y-axis (forward direction)
    float angle_rad = std::atan2(static_cast<float>(x), static_cast<float>(y));
    return toDegrees(angle_rad);
}

/**
 * Dump zone configuration for debugging
 */
void debug_print_zone(const Zone& z, int zone_num) {
    ESP_LOGD("zone", "Zone %d: x=%d, y=%d, w=%d, h=%d, targets=%d, valid=%d",
             zone_num, z.x, z.y, z.width, z.height, z.target_count, z.isValid());
}

/**
 * Dump target information for debugging
 */
void debug_print_target(const Position& p, int target_num) {
    ESP_LOGD("target", "Target %d: x=%d, y=%d, speed=%d, angle=%.1f°, pos=%s, dir=%s, valid=%d",
             target_num, p.x, p.y, p.speed, p.angle, 
             p.position.c_str(), p.direction.c_str(), p.valid);
}
