#pragma once

namespace Config {
    // Compact grid size - fits better in window while still challenging
    constexpr int GRID_WIDTH = 35;
    constexpr int GRID_HEIGHT = 18;  // Reduced from 25 to 18 for better fit
    
    constexpr int NUM_AGENTS = 4;  // Reduced for better performance
    constexpr double AGENT_SPEED = 1.0;  // cells per second
    constexpr int AGENT_VISION_RANGE = 3;  // cells
    
    constexpr double SIMULATION_TIMESTEP = 0.1;  // seconds
    constexpr int MAX_SIMULATION_STEPS = 1000;  // increased for longer simulation
    constexpr bool ENABLE_VISUALIZATION = true;
    constexpr int VISUALIZATION_DELAY_MS = 100;  // Faster visualization
    
    constexpr double OBSTACLE_DENSITY = 0.15;  // Reduced slightly for better navigation
    constexpr int NUM_PATROL_WAYPOINTS = 5;  // Reduced for smaller grid
    
    // A* stuff - increased limits for larger grid
    constexpr double HEURISTIC_WEIGHT = 1.0;  
    constexpr int MAX_PATH_LENGTH = 250;  // Increased for longer paths on larger grid
    
    constexpr double WAYPOINT_REACHED_THRESHOLD = 1.2;  // Slightly more forgiving threshold
    constexpr double WAYPOINT_PAUSE_DURATION = 0.3;    // Shorter pause to reduce stuck time
    constexpr double OBSTACLE_DETECTION_RANGE = 2.0;
    constexpr double COLLISION_AVOIDANCE_RANGE = 1.0;  // Further reduced to minimize deadlocks
    
    // Debug options
    constexpr bool DEBUG_PATHFINDING = true;
    constexpr bool DEBUG_BEHAVIOR_TREE = true;
    constexpr bool DEBUG_AGENT_DECISIONS = true;
}
