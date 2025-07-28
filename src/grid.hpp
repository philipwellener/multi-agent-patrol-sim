#pragma once

#include "utils.hpp"
#include "config.hpp"
#include <vector>
#include <unordered_set>
#include <memory>

// Forward declaration
class Agent;

// Use the GridCoord from Utils namespace
using Utils::GridCoord;

enum class CellType {
    EMPTY,
    OBSTACLE,
    WAYPOINT,
    AGENT
};

class Grid {
private:
    int width_, height_;
    std::vector<std::vector<CellType>> cells_;
    std::vector<GridCoord> waypoints_;
    std::vector<GridCoord> obstacles_;
    std::unordered_set<std::string> agent_positions_;  // Track agent positions for collision avoidance
    Utils::RandomGenerator random_generator_;
    
    // Convert grid coordinates to string for hash set
    std::string coordToString(const GridCoord& coord) const;
    GridCoord stringToCoord(const std::string& str) const;
    
public:
    Grid(int width = Config::GRID_WIDTH, int height = Config::GRID_HEIGHT);
    ~Grid() = default;
    
    // Grid dimensions
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    
    // Cell operations
    CellType getCellType(const GridCoord& coord) const;
    void setCellType(const GridCoord& coord, CellType type);
    bool isObstacle(const GridCoord& coord) const;
    bool isEmpty(const GridCoord& coord) const;
    bool isValidCoord(const GridCoord& coord) const;
    
    // Waypoint management
    void addWaypoint(const GridCoord& coord);
    void removeWaypoint(const GridCoord& coord);
    const std::vector<GridCoord>& getWaypoints() const { return waypoints_; }
    bool isWaypoint(const GridCoord& coord) const;
    GridCoord getNearestWaypoint(const GridCoord& from) const;
    GridCoord getRandomWaypoint() const;
    
    // Obstacle management
    void addObstacle(const GridCoord& coord);
    void removeObstacle(const GridCoord& coord);
    const std::vector<GridCoord>& getObstacles() const { return obstacles_; }
    void generateRandomObstacles(double density = Config::OBSTACLE_DENSITY);
    void clearObstacles();
    
    // Agent position tracking (for collision avoidance)
    void addAgentPosition(const GridCoord& coord);
    void removeAgentPosition(const GridCoord& coord);
    void updateAgentPosition(const GridCoord& oldPos, const GridCoord& newPos);
    bool isOccupiedByAgent(const GridCoord& coord) const;
    
    // Pathfinding support
    bool isTraversable(const GridCoord& coord) const;
    std::vector<GridCoord> getTraversableNeighbors(const GridCoord& coord) const;
    double getMovementCost(const GridCoord& from, const GridCoord& to) const;
    bool isDiagonalMovementValid(const GridCoord& from, const GridCoord& to) const;
    
    // Environment generation
    void generateEnvironment();
    void generateWaypoints(int numWaypoints = Config::NUM_PATROL_WAYPOINTS);
    
    // Visualization
    void display() const;
    void displayWithAgents(const std::vector<std::shared_ptr<Agent>>& agents) const;
    std::string toString() const;
    std::string toStringWithAgents(const std::vector<std::shared_ptr<Agent>>& agents) const;
    
    // Utility functions
    void clear();
    void reset();
    GridCoord getRandomEmptyCell() const;
    std::vector<GridCoord> getEmptyCells() const;
    
    // Statistics
    double getObstacleDensity() const;
    int getObstacleCount() const;
    int getWaypointCount() const;
};
