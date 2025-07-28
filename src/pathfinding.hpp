#pragma once

#include "utils.hpp"
#include "grid.hpp"
#include <vector>
#include <queue>
#include <unordered_map>
#include <functional>

// Forward declaration
class Grid;

// Use the GridCoord from Utils namespace
using Utils::GridCoord;

struct PathNode {
    GridCoord position;
    double gCost;  // Cost from start
    double hCost;  // Heuristic cost to goal
    double fCost;  // Total cost (g + h)
    GridCoord parent;
    bool hasParent;
    
    PathNode(const GridCoord& pos = GridCoord(), double g = 0.0, double h = 0.0)
        : position(pos), gCost(g), hCost(h), fCost(g + h), parent(), hasParent(false) {}
    
    bool operator>(const PathNode& other) const {
        return fCost > other.fCost;
    }
};

class Pathfinder {
private:
    const Grid* grid_;
    std::unordered_map<std::string, PathNode> nodes_;
    std::priority_queue<PathNode, std::vector<PathNode>, std::greater<PathNode>> openSet_;
    std::unordered_map<std::string, bool> closedSet_;
    
    // Helper functions
    std::string coordToString(const GridCoord& coord) const;
    double calculateHeuristic(const GridCoord& from, const GridCoord& to) const;
    std::vector<GridCoord> reconstructPath(const GridCoord& start, const GridCoord& goal) const;
    void reset();
    
public:
    explicit Pathfinder(const Grid* grid);
    ~Pathfinder() = default;
    
    // Main pathfinding functions
    std::vector<GridCoord> findPath(const GridCoord& start, const GridCoord& goal);
    std::vector<GridCoord> findPath(const GridCoord& start, const GridCoord& goal, 
                                  const std::vector<GridCoord>& temporaryObstacles);
    
    // Pathfinding with agent avoidance
    std::vector<GridCoord> findPathAvoidingAgents(const GridCoord& start, const GridCoord& goal,
                                                const std::vector<GridCoord>& agentPositions);
    
    // Path optimization
    std::vector<GridCoord> smoothPath(const std::vector<GridCoord>& path) const;
    std::vector<GridCoord> simplifyPath(const std::vector<GridCoord>& path) const;
    
    // Path validation
    bool isPathValid(const std::vector<GridCoord>& path) const;
    bool isLineOfSightClear(const GridCoord& start, const GridCoord& end) const;
    
    // Utility functions
    double getPathLength(const std::vector<GridCoord>& path) const;
    GridCoord getNextStep(const GridCoord& start, const GridCoord& goal);
    
    // Advanced pathfinding
    std::vector<GridCoord> findAlternatePath(const GridCoord& start, const GridCoord& goal,
                                           const std::vector<GridCoord>& avoidPath) const;
    
    // Multi-goal pathfinding
    std::vector<GridCoord> findNearestGoal(const GridCoord& start, 
                                         const std::vector<GridCoord>& goals);
    GridCoord selectBestGoal(const GridCoord& start, 
                           const std::vector<GridCoord>& goals);
    
    // Debug functions
    void printPath(const std::vector<GridCoord>& path) const;
    std::string pathToString(const std::vector<GridCoord>& path) const;
};
