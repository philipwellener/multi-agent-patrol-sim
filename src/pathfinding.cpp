#include "pathfinding.hpp"
#include "config.hpp"
#include <algorithm>
#include <iostream>
#include <sstream>
#include <cmath>
#include <limits>
#include <iomanip>

Pathfinder::Pathfinder(const Grid* grid) : grid_(grid) {}

std::string Pathfinder::coordToString(const GridCoord& coord) const {
    return std::to_string(coord.x) + "," + std::to_string(coord.y);
}

double Pathfinder::calculateHeuristic(const GridCoord& from, const GridCoord& to) const {
    // Using Manhattan distance with slight diagonal allowance
    double dx = std::abs(from.x - to.x);
    double dy = std::abs(from.y - to.y);
    
    // Euclidean distance for more natural paths
    return std::sqrt(dx * dx + dy * dy) * Config::HEURISTIC_WEIGHT;
}

void Pathfinder::reset() {
    nodes_.clear();
    while (!openSet_.empty()) {
        openSet_.pop();
    }
    closedSet_.clear();
}

std::vector<GridCoord> Pathfinder::findPath(const GridCoord& start, const GridCoord& goal) {
    if (!grid_ || !grid_->isTraversable(start) || !grid_->isTraversable(goal)) {
        return {};
    }
    
    if (start == goal) {
        return {start};
    }
    
    reset();
    
    // Initialize start node
    PathNode startNode(start, 0.0, calculateHeuristic(start, goal));
    nodes_[coordToString(start)] = startNode;
    openSet_.push(startNode);
    
    while (!openSet_.empty()) {
        // Get node with lowest f-cost
        PathNode current = openSet_.top();
        openSet_.pop();
        
        std::string currentKey = coordToString(current.position);
        
        // Skip if already processed
        if (closedSet_[currentKey]) {
            continue;
        }
        
        closedSet_[currentKey] = true;
        
        // Check if we reached the goal
        if (current.position == goal) {
            return reconstructPath(start, goal);
        }
        
        // Explore neighbors
        std::vector<GridCoord> neighbors = grid_->getTraversableNeighbors(current.position);
        
        for (const auto& neighbor : neighbors) {
            std::string neighborKey = coordToString(neighbor);
            
            // Skip if already processed
            if (closedSet_[neighborKey]) {
                continue;
            }
            
            double movementCost = grid_->getMovementCost(current.position, neighbor);
            double tentativeGCost = current.gCost + movementCost;
            
            // Check if this path to neighbor is better
            auto it = nodes_.find(neighborKey);
            bool isNewNode = (it == nodes_.end());
            
            if (isNewNode || tentativeGCost < it->second.gCost) {
                PathNode neighborNode(neighbor, tentativeGCost, calculateHeuristic(neighbor, goal));
                neighborNode.parent = current.position;
                neighborNode.hasParent = true;
                
                nodes_[neighborKey] = neighborNode;
                openSet_.push(neighborNode);
            }
        }
    }
    
    // No path found
    return {};
}

std::vector<GridCoord> Pathfinder::findPath(const GridCoord& start, const GridCoord& goal,
                                          const std::vector<GridCoord>& temporaryObstacles) {
    // Create a temporary grid state with additional obstacles
    // For simplicity, we'll modify the pathfinding to check against the temporary obstacles
    // In a more complex implementation, you might create a temporary grid copy
    
    if (!grid_ || !grid_->isTraversable(start) || !grid_->isTraversable(goal)) {
        return {};
    }
    
    // Check if start or goal are in temporary obstacles
    for (const auto& obstacle : temporaryObstacles) {
        if (start == obstacle || goal == obstacle) {
            return {};
        }
    }
    
    if (start == goal) {
        return {start};
    }
    
    reset();
    
    // Initialize start node
    PathNode startNode(start, 0.0, calculateHeuristic(start, goal));
    nodes_[coordToString(start)] = startNode;
    openSet_.push(startNode);
    
    while (!openSet_.empty()) {
        PathNode current = openSet_.top();
        openSet_.pop();
        
        std::string currentKey = coordToString(current.position);
        
        if (closedSet_[currentKey]) {
            continue;
        }
        
        closedSet_[currentKey] = true;
        
        if (current.position == goal) {
            return reconstructPath(start, goal);
        }
        
        std::vector<GridCoord> neighbors = grid_->getTraversableNeighbors(current.position);
        
        for (const auto& neighbor : neighbors) {
            std::string neighborKey = coordToString(neighbor);
            
            // Check if neighbor is a temporary obstacle
            bool isTemporaryObstacle = std::find(temporaryObstacles.begin(), 
                                               temporaryObstacles.end(), 
                                               neighbor) != temporaryObstacles.end();
            
            if (closedSet_[neighborKey] || isTemporaryObstacle) {
                continue;
            }
            
            double movementCost = grid_->getMovementCost(current.position, neighbor);
            double tentativeGCost = current.gCost + movementCost;
            
            auto it = nodes_.find(neighborKey);
            bool isNewNode = (it == nodes_.end());
            
            if (isNewNode || tentativeGCost < it->second.gCost) {
                PathNode neighborNode(neighbor, tentativeGCost, calculateHeuristic(neighbor, goal));
                neighborNode.parent = current.position;
                neighborNode.hasParent = true;
                
                nodes_[neighborKey] = neighborNode;
                openSet_.push(neighborNode);
            }
        }
    }
    
    return {};
}

std::vector<GridCoord> Pathfinder::findPathAvoidingAgents(const GridCoord& start, const GridCoord& goal,
                                                        const std::vector<GridCoord>& agentPositions) {
    return findPath(start, goal, agentPositions);
}

std::vector<GridCoord> Pathfinder::reconstructPath(const GridCoord& start, const GridCoord& goal) const {
    std::vector<GridCoord> path;
    GridCoord current = goal;
    
    while (true) {
        path.push_back(current);
        
        if (current == start) {
            break;
        }
        
        std::string currentKey = coordToString(current);
        auto it = nodes_.find(currentKey);
        
        if (it == nodes_.end() || !it->second.hasParent) {
            // Invalid path
            return {};
        }
        
        current = it->second.parent;
    }
    
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<GridCoord> Pathfinder::smoothPath(const std::vector<GridCoord>& path) const {
    if (path.size() <= 2) {
        return path;
    }
    
    std::vector<GridCoord> smoothed;
    smoothed.push_back(path[0]);
    
    size_t current = 0;
    
    while (current < path.size() - 1) {
        size_t farthest = current + 1;
        
        // Find the farthest point we can reach with a straight line
        for (size_t i = current + 2; i < path.size(); ++i) {
            if (isLineOfSightClear(path[current], path[i])) {
                farthest = i;
            } else {
                break;
            }
        }
        
        smoothed.push_back(path[farthest]);
        current = farthest;
    }
    
    return smoothed;
}

std::vector<GridCoord> Pathfinder::simplifyPath(const std::vector<GridCoord>& path) const {
    if (path.size() <= 2) {
        return path;
    }
    
    std::vector<GridCoord> simplified;
    simplified.push_back(path[0]);
    
    for (size_t i = 1; i < path.size() - 1; ++i) {
        GridCoord prev = path[i - 1];
        GridCoord curr = path[i];
        GridCoord next = path[i + 1];
        
        // Check if current point is on the same line as prev and next
        int dx1 = curr.x - prev.x;
        int dy1 = curr.y - prev.y;
        int dx2 = next.x - curr.x;
        int dy2 = next.y - curr.y;
        
        // If direction changes, keep the point
        if (dx1 != dx2 || dy1 != dy2) {
            simplified.push_back(curr);
        }
    }
    
    simplified.push_back(path.back());
    return simplified;
}

bool Pathfinder::isPathValid(const std::vector<GridCoord>& path) const {
    if (!grid_ || path.empty()) {
        return false;
    }
    
    for (const auto& coord : path) {
        if (!grid_->isTraversable(coord)) {
            return false;
        }
    }
    
    return true;
}

bool Pathfinder::isLineOfSightClear(const GridCoord& start, const GridCoord& end) const {
    if (!grid_) {
        return false;
    }
    
    // Bresenham's line algorithm
    int dx = std::abs(end.x - start.x);
    int dy = std::abs(end.y - start.y);
    int x = start.x;
    int y = start.y;
    int stepX = (start.x < end.x) ? 1 : -1;
    int stepY = (start.y < end.y) ? 1 : -1;
    
    if (dx > dy) {
        int err = dx / 2;
        while (x != end.x) {
            if (!grid_->isTraversable(GridCoord(x, y))) {
                return false;
            }
            err -= dy;
            if (err < 0) {
                y += stepY;
                err += dx;
            }
            x += stepX;
        }
    } else {
        int err = dy / 2;
        while (y != end.y) {
            if (!grid_->isTraversable(GridCoord(x, y))) {
                return false;
            }
            err -= dx;
            if (err < 0) {
                x += stepX;
                err += dy;
            }
            y += stepY;
        }
    }
    
    return grid_->isTraversable(end);
}

double Pathfinder::getPathLength(const std::vector<GridCoord>& path) const {
    if (path.size() < 2) {
        return 0.0;
    }
    
    double length = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        length += Utils::euclideanDistance(path[i - 1], path[i]);
    }
    
    return length;
}

GridCoord Pathfinder::getNextStep(const GridCoord& start, const GridCoord& goal) {
    std::vector<GridCoord> path = findPath(start, goal);
    
    if (path.size() >= 2) {
        return path[1];  // Return the next step
    }
    
    return start;  // Stay in place if no path found
}

std::vector<GridCoord> Pathfinder::findNearestGoal(const GridCoord& start,
                                                 const std::vector<GridCoord>& goals) {
    if (goals.empty()) {
        return {};
    }
    
    GridCoord nearestGoal = goals[0];
    std::vector<GridCoord> bestPath;
    double shortestDistance = std::numeric_limits<double>::max();
    
    for (const auto& goal : goals) {
        std::vector<GridCoord> path = findPath(start, goal);
        if (!path.empty()) {
            double distance = getPathLength(path);
            if (distance < shortestDistance) {
                shortestDistance = distance;
                nearestGoal = goal;
                bestPath = path;
            }
        }
    }
    
    return bestPath;
}

GridCoord Pathfinder::selectBestGoal(const GridCoord& start,
                                    const std::vector<GridCoord>& goals) {
    if (goals.empty()) {
        return start;
    }
    
    GridCoord bestGoal = goals[0];
    double shortestDistance = std::numeric_limits<double>::max();
    
    for (const auto& goal : goals) {
        std::vector<GridCoord> path = findPath(start, goal);
        if (!path.empty()) {
            double distance = getPathLength(path);
            if (distance < shortestDistance) {
                shortestDistance = distance;
                bestGoal = goal;
            }
        }
    }
    
    return bestGoal;
}

void Pathfinder::printPath(const std::vector<GridCoord>& path) const {
    std::cout << pathToString(path) << std::endl;
}

std::string Pathfinder::pathToString(const std::vector<GridCoord>& path) const {
    if (path.empty()) {
        return "No path";
    }
    
    std::ostringstream oss;
    oss << "Path (" << path.size() << " steps): ";
    
    for (size_t i = 0; i < path.size(); ++i) {
        oss << "(" << path[i].x << "," << path[i].y << ")";
        if (i < path.size() - 1) {
            oss << " -> ";
        }
    }
    
    oss << " [Length: " << std::fixed << std::setprecision(2) << getPathLength(path) << "]";
    
    return oss.str();
}
