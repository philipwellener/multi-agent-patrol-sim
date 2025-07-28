#include "grid.hpp"
#include "agent.hpp"
#include <iostream>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <cmath>

Grid::Grid(int width, int height) 
    : width_(width), height_(height), random_generator_() {
    cells_.resize(height_, std::vector<CellType>(width_, CellType::EMPTY));
    generateEnvironment();
}

std::string Grid::coordToString(const GridCoord& coord) const {
    return std::to_string(coord.x) + "," + std::to_string(coord.y);
}

GridCoord Grid::stringToCoord(const std::string& str) const {
    size_t comma = str.find(',');
    int x = std::stoi(str.substr(0, comma));
    int y = std::stoi(str.substr(comma + 1));
    return GridCoord(x, y);
}

CellType Grid::getCellType(const GridCoord& coord) const {
    if (!isValidCoord(coord)) {
        return CellType::OBSTACLE;  // Treat out-of-bounds as obstacles
    }
    return cells_[coord.y][coord.x];
}

void Grid::setCellType(const GridCoord& coord, CellType type) {
    if (isValidCoord(coord)) {
        cells_[coord.y][coord.x] = type;
    }
}

bool Grid::isObstacle(const GridCoord& coord) const {
    return getCellType(coord) == CellType::OBSTACLE;
}

bool Grid::isEmpty(const GridCoord& coord) const {
    return getCellType(coord) == CellType::EMPTY;
}

bool Grid::isValidCoord(const GridCoord& coord) const {
    return Utils::isValidGridCoord(coord, width_, height_);
}

void Grid::addWaypoint(const GridCoord& coord) {
    if (isValidCoord(coord) && !isWaypoint(coord)) {
        waypoints_.push_back(coord);
        setCellType(coord, CellType::WAYPOINT);
    }
}

void Grid::removeWaypoint(const GridCoord& coord) {
    auto it = std::find(waypoints_.begin(), waypoints_.end(), coord);
    if (it != waypoints_.end()) {
        waypoints_.erase(it);
        setCellType(coord, CellType::EMPTY);
    }
}

bool Grid::isWaypoint(const GridCoord& coord) const {
    return std::find(waypoints_.begin(), waypoints_.end(), coord) != waypoints_.end();
}

GridCoord Grid::getNearestWaypoint(const GridCoord& from) const {
    if (waypoints_.empty()) {
        return from;  // Return current position if no waypoints
    }
    
    GridCoord nearest = waypoints_[0];
    double minDistance = Utils::euclideanDistance(from, nearest);
    
    for (const auto& waypoint : waypoints_) {
        double distance = Utils::euclideanDistance(from, waypoint);
        if (distance < minDistance) {
            minDistance = distance;
            nearest = waypoint;
        }
    }
    
    return nearest;
}

GridCoord Grid::getRandomWaypoint() const {
    if (waypoints_.empty()) {
        return GridCoord(width_ / 2, height_ / 2);
    }
    
    int index = random_generator_.randomInt(0, waypoints_.size() - 1);
    return waypoints_[index];
}

void Grid::addObstacle(const GridCoord& coord) {
    if (isValidCoord(coord) && !isObstacle(coord) && !isWaypoint(coord)) {
        obstacles_.push_back(coord);
        setCellType(coord, CellType::OBSTACLE);
    }
}

void Grid::removeObstacle(const GridCoord& coord) {
    auto it = std::find(obstacles_.begin(), obstacles_.end(), coord);
    if (it != obstacles_.end()) {
        obstacles_.erase(it);
        setCellType(coord, CellType::EMPTY);
    }
}

void Grid::generateRandomObstacles(double density) {
    clearObstacles();
    
    int totalCells = width_ * height_;
    int numObstacles = static_cast<int>(totalCells * density);
    
    int attempts = 0;
    int maxAttempts = numObstacles * 3;  // Prevent infinite loops
    
    while (obstacles_.size() < static_cast<size_t>(numObstacles) && attempts < maxAttempts) {
        int x = random_generator_.randomInt(0, width_ - 1);
        int y = random_generator_.randomInt(0, height_ - 1);
        GridCoord coord(x, y);
        
        if (isEmpty(coord) && !isWaypoint(coord)) {
            addObstacle(coord);
        }
        
        attempts++;
    }
}

void Grid::clearObstacles() {
    for (const auto& obstacle : obstacles_) {
        setCellType(obstacle, CellType::EMPTY);
    }
    obstacles_.clear();
}

void Grid::addAgentPosition(const GridCoord& coord) {
    if (isValidCoord(coord)) {
        agent_positions_.insert(coordToString(coord));
    }
}

void Grid::removeAgentPosition(const GridCoord& coord) {
    agent_positions_.erase(coordToString(coord));
}

void Grid::updateAgentPosition(const GridCoord& oldPos, const GridCoord& newPos) {
    removeAgentPosition(oldPos);
    addAgentPosition(newPos);
}

bool Grid::isOccupiedByAgent(const GridCoord& coord) const {
    return agent_positions_.find(coordToString(coord)) != agent_positions_.end();
}

bool Grid::isTraversable(const GridCoord& coord) const {
    return isValidCoord(coord) && !isObstacle(coord);
}

std::vector<GridCoord> Grid::getTraversableNeighbors(const GridCoord& coord) const {
    std::vector<GridCoord> neighbors = Utils::getNeighbors(coord, width_, height_);
    
    // Filter out obstacles
    neighbors.erase(
        std::remove_if(neighbors.begin(), neighbors.end(),
            [this](const GridCoord& c) { return !isTraversable(c); }),
        neighbors.end()
    );
    
    return neighbors;
}

double Grid::getMovementCost(const GridCoord& from, const GridCoord& to) const {
    if (!isTraversable(to)) {
        return std::numeric_limits<double>::infinity();
    }
    
    // Basic movement cost (can be enhanced with terrain types)
    double baseCost = Utils::euclideanDistance(from, to);
    
    // Add penalty for agent-occupied cells to encourage avoidance
    if (isOccupiedByAgent(to)) {
        baseCost += 10.0;
    }
    
    return baseCost;
}

bool Grid::isDiagonalMovementValid(const GridCoord& from, const GridCoord& to) const {
    // Check if this is a diagonal movement
    int dx = std::abs(to.x - from.x);
    int dy = std::abs(to.y - from.y);
    
    // Not a diagonal movement
    if (dx <= 1 && dy <= 1 && !(dx == 1 && dy == 1)) {
        return true;
    }
    
    // For diagonal movement, check that both adjacent cells are traversable
    // This prevents "cutting corners" through obstacles
    if (dx == 1 && dy == 1) {
        GridCoord corner1(from.x, to.y);
        GridCoord corner2(to.x, from.y);
        return isTraversable(corner1) && isTraversable(corner2);
    }
    
    // For longer movements, use line-of-sight check
    return isValidCoord(to) && isTraversable(to);
}

void Grid::generateEnvironment() {
    clear();
    generateWaypoints();
    generateRandomObstacles();
}

void Grid::generateWaypoints(int numWaypoints) {
    waypoints_.clear();
    
    // Ensure we have valid waypoints distributed across the grid
    for (int i = 0; i < numWaypoints; ++i) {
        int attempts = 0;
        int maxAttempts = 100;
        
        while (attempts < maxAttempts) {
            int x = random_generator_.randomInt(1, width_ - 2);
            int y = random_generator_.randomInt(1, height_ - 2);
            GridCoord coord(x, y);
            
            if (isEmpty(coord)) {
                // Ensure waypoints are reasonably spaced
                bool tooClose = false;
                for (const auto& waypoint : waypoints_) {
                    if (Utils::euclideanDistance(coord, waypoint) < 3.0) {
                        tooClose = true;
                        break;
                    }
                }
                
                if (!tooClose) {
                    addWaypoint(coord);
                    break;
                }
            }
            
            attempts++;
        }
    }
}

void Grid::display() const {
    std::cout << toString() << std::endl;
}

void Grid::displayWithAgents(const std::vector<std::shared_ptr<Agent>>& agents) const {
    std::cout << toStringWithAgents(agents) << std::endl;
}

std::string Grid::toString() const {
    std::ostringstream oss;
    
    // Top border
    oss << "+" << std::string(width_ * 2 + 1, '-') << "+\n";
    
    for (int y = 0; y < height_; ++y) {
        oss << "| ";
        for (int x = 0; x < width_; ++x) {
            GridCoord coord(x, y);
            
            switch (getCellType(coord)) {
                case CellType::EMPTY:
                    oss << ". ";
                    break;
                case CellType::OBSTACLE:
                    oss << "██";
                    break;
                case CellType::WAYPOINT:
                    oss << "W ";
                    break;
                case CellType::AGENT:
                    oss << "A ";
                    break;
            }
        }
        oss << "|\n";
    }
    
    // Bottom border
    oss << "+" << std::string(width_ * 2 + 1, '-') << "+";
    
    return oss.str();
}

std::string Grid::toStringWithAgents(const std::vector<std::shared_ptr<Agent>>& agents) const {
    // Create a copy of the grid to overlay agents
    auto tempCells = cells_;
    
    // Mark agent positions
    for (size_t i = 0; i < agents.size(); ++i) {
        if (agents[i]) {
            GridCoord agentPos = agents[i]->getGridPosition();
            if (isValidCoord(agentPos)) {
                tempCells[agentPos.y][agentPos.x] = CellType::AGENT;
            }
        }
    }
    
    std::ostringstream oss;
    
    // Top border
    oss << "+" << std::string(width_ * 2 + 1, '-') << "+\n";
    
    for (int y = 0; y < height_; ++y) {
        oss << "| ";
        for (int x = 0; x < width_; ++x) {
            GridCoord coord(x, y);
            
            // Check if there's an agent at this position
            int agentId = -1;
            for (size_t i = 0; i < agents.size(); ++i) {
                if (agents[i] && agents[i]->getGridPosition() == coord) {
                    agentId = static_cast<int>(i);
                    break;
                }
            }
            
            if (agentId >= 0) {
                oss << "A" << agentId;
            } else {
                switch (getCellType(coord)) {
                    case CellType::EMPTY:
                        oss << ". ";
                        break;
                    case CellType::OBSTACLE:
                        oss << "██";
                        break;
                    case CellType::WAYPOINT:
                        oss << "W ";
                        break;
                    case CellType::AGENT:
                        oss << "A ";
                        break;
                }
            }
        }
        oss << "|\n";
    }
    
    // Bottom border
    oss << "+" << std::string(width_ * 2 + 1, '-') << "+";
    
    return oss.str();
}

void Grid::clear() {
    for (auto& row : cells_) {
        std::fill(row.begin(), row.end(), CellType::EMPTY);
    }
    waypoints_.clear();
    obstacles_.clear();
    agent_positions_.clear();
}

void Grid::reset() {
    clear();
    generateEnvironment();
}

GridCoord Grid::getRandomEmptyCell() const {
    std::vector<GridCoord> emptyCells = getEmptyCells();
    
    if (emptyCells.empty()) {
        return GridCoord(0, 0);  // Fallback
    }
    
    int index = random_generator_.randomInt(0, emptyCells.size() - 1);
    return emptyCells[index];
}

std::vector<GridCoord> Grid::getEmptyCells() const {
    std::vector<GridCoord> emptyCells;
    
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            GridCoord coord(x, y);
            if (isEmpty(coord)) {
                emptyCells.push_back(coord);
            }
        }
    }
    
    return emptyCells;
}

double Grid::getObstacleDensity() const {
    int totalCells = width_ * height_;
    return static_cast<double>(obstacles_.size()) / totalCells;
}

int Grid::getObstacleCount() const {
    return static_cast<int>(obstacles_.size());
}

int Grid::getWaypointCount() const {
    return static_cast<int>(waypoints_.size());
}
