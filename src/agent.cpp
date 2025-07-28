#include "agent.hpp"
#include <iostream>
#include <sstream>
#include <algorithm>
#include <cmath>

Agent::Agent(int id, const Utils::Vector2D& startPos, Grid* grid)
    : id_(id), position_(startPos), velocity_(0.0, 0.0),
      currentGridPos_(startPos.gridX(), startPos.gridY()),
      targetGridPos_(startPos.gridX(), startPos.gridY()),
      state_(AgentState::IDLE), speed_(Config::AGENT_SPEED),
      visionRange_(Config::AGENT_VISION_RANGE),
      pathIndex_(0), hasDestination_(false),
      currentWaypointIndex_(0), isPatrolling_(false),
      stuckTimer_(0.0), lastPosition_(startPos),
      waypointAttemptTimer_(0.0), lastFailedWaypointIndex_(-1),
      lastWaypointReachedTime_(0.0), consecutivePathFailures_(0), lastPathFailureTime_(0.0),
      positionHistoryTimer_(0.0), lastOscillationDetectionTime_(0.0),
      grid_(grid), lastUpdateTime_(0.0),
      avoidanceVector_(0.0, 0.0) {
    
    if (grid_) {
        pathfinder_ = std::make_unique<Pathfinder>(grid_);
        grid_->addAgentPosition(currentGridPos_);
    }
    
    initializeBehaviorTree();
}

void Agent::setPosition(const Utils::Vector2D& pos) {
    if (grid_) {
        grid_->removeAgentPosition(currentGridPos_);
    }
    
    position_ = pos;
    updateGridPosition();
    
    if (grid_) {
        grid_->addAgentPosition(currentGridPos_);
    }
}

void Agent::setGridPosition(const GridCoord& pos) {
    setPosition(Utils::Vector2D(pos.x, pos.y));
}

void Agent::update(double deltaTime) {
    lastUpdateTime_ = updateTimer_.elapsed();
    
    // Update position history for oscillation detection
    positionHistoryTimer_ += deltaTime;
    if (positionHistoryTimer_ >= 0.5) {  // Record position every 0.5 seconds
        recentPositions_.push_back(position_);
        if (recentPositions_.size() > 8) {  // Keep last 4 seconds of positions
            recentPositions_.erase(recentPositions_.begin());
        }
        positionHistoryTimer_ = 0.0;
    }
    
    // Detect position oscillation (stuck cycling)
    bool isOscillating = false;
    if (recentPositions_.size() >= 6) {  // Need at least 3 seconds of data
        // Check if agent is oscillating between positions
        double totalDistance = 0.0;
        double maxDistance = 0.0;
        Utils::Vector2D centroid(0.0, 0.0);
        
        // Calculate centroid of recent positions
        for (const auto& pos : recentPositions_) {
            centroid = centroid + pos;
        }
        centroid = centroid * (1.0 / recentPositions_.size());
        
        // Check if all positions are close to centroid (indicates oscillation)
        for (const auto& pos : recentPositions_) {
            double dist = (pos - centroid).magnitude();
            maxDistance = std::max(maxDistance, dist);
        }
        
        // Also check if agent is making minimal forward progress
        double progressDistance = (recentPositions_.back() - recentPositions_.front()).magnitude();
        
        // If agent is staying in small area with minimal progress, it's oscillating
        if (maxDistance < 2.0 && progressDistance < 1.5) {
            isOscillating = true;
        }
    }
    
    // Check if agent is stuck (not moving significantly)
    double distanceMoved = (position_ - lastPosition_).magnitude();
    if (distanceMoved < 0.2) {  // More sensitive stuck detection
        stuckTimer_ += deltaTime;
    } else {
        stuckTimer_ = 0.0;
        lastPosition_ = position_;
    }
    
    // Update waypoint attempt timer
    if (waypointAttemptTimer_ > 0.0) {
        waypointAttemptTimer_ -= deltaTime;
    }
    
    // Handle oscillation detection
    double currentTime = updateTimer_.elapsed();
    if (isOscillating && (currentTime - lastOscillationDetectionTime_) > 8.0) {
        // Agent is oscillating - force a different strategy
        lastOscillationDetectionTime_ = currentTime;
        waypointAttemptTimer_ = 5.0;  // Pause waypoint attempts
        consecutivePathFailures_ = 0;  // Reset failure count
        recentWaypointHistory_.clear();  // Clear waypoint history
        recentPositions_.clear();  // Clear position history
        
        std::cout << "Agent " << id_ << " detected oscillating, forcing recovery" << std::endl;
        
        // Force the agent to pick a completely different waypoint
        if (!patrolWaypoints_.empty()) {
            // Find the waypoint furthest from current position
            double maxDist = 0.0;
            size_t bestIndex = 0;
            for (size_t i = 0; i < patrolWaypoints_.size(); ++i) {
                double dist = Utils::euclideanDistance(currentGridPos_, patrolWaypoints_[i]);
                if (dist > maxDist) {
                    maxDist = dist;
                    bestIndex = i;
                }
            }
            currentWaypointIndex_ = bestIndex;
        }
        
        // Force unstuck behavior
        forceUnstuck();
        return;
    }
    
    // Detect oscillation pattern - if we've been changing waypoints too frequently
    if (consecutivePathFailures_ >= 3 && (currentTime - lastPathFailureTime_) < 5.0) {
        // Agent is oscillating - force a longer pause and different strategy
        waypointAttemptTimer_ = 8.0;  // Long pause
        consecutivePathFailures_ = 0;  // Reset to break the cycle
        recentWaypointHistory_.clear();  // Clear history to allow any waypoint
        
        // Force to IDLE state temporarily
        setState(AgentState::IDLE);
        clearPath();
        hasDestination_ = false;
    }
    
    // If stuck for too long, force a random movement
    if (stuckTimer_ > 1.5) {  // Faster recovery time
        forceUnstuck();
        stuckTimer_ = 0.0;
    }
    
    updateAgentDetection({});  // This would be called with all agents in the simulation
    updateBehaviorTree();
    updatePosition(deltaTime);
    updateGridPosition();
    updateTrail();
}

void Agent::updatePosition(double deltaTime) {
    Utils::Vector2D originalPosition = position_;
    Utils::Vector2D newPosition = position_;
    
    if (hasPath()) {
        followPath(deltaTime);
        // Apply velocity that was set by followPath
        if (velocity_.magnitude() > 0.01) {
            newPosition = position_ + velocity_ * deltaTime;
        }
    } else {
        // Apply velocity even without a path (e.g., for obstacle avoidance)
        if (velocity_.magnitude() > 0.01) {
            newPosition = position_ + velocity_ * deltaTime;
        }
    }
    
    // Apply avoidance vector with reduced strength
    if (avoidanceVector_.magnitude() > 0.1) {
        Utils::Vector2D avoidanceMovement = avoidanceVector_ * speed_ * deltaTime * 0.2;  // Further reduced
        newPosition = newPosition + avoidanceMovement;
        avoidanceVector_ = avoidanceVector_ * 0.9;  // Slower decay for stability
    }
    
    // Validate the new position - but be less strict about stopping movement
    if (grid_) {
        GridCoord newGridPos(newPosition.gridX(), newPosition.gridY());
        
        // Check bounds
        if (!grid_->isValidCoord(newGridPos)) {
            // Clamp to valid bounds
            newPosition.x = Utils::clamp(newPosition.x, 0.1, grid_->getWidth() - 1.1);
            newPosition.y = Utils::clamp(newPosition.y, 0.1, grid_->getHeight() - 1.1);
            newGridPos = GridCoord(newPosition.gridX(), newPosition.gridY());
        }
        
        // Check obstacles - only block if directly hitting solid obstacles
        if (grid_->isObstacle(newGridPos)) {
            // Instead of stopping completely, try to slide along obstacles
            Utils::Vector2D movement = newPosition - originalPosition;
            
            // Try moving only in X direction
            Utils::Vector2D xOnlyPos = originalPosition + Utils::Vector2D(movement.x, 0.0);
            GridCoord xOnlyGrid(xOnlyPos.gridX(), xOnlyPos.gridY());
            if (grid_->isValidCoord(xOnlyGrid) && !grid_->isObstacle(xOnlyGrid)) {
                newPosition = xOnlyPos;
            }
            // Try moving only in Y direction
            else {
                Utils::Vector2D yOnlyPos = originalPosition + Utils::Vector2D(0.0, movement.y);
                GridCoord yOnlyGrid(yOnlyPos.gridX(), yOnlyPos.gridY());
                if (grid_->isValidCoord(yOnlyGrid) && !grid_->isObstacle(yOnlyGrid)) {
                    newPosition = yOnlyPos;
                } else {
                    // Completely blocked - stay in place but don't zero velocity (for stuck detection)
                    newPosition = originalPosition;
                }
            }
        }
    }
    
    position_ = newPosition;
}

void Agent::updateBehaviorTree() {
    if (behaviorTree_) {
        behaviorTree_->tick();
    }
}

void Agent::updateGridPosition() {
    GridCoord newGridPos(position_.gridX(), position_.gridY());
    
    if (newGridPos != currentGridPos_) {
        if (grid_) {
            grid_->updateAgentPosition(currentGridPos_, newGridPos);
        }
        currentGridPos_ = newGridPos;
    }
}

void Agent::updateTrail() {
    trail_.push_back(position_);
    if (trail_.size() > TRAIL_LENGTH) {
        trail_.pop_front();
    }
}

bool Agent::setDestination(const GridCoord& destination) {
    if (!grid_ || !grid_->isTraversable(destination)) {
        return false;
    }
    
    targetGridPos_ = destination;
    hasDestination_ = true;
    
    return planPathTo(destination);
}

void Agent::clearPath() {
    currentPath_.clear();
    pathIndex_ = 0;
    hasDestination_ = false;
}

GridCoord Agent::getCurrentTarget() const {
    if (hasPath()) {
        return currentPath_[pathIndex_];
    }
    return currentGridPos_;
}

GridCoord Agent::getNextWaypoint() const {
    if (!patrolWaypoints_.empty()) {
        return patrolWaypoints_[currentWaypointIndex_];
    }
    return currentGridPos_;
}

void Agent::setPatrolWaypoints(const std::vector<GridCoord>& waypoints) {
    patrolWaypoints_ = waypoints;
    currentWaypointIndex_ = 0;
}

void Agent::addPatrolWaypoint(const GridCoord& waypoint) {
    patrolWaypoints_.push_back(waypoint);
}

void Agent::startPatrol() {
    isPatrolling_ = true;
    setState(AgentState::PATROLLING);
    
    if (!patrolWaypoints_.empty()) {
        setDestination(patrolWaypoints_[currentWaypointIndex_]);
    }
}

void Agent::stopPatrol() {
    isPatrolling_ = false;
    setState(AgentState::IDLE);
}

std::vector<GridCoord> Agent::detectNearbyAgents(const std::vector<Agent*>& allAgents) const {
    std::vector<GridCoord> nearbyAgents;
    
    for (const auto* agent : allAgents) {
        if (agent && agent != this) {
            double distance = Utils::euclideanDistance(currentGridPos_, agent->getGridPosition());
            if (distance <= visionRange_) {
                nearbyAgents.push_back(agent->getGridPosition());
            }
        }
    }
    
    return nearbyAgents;
}

void Agent::updateAgentDetection(const std::vector<Agent*>& allAgents) {
    detectedAgents_.clear();
    
    for (size_t i = 0; i < allAgents.size(); ++i) {
        const auto* agent = allAgents[i];
        if (agent && agent != this) {
            double distance = Utils::euclideanDistance(currentGridPos_, agent->getGridPosition());
            if (distance <= Config::COLLISION_AVOIDANCE_RANGE) {
                detectedAgents_.push_back(agent->getId());
                
                // Calculate avoidance vector
                Utils::Vector2D separation = position_ - agent->getPosition();
                if (separation.magnitude() > 0.1) {
                    separation = separation.normalize();
                    avoidanceVector_ = avoidanceVector_ + separation;
                }
            }
        }
    }
}

Utils::Vector2D Agent::calculateAvoidanceVector(const std::vector<Agent*>& nearbyAgents) const {
    Utils::Vector2D avoidance(0.0, 0.0);
    
    for (const auto* agent : nearbyAgents) {
        if (agent && agent != this) {
            Utils::Vector2D separation = position_ - agent->getPosition();
            double distance = separation.magnitude();
            
            if (distance > 0.1 && distance < Config::COLLISION_AVOIDANCE_RANGE) {
                separation = separation.normalize();
                // Stronger avoidance for closer agents
                double strength = (Config::COLLISION_AVOIDANCE_RANGE - distance) / Config::COLLISION_AVOIDANCE_RANGE;
                
                // Add slight randomness based on agent ID to break symmetrical deadlocks
                Utils::RandomGenerator rng;
                double randomOffset = (id_ % 4) * 0.1 - 0.2; // Range: -0.2 to +0.1
                Utils::Vector2D perpendicular(-separation.y, separation.x);
                separation = separation + perpendicular * randomOffset;
                separation = separation.normalize();
                
                avoidance = avoidance + separation * strength;
            }
        }
    }
    
    return avoidance;
}

std::vector<GridCoord> Agent::detectNearbyObstacles() const {
    std::vector<GridCoord> obstacles;
    
    if (!grid_) return obstacles;
    
    // Check surrounding cells
    for (int dx = -static_cast<int>(visionRange_); dx <= static_cast<int>(visionRange_); ++dx) {
        for (int dy = -static_cast<int>(visionRange_); dy <= static_cast<int>(visionRange_); ++dy) {
            GridCoord checkPos(currentGridPos_.x + dx, currentGridPos_.y + dy);
            
            if (grid_->isValidCoord(checkPos) && grid_->isObstacle(checkPos)) {
                double distance = Utils::euclideanDistance(currentGridPos_, checkPos);
                if (distance <= visionRange_) {
                    obstacles.push_back(checkPos);
                }
            }
        }
    }
    
    return obstacles;
}

bool Agent::isPathBlocked() const {
    if (!hasPath() || !grid_) {
        return false;
    }
    
    // Check if the next few steps in the path are blocked
    for (size_t i = pathIndex_; i < std::min(pathIndex_ + 3, currentPath_.size()); ++i) {
        if (!grid_->isTraversable(currentPath_[i]) || 
            grid_->isOccupiedByAgent(currentPath_[i])) {
            return true;
        }
    }
    
    return false;
}

// Behavior Tree Actions
NodeStatus Agent::actionMoveTo(const GridCoord& target) {
    if (!setDestination(target)) {
        return NodeStatus::FAILURE;
    }
    
    setState(AgentState::MOVING);
    
    if (isAtPosition(target)) {
        return NodeStatus::SUCCESS;
    }
    
    return NodeStatus::RUNNING;
}

NodeStatus Agent::actionPatrol() {
    if (patrolWaypoints_.empty()) {
        return NodeStatus::FAILURE;
    }
    
    setState(AgentState::PATROLLING);
    
    if (patrolWaypoints_.size() <= 1) {
        return NodeStatus::SUCCESS; // Nothing to patrol
    }
    
    GridCoord currentWaypoint = patrolWaypoints_[currentWaypointIndex_];
    
    // Check if we've reached the current waypoint
    if (isAtPosition(currentWaypoint, 1.5)) {
        // Reached current waypoint - move to next one
        recentWaypointHistory_.push_back(currentWaypointIndex_);
        if (recentWaypointHistory_.size() > 3) {
            recentWaypointHistory_.erase(recentWaypointHistory_.begin());  // Keep only last 3
        }
        lastWaypointReachedTime_ = updateTimer_.elapsed();
        
        currentWaypointIndex_ = (currentWaypointIndex_ + 1) % patrolWaypoints_.size();
        GridCoord nextWaypoint = patrolWaypoints_[currentWaypointIndex_];
        
        if (id_ == 0) {
            std::cout << "Agent " << id_ << " reached waypoint, moving to next: " 
                      << nextWaypoint.x << "," << nextWaypoint.y << std::endl;
        }
        
        // Clear old path and set new destination
        clearPath();
        if (!setDestination(nextWaypoint)) {
            // If we can't path to the next waypoint, try alternatives but with improved logic
            if (waypointAttemptTimer_ <= 0.0) {
                int attempts = 0;
                bool foundPath = false;
                int originalIndex = currentWaypointIndex_;
                size_t startIndex = currentWaypointIndex_;
                
                while (attempts < patrolWaypoints_.size() && !foundPath) {
                    currentWaypointIndex_ = (currentWaypointIndex_ + 1) % patrolWaypoints_.size();
                    
                    // Skip recently visited waypoints if possible
                    bool recentlyVisited = false;
                    for (size_t recentIndex : recentWaypointHistory_) {
                        if (currentWaypointIndex_ == recentIndex) {
                            recentlyVisited = true;
                            break;
                        }
                    }
                    
                    // Skip the last failed waypoint if we just tried it recently
                    if ((currentWaypointIndex_ == lastFailedWaypointIndex_ || recentlyVisited) 
                        && attempts < patrolWaypoints_.size() - 1) {
                        attempts++;
                        continue;
                    }
                    
                    nextWaypoint = patrolWaypoints_[currentWaypointIndex_];
                    if (setDestination(nextWaypoint)) {
                        foundPath = true;
                        lastFailedWaypointIndex_ = -1; // Reset failed waypoint
                        consecutivePathFailures_ = 0;   // Reset failure count
                    } else {
                        consecutivePathFailures_++;
                        lastPathFailureTime_ = updateTimer_.elapsed();
                    }
                    attempts++;
                }
                
                if (!foundPath) {
                    // All waypoints failed, set cooldown and remember the failed waypoint
                    lastFailedWaypointIndex_ = originalIndex;
                    // Increase cooldown based on consecutive failures
                    double cooldownTime = 3.0 + (consecutivePathFailures_ * 0.5);
                    waypointAttemptTimer_ = std::min(cooldownTime, 10.0); // Cap at 10 seconds
                    currentWaypointIndex_ = originalIndex; // Reset to original
                    return NodeStatus::FAILURE;
                }
            } else {
                // Still in cooldown, don't change waypoints rapidly
                return NodeStatus::RUNNING;
            }
        } else {
            // Successfully set destination, reset failure tracking
            consecutivePathFailures_ = 0;
        }
    } else if (!hasDestination_) {
        // No destination set - find the most accessible waypoint
        if (id_ == 0) {
            std::cout << "Agent " << id_ << " setting initial destination to waypoint " 
                      << currentWaypoint.x << "," << currentWaypoint.y << std::endl;
        }
        
        // Try current waypoint first, then find best alternative using smarter selection
        if (!setDestination(currentWaypoint)) {
            // Find the most reachable waypoint using multiple criteria
            GridCoord bestWaypoint = currentWaypoint;
            int shortestPathLength = INT_MAX;
            double bestScore = -1.0;
            
            for (size_t i = 0; i < patrolWaypoints_.size(); ++i) {
                // Skip recently visited waypoints if we have alternatives
                bool recentlyVisited = false;
                for (size_t recentIndex : recentWaypointHistory_) {
                    if (i == recentIndex) {
                        recentlyVisited = true;
                        break;
                    }
                }
                
                auto path = pathfinder_->findPath(currentGridPos_, patrolWaypoints_[i]);
                if (path.size() > 1) {
                    double distance = Utils::euclideanDistance(currentGridPos_, patrolWaypoints_[i]);
                    double pathLength = static_cast<double>(path.size());
                    
                    // Score based on multiple factors
                    double score = 100.0 / pathLength;  // Prefer shorter paths
                    score += 20.0 / (distance + 1.0);   // Prefer closer waypoints
                    
                    if (recentlyVisited) {
                        score *= 0.3;  // Heavily penalize recently visited
                    }
                    
                    if (i == lastFailedWaypointIndex_) {
                        score *= 0.1;  // Heavily penalize recently failed
                    }
                    
                    if (score > bestScore) {
                        bestScore = score;
                        shortestPathLength = static_cast<int>(path.size());
                        bestWaypoint = patrolWaypoints_[i];
                        currentWaypointIndex_ = i;
                    }
                }
            }
            
            if (bestScore > 0 && setDestination(bestWaypoint)) {
                consecutivePathFailures_ = 0;
            } else {
                consecutivePathFailures_++;
                return NodeStatus::FAILURE;
            }
        } else {
            consecutivePathFailures_ = 0;
        }
    }
    
    return NodeStatus::RUNNING;
}

NodeStatus Agent::actionAvoidObstacle() {
    setState(AgentState::AVOIDING_OBSTACLE);
    
    // Try to find an alternative path first
    if (hasDestination_ && pathfinder_) {
        std::vector<GridCoord> obstacles = detectNearbyObstacles();
        
        // Filter out waypoints from obstacles - we want to path through waypoints!
        std::vector<GridCoord> actualObstacles;
        for (const auto& obstacle : obstacles) {
            bool isWaypoint = false;
            
            // Check if this "obstacle" is actually a waypoint
            if (grid_) {
                const auto& waypoints = grid_->getWaypoints();
                for (const auto& waypoint : waypoints) {
                    if (obstacle.x == waypoint.x && obstacle.y == waypoint.y) {
                        isWaypoint = true;
                        break;
                    }
                }
            }
            
            // Also check patrol waypoints
            for (const auto& waypoint : patrolWaypoints_) {
                if (obstacle.x == waypoint.x && obstacle.y == waypoint.y) {
                    isWaypoint = true;
                    break;
                }
            }
            
            if (!isWaypoint) {
                actualObstacles.push_back(obstacle);
            }
        }
        
        std::vector<GridCoord> newPath = pathfinder_->findPath(currentGridPos_, targetGridPos_, actualObstacles);
        
        if (!newPath.empty()) {
            currentPath_ = newPath;
            pathIndex_ = 0;
            return NodeStatus::SUCCESS;
        }
    }
    
    // If pathfinding fails, try local obstacle avoidance movement
    if (grid_) {
        // Use only cardinal directions to prevent diagonal movement through obstacles
        std::vector<Utils::Vector2D> directions = {
            {1.0, 0.0}, {-1.0, 0.0}, {0.0, 1.0}, {0.0, -1.0}  // cardinal only
        };
        
        std::vector<double> stepSizes = {1.0, 0.5, 1.5, 2.0};
        
        for (double stepSize : stepSizes) {
            for (const auto& dir : directions) {
                Utils::Vector2D newPos = position_ + dir * stepSize;
                GridCoord newGridPos(newPos.gridX(), newPos.gridY());
                
                if (grid_->isValidCoord(newGridPos) && grid_->isTraversable(newGridPos)) {
                    // Check if this direction moves us toward our destination
                    bool movingTowardDestination = true;
                    if (hasDestination_) {
                        double currentDist = Utils::euclideanDistance(currentGridPos_, targetGridPos_);
                        double newDist = Utils::euclideanDistance(newGridPos, targetGridPos_);
                        movingTowardDestination = (newDist < currentDist);
                    }
                    
                    // Prefer directions that move toward destination, but accept any valid direction if stuck
                    if (movingTowardDestination || stepSize > 1.0) {
                        velocity_ = dir.normalize() * speed_;
                        return NodeStatus::RUNNING;  // Continue avoiding
                    }
                }
            }
        }
        
        // If completely stuck, try a random direction with larger steps
        Utils::RandomGenerator rng;
        // ok last resort - just move randomly and hope for the best
        for (int attempts = 0; attempts < 20; ++attempts) {
            double angle = rng.randomDouble(0.0, 2.0 * M_PI);
            Utils::Vector2D randomDir(cos(angle), sin(angle));
            Utils::Vector2D newPos = position_ + randomDir * (2.0 + attempts * 0.5);
            GridCoord newGridPos(newPos.gridX(), newPos.gridY());
            
            if (grid_->isValidCoord(newGridPos) && grid_->isTraversable(newGridPos)) {
                velocity_ = randomDir.normalize() * speed_;
                return NodeStatus::RUNNING;
            }
        }
    }
    
    // completely stuck, just wait it out
    velocity_ = Utils::Vector2D(0.0, 0.0);
    return NodeStatus::RUNNING;
}

NodeStatus Agent::actionAvoidAgent() {
    setState(AgentState::AVOIDING_AGENT);
    
    // Agent avoidance is handled in updateAgentDetection
    // This action just indicates we're actively avoiding
    if (detectedAgents_.empty()) {
        return NodeStatus::SUCCESS;
    }
    
    return NodeStatus::RUNNING;
}

NodeStatus Agent::actionWait() {
    setState(AgentState::WAITING);
    velocity_ = Utils::Vector2D(0.0, 0.0);
    return NodeStatus::SUCCESS;
}

void Agent::forceUnstuck() {
    // Force the agent to move to break out of stuck situations
    setState(AgentState::AVOIDING_OBSTACLE);
    
    std::cout << "Agent " << id_ << " force unstuck at (" << position_.x << ", " << position_.y << ")" << std::endl;
    
    // Clear all state to force fresh planning
    clearPath();
    hasDestination_ = false;
    recentWaypointHistory_.clear();
    recentPositions_.clear();  // Clear position history
    
    // Try random directions to break free, but only use cardinal directions to be consistent
    Utils::RandomGenerator rng;
    std::vector<Utils::Vector2D> directions = {
        {1.0, 0.0}, {-1.0, 0.0}, {0.0, 1.0}, {0.0, -1.0}  // Cardinal only, consistent with other movement
    };
    
    // Shuffle directions for randomness
    for (int i = 0; i < 10; ++i) {
        int a = rng.randomInt(0, directions.size() - 1);
        int b = rng.randomInt(0, directions.size() - 1);
        std::swap(directions[a], directions[b]);
    }
    
    bool foundEscape = false;
    for (const auto& dir : directions) {
        // Try multiple distances to find best escape route, starting with larger distances
        for (double distance : {4.0, 3.5, 3.0, 2.5, 2.0, 1.5, 1.0}) {
            Utils::Vector2D newPos = position_ + dir * distance;
            GridCoord newGridPos(newPos.gridX(), newPos.gridY());
            
            if (grid_ && grid_->isValidCoord(newGridPos) && grid_->isTraversable(newGridPos)) {
                // Move more aggressively to break free from oscillation
                velocity_ = dir.normalize() * speed_ * 2.0;  // Strong boost to break free
                waypointAttemptTimer_ = 2.0;  // Longer cooldown after unsticking
                foundEscape = true;
                break;
            }
        }
        if (foundEscape) break;
    }
    
    // If still no escape found, try a more aggressive approach
    if (!foundEscape && !patrolWaypoints_.empty()) {
        // Find the waypoint furthest from current position
        GridCoord bestWaypoint = patrolWaypoints_[0];
        double maxDistance = 0.0;
        size_t bestIndex = 0;
        
        for (size_t i = 0; i < patrolWaypoints_.size(); ++i) {
            double dist = Utils::euclideanDistance(currentGridPos_, patrolWaypoints_[i]);
            if (dist > maxDistance) {
                maxDistance = dist;
                bestWaypoint = patrolWaypoints_[i];
                bestIndex = i;
            }
        }
        
        // Force new waypoint selection
        currentWaypointIndex_ = bestIndex;
        waypointAttemptTimer_ = 3.0;  // Longer cooldown for aggressive unsticking
        
        std::cout << "Agent " << id_ << " forcing waypoint change to " << bestWaypoint.x << "," << bestWaypoint.y << std::endl;
        
        // Try to move towards the general direction of the distant waypoint
        Utils::Vector2D dirToWaypoint = Utils::Vector2D(bestWaypoint.x, bestWaypoint.y) - position_;
        if (dirToWaypoint.magnitude() > 0.1) {
            // Use only the dominant axis for movement (cardinal direction)
            if (std::abs(dirToWaypoint.x) > std::abs(dirToWaypoint.y)) {
                velocity_ = Utils::Vector2D(dirToWaypoint.x > 0 ? 1.0 : -1.0, 0.0) * speed_ * 2.0;
            } else {
                velocity_ = Utils::Vector2D(0.0, dirToWaypoint.y > 0 ? 1.0 : -1.0) * speed_ * 2.0;
            }
        }
    }
}

NodeStatus Agent::actionPlanPath(const GridCoord& destination) {
    setState(AgentState::PLANNING_PATH);
    
    if (planPathTo(destination)) {
        return NodeStatus::SUCCESS;
    }
    
    return NodeStatus::FAILURE;
}

// Behavior Tree Conditions
bool Agent::conditionHasDestination() const {
    return hasDestination_;
}

bool Agent::conditionAtDestination() const {
    return hasDestination_ && isAtPosition(targetGridPos_);
}

bool Agent::conditionPathBlocked() const {
    return isPathBlocked();
}

bool Agent::conditionAgentNearby() const {
    return !detectedAgents_.empty();
}

bool Agent::conditionObstacleNearby() const {
    // Only trigger obstacle avoidance if we're actually blocked or very close to obstacles
    auto obstacles = detectNearbyObstacles();
    if (obstacles.empty()) {
        return false;
    }
    
    // Filter out waypoints from obstacles - we want to reach waypoints, not avoid them!
    std::vector<GridCoord> actualObstacles;
    for (const auto& obstacle : obstacles) {
        bool isWaypoint = false;
        
        // Check against grid waypoints
        if (grid_) {
            const auto& waypoints = grid_->getWaypoints();
            for (const auto& waypoint : waypoints) {
                if (obstacle.x == waypoint.x && obstacle.y == waypoint.y) {
                    isWaypoint = true;
                    break;
                }
            }
        }
        
        // Check against our specific patrol waypoints
        if (!isWaypoint) {
            for (const auto& waypoint : patrolWaypoints_) {
                if (obstacle.x == waypoint.x && obstacle.y == waypoint.y) {
                    isWaypoint = true;
                    break;
                }
            }
        }
        
        // Check if we're trying to reach this as our current target
        if (!isWaypoint && hasDestination_) {
            if (obstacle.x == targetGridPos_.x && obstacle.y == targetGridPos_.y) {
                isWaypoint = true;
            }
        }
        
        if (!isWaypoint) {
            actualObstacles.push_back(obstacle);
        }
    }
    
    if (actualObstacles.empty()) {
        return false; // No real obstacles, only waypoints
    }
    
    // Only trigger avoidance if obstacles are directly blocking our path
    Utils::Vector2D movementDirection;
    bool hasMovementDirection = false;
    
    // Determine intended movement direction
    if (hasPath() && !currentPath_.empty()) {
        GridCoord nextStep = getNextPathStep();
        Utils::Vector2D nextPos(static_cast<double>(nextStep.x), static_cast<double>(nextStep.y));
        movementDirection = (nextPos - position_).normalize();
        hasMovementDirection = true;
    } else if (velocity_.magnitude() > 0.01) {
        movementDirection = velocity_.normalize();
        hasMovementDirection = true;
    }
    
    if (!hasMovementDirection) {
        return false; // No movement intent
    }
    
    // Only trigger if obstacle is directly in our path (next 1-2 steps)
    for (int step = 1; step <= 2; ++step) {
        Utils::Vector2D checkPos = position_ + movementDirection * step;
        GridCoord checkGrid(static_cast<int>(checkPos.x + 0.5), static_cast<int>(checkPos.y + 0.5));
        
        for (const auto& obstacle : actualObstacles) {
            if (obstacle.x == checkGrid.x && obstacle.y == checkGrid.y) {
                return true; // Direct collision imminent
            }
        }
    }
    
    return false;
}

bool Agent::conditionIsPatrolling() const {
    return isPatrolling_;
}

std::string Agent::getStateString() const {
    switch (state_) {
        case AgentState::IDLE: return "IDLE";
        case AgentState::PATROLLING: return "PATROLLING";
        case AgentState::PLANNING_PATH: return "PLANNING_PATH";
        case AgentState::MOVING: return "MOVING";
        case AgentState::AVOIDING_OBSTACLE: return "AVOIDING_OBSTACLE";
        case AgentState::AVOIDING_AGENT: return "AVOIDING_AGENT";
        case AgentState::REROUTING: return "REROUTING";
        case AgentState::WAITING: return "WAITING";
    }
    return "UNKNOWN_STATE";  // shouldn't happen but just in case
}

std::string Agent::toString() const {
    std::ostringstream oss;
    oss << "Agent " << id_ << ": " << Utils::formatPosition(position_) 
        << " [" << getStateString() << "]";
    return oss.str();
}

std::string Agent::getDebugInfo() const {
    std::ostringstream oss;
    oss << "Agent " << id_ << " Debug Info:\n";
    oss << "  Position: " << Utils::formatPosition(position_) << "\n";
    oss << "  Grid Position: " << Utils::formatGridCoord(currentGridPos_) << "\n";
    oss << "  State: " << getStateString() << "\n";
    oss << "  Speed: " << speed_ << "\n";
    oss << "  Has Destination: " << (hasDestination_ ? "Yes" : "No") << "\n";
    oss << "  Path Length: " << currentPath_.size() << "\n";
    oss << "  Path Index: " << pathIndex_ << "\n";
    oss << "  Is Patrolling: " << (isPatrolling_ ? "Yes" : "No") << "\n";
    oss << "  Detected Agents: " << detectedAgents_.size() << "\n";
    
    return oss.str();
}

void Agent::printStatus() const {
    std::cout << toString() << std::endl;
}

double Agent::distanceToTarget() const {
    if (hasDestination_) {
        return Utils::euclideanDistance(currentGridPos_, targetGridPos_);
    }
    return 0.0;
}

bool Agent::isAtPosition(const GridCoord& pos, double threshold) const {
    return Utils::euclideanDistance(currentGridPos_, pos) <= threshold;
}

GridCoord Agent::selectRandomWaypoint() const {
    if (grid_) {
        const auto& waypoints = grid_->getWaypoints();
        if (!waypoints.empty()) {
            Utils::RandomGenerator rng;
            int index = rng.randomInt(0, waypoints.size() - 1);
            return waypoints[index];
        }
    }
    return currentGridPos_;
}

bool Agent::planPathTo(const GridCoord& destination) {
    if (!pathfinder_) {
        return false;
    }
    
    std::vector<GridCoord> newPath = pathfinder_->findPath(currentGridPos_, destination);
    
    if (!newPath.empty()) {
        currentPath_ = newPath;
        pathIndex_ = 0;
        return true;
    }
    
    return false;
}

void Agent::followPath(double deltaTime) {
    if (!hasPath()) {
        return;
    }
    
    GridCoord nextStep = getNextPathStep();
    Utils::Vector2D target = nextStep.toVector2D();
    
    // Calculate movement direction
    Utils::Vector2D direction = target - position_;
    double distance = direction.magnitude();
    
    if (distance < 0.1) {
        // Reached current path step, move to next
        pathIndex_++;
        
        if (pathIndex_ >= currentPath_.size()) {
            clearPath();
            setState(AgentState::IDLE);
        }
    } else {
        // Set velocity towards target - position will be updated in updatePosition()
        direction = direction.normalize();
        velocity_ = direction * speed_;
        
        // Don't directly update position here - let updatePosition() handle validation
    }
}

GridCoord Agent::getNextPathStep() const {
    if (hasPath()) {
        return currentPath_[pathIndex_];
    }
    return currentGridPos_;
}

void Agent::initializeBehaviorTree() {
    behaviorTree_ = std::make_unique<BehaviorTree>("Agent " + std::to_string(id_) + " Behavior", this);
    
    // Create the main behavior tree structure
    auto rootSelector = behaviorTree_->createSelector("Root Selector");
    
    // Priority 1: Handle avoidance
    auto avoidanceSequence = behaviorTree_->createSequence("Avoidance Sequence");
    auto obstacleAvoidance = behaviorTree_->createCondition("Obstacle Nearby", 
        [this](Agent* agent) { return agent->conditionObstacleNearby(); });
    auto avoidObstacleAction = behaviorTree_->createAction("Avoid Obstacle",
        [this](Agent* agent) { return agent->actionAvoidObstacle(); });
    
    avoidanceSequence->addChild(obstacleAvoidance);
    avoidanceSequence->addChild(avoidObstacleAction);
    
    // Priority 2: Agent avoidance
    auto agentAvoidanceSequence = behaviorTree_->createSequence("Agent Avoidance Sequence");
    auto agentNearby = behaviorTree_->createCondition("Agent Nearby",
        [this](Agent* agent) { return agent->conditionAgentNearby(); });
    auto avoidAgentAction = behaviorTree_->createAction("Avoid Agent",
        [this](Agent* agent) { return agent->actionAvoidAgent(); });
    
    agentAvoidanceSequence->addChild(agentNearby);
    agentAvoidanceSequence->addChild(avoidAgentAction);
    
    // Priority 3: Patrol behavior
    auto patrolBehavior = createPatrolBehavior();
    
    // Priority 4: Idle/Wait
    auto waitAction = behaviorTree_->createAction("Wait",
        [this](Agent* agent) { return agent->actionWait(); });
    
    // Assemble the tree
    rootSelector->addChild(avoidanceSequence);
    rootSelector->addChild(agentAvoidanceSequence);
    rootSelector->addChild(patrolBehavior);
    rootSelector->addChild(waitAction);
    
    behaviorTree_->setRoot(rootSelector);
}

std::shared_ptr<BehaviorNode> Agent::createPatrolBehavior() {
    auto patrolRepeater = behaviorTree_->createRepeater("Patrol Repeater", nullptr, -1);
    auto patrolSequence = behaviorTree_->createSequence("Patrol Sequence");
    
    auto isPatrollingCondition = behaviorTree_->createCondition("Is Patrolling",
        [this](Agent* agent) { return agent->conditionIsPatrolling(); });
    
    auto patrolAction = behaviorTree_->createAction("Patrol Action",
        [this](Agent* agent) { return agent->actionPatrol(); });
    
    patrolSequence->addChild(isPatrollingCondition);
    patrolSequence->addChild(patrolAction);
    
    patrolRepeater->setChild(patrolSequence);
    
    return patrolRepeater;
}

std::shared_ptr<BehaviorNode> Agent::createAvoidanceBehavior() {
    // This could be expanded with more sophisticated avoidance behaviors
    auto avoidanceSelector = behaviorTree_->createSelector("Avoidance Selector");
    
    return avoidanceSelector;
}

std::shared_ptr<BehaviorNode> Agent::createMovementBehavior() {
    // This could be expanded with more movement behaviors
    auto movementSequence = behaviorTree_->createSequence("Movement Sequence");
    
    return movementSequence;
}

bool Agent::isAgentInRange(const Agent* other, double range) const {
    if (!other) return false;
    
    double distance = Utils::euclideanDistance(position_, other->getPosition());
    return distance <= range;
}

Utils::Vector2D Agent::calculateSeparationForce(const std::vector<Agent*>& nearbyAgents) const {
    Utils::Vector2D separation(0.0, 0.0);
    
    for (const auto* agent : nearbyAgents) {
        if (agent && agent != this) {
            Utils::Vector2D diff = position_ - agent->getPosition();
            double distance = diff.magnitude();
            
            if (distance > 0.1) {
                diff = diff.normalize();
                separation = separation + diff;
            }
        }
    }
    
    return separation.normalize();
}

Utils::Vector2D Agent::calculateCohesionForce(const std::vector<Agent*>& nearbyAgents) const {
    Utils::Vector2D centerOfMass(0.0, 0.0);
    int count = 0;
    
    for (const auto* agent : nearbyAgents) {
        if (agent && agent != this) {
            centerOfMass = centerOfMass + agent->getPosition();
            count++;
        }
    }
    
    if (count > 0) {
        centerOfMass = centerOfMass * (1.0 / count);
        return (centerOfMass - position_).normalize();
    }
    
    return Utils::Vector2D(0.0, 0.0);
}

Utils::Vector2D Agent::calculateAlignmentForce(const std::vector<Agent*>& nearbyAgents) const {
    Utils::Vector2D averageVelocity(0.0, 0.0);
    int count = 0;
    
    for (const auto* agent : nearbyAgents) {
        if (agent && agent != this) {
            averageVelocity = averageVelocity + agent->velocity_;
            count++;
        }
    }
    
    if (count > 0) {
        averageVelocity = averageVelocity * (1.0 / count);
        return averageVelocity.normalize();
    }
    
    return Utils::Vector2D(0.0, 0.0);
}
