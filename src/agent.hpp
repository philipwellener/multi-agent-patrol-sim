#pragma once

#include "utils.hpp"
#include "grid.hpp"
#include "pathfinding.hpp"
#include "behavior_tree.hpp"
#include "config.hpp"
#include <memory>
#include <vector>
#include <string>
#include <deque>

class Grid;
class Pathfinder;
class BehaviorTree;

using Utils::GridCoord;

// TODO: maybe add COMBAT state later?
enum class AgentState {
    IDLE,
    PATROLLING,
    PLANNING_PATH,
    MOVING,
    AVOIDING_OBSTACLE,
    AVOIDING_AGENT,
    REROUTING,
    WAITING
};

class Agent {
private:
    int id_;
    Utils::Vector2D position_;
    Utils::Vector2D velocity_;
    GridCoord currentGridPos_;
    GridCoord targetGridPos_;
    
    AgentState state_;
    double speed_;
    double visionRange_;
    
    // path stuff
    std::vector<GridCoord> currentPath_;
    size_t pathIndex_;
    bool hasDestination_;
    
    // patrol stuff
    std::vector<GridCoord> patrolWaypoints_;
    size_t currentWaypointIndex_;
    bool isPatrolling_;
    
    // stuck detection and cycling prevention
    mutable double stuckTimer_;
    mutable Utils::Vector2D lastPosition_;
    
    // waypoint attempt cooldown to prevent rapid cycling
    mutable double waypointAttemptTimer_;
    mutable int lastFailedWaypointIndex_;
    
    // Track waypoint visit history to prevent immediate revisiting
    mutable std::vector<size_t> recentWaypointHistory_;
    mutable double lastWaypointReachedTime_;
    
    // Track consecutive pathfinding failures
    mutable int consecutivePathFailures_;
    mutable double lastPathFailureTime_;
    
    // Position oscillation detection
    mutable std::vector<Utils::Vector2D> recentPositions_;
    mutable double positionHistoryTimer_;
    mutable double lastOscillationDetectionTime_;
    
    std::unique_ptr<BehaviorTree> behaviorTree_;
    
    Grid* grid_;
    std::unique_ptr<Pathfinder> pathfinder_;
    
    Utils::Timer updateTimer_;
    double lastUpdateTime_;
    
    // FIXME: this feels hacky, maybe refactor later
    std::vector<int> detectedAgents_;
    Utils::Vector2D avoidanceVector_;
    
    // trail tracking for visualization
    std::deque<Utils::Vector2D> trail_;
    static constexpr int TRAIL_LENGTH = 8;
    
public:
    Agent(int id, const Utils::Vector2D& startPos, Grid* grid);
    ~Agent() = default;
    
    int getId() const { return id_; }
    Utils::Vector2D getPosition() const { return position_; }
    GridCoord getGridPosition() const { return currentGridPos_; }
    AgentState getState() const { return state_; }
    double getSpeed() const { return speed_; }
    
    void setPosition(const Utils::Vector2D& pos);
    void setGridPosition(const GridCoord& pos);
    void setSpeed(double speed) { speed_ = Utils::clamp(speed, 0.0, 10.0); }
    
    void update(double deltaTime);
    void updateTrail();
    
    bool setDestination(const GridCoord& destination);
    bool hasPath() const { return !currentPath_.empty() && pathIndex_ < currentPath_.size(); }
    void clearPath();
    GridCoord getCurrentTarget() const;
    GridCoord getNextWaypoint() const;
    
    void setPatrolWaypoints(const std::vector<GridCoord>& waypoints);
    void addPatrolWaypoint(const GridCoord& waypoint);
    void startPatrol();
    void stopPatrol();
    bool isPatrolling() const { return isPatrolling_; }
    
    // agent detection stuff - kinda messy but works
    std::vector<GridCoord> detectNearbyAgents(const std::vector<Agent*>& allAgents) const;
    void updateAgentDetection(const std::vector<Agent*>& allAgents);
    Utils::Vector2D calculateAvoidanceVector(const std::vector<Agent*>& nearbyAgents) const;
    
    std::vector<GridCoord> detectNearbyObstacles() const;
    bool isPathBlocked() const;
    
    // behavior tree callbacks
    NodeStatus actionMoveTo(const GridCoord& target);
    NodeStatus actionPatrol();
    NodeStatus actionAvoidObstacle();
    NodeStatus actionAvoidAgent();
    NodeStatus actionWait();
    NodeStatus actionPlanPath(const GridCoord& destination);
    
    bool conditionHasDestination() const;
    bool conditionAtDestination() const;
    bool conditionPathBlocked() const;
    bool conditionAgentNearby() const;
    bool conditionObstacleNearby() const;
    bool conditionIsPatrolling() const;
    
    void setState(AgentState state) { state_ = state; }
    std::string getStateString() const;
    
    // visualization support
    const std::deque<Utils::Vector2D>& getTrail() const { return trail_; }
    const std::vector<GridCoord>& getCurrentPath() const { return currentPath_; }
    size_t getPathIndex() const { return pathIndex_; }
    
    std::string toString() const;
    std::string getDebugInfo() const;
    void printStatus() const;
    
    double distanceToTarget() const;
    bool isAtPosition(const GridCoord& pos, double threshold = Config::WAYPOINT_REACHED_THRESHOLD) const;
    GridCoord selectRandomWaypoint() const;
    
private:
    void updatePosition(double deltaTime);
    void updateBehaviorTree();
    void updateGridPosition();
    void forceUnstuck();
    
    bool planPathTo(const GridCoord& destination);
    void followPath(double deltaTime);
    GridCoord getNextPathStep() const;
    
    void initializeBehaviorTree();
    std::shared_ptr<BehaviorNode> createPatrolBehavior();
    std::shared_ptr<BehaviorNode> createAvoidanceBehavior();
    std::shared_ptr<BehaviorNode> createMovementBehavior();
    
    bool isAgentInRange(const Agent* other, double range) const;
    // these three methods implement basic flocking behavior
    Utils::Vector2D calculateSeparationForce(const std::vector<Agent*>& nearbyAgents) const;
    Utils::Vector2D calculateCohesionForce(const std::vector<Agent*>& nearbyAgents) const;
    Utils::Vector2D calculateAlignmentForce(const std::vector<Agent*>& nearbyAgents) const;
};
