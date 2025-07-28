#pragma once

#include "utils.hpp"
#include <memory>
#include <vector>
#include <string>
#include <functional>

// Forward declarations
class Agent;

// Behavior Tree Node States
enum class NodeStatus {
    SUCCESS,
    FAILURE,
    RUNNING
};

// Base Behavior Tree Node
class BehaviorNode {
protected:
    std::string name_;
    
public:
    explicit BehaviorNode(const std::string& name) : name_(name) {}
    virtual ~BehaviorNode() = default;
    
    virtual NodeStatus execute(Agent* agent) = 0;
    virtual void reset() {}
    virtual std::string getName() const { return name_; }
    virtual std::string getType() const = 0;
};

// Composite Nodes
class CompositeNode : public BehaviorNode {
protected:
    std::vector<std::shared_ptr<BehaviorNode>> children_;
    
public:
    explicit CompositeNode(const std::string& name) : BehaviorNode(name) {}
    
    void addChild(std::shared_ptr<BehaviorNode> child) {
        children_.push_back(child);
    }
    
    void removeChild(std::shared_ptr<BehaviorNode> child) {
        children_.erase(
            std::remove(children_.begin(), children_.end(), child),
            children_.end()
        );
    }
    
    void clearChildren() {
        children_.clear();
    }
    
    size_t getChildCount() const { return children_.size(); }
    std::shared_ptr<BehaviorNode> getChild(size_t index) const {
        return (index < children_.size()) ? children_[index] : nullptr;
    }
    
    void reset() override {
        for (auto& child : children_) {
            child->reset();
        }
    }
};

// Sequence Node - Executes children in order until one fails
class SequenceNode : public CompositeNode {
private:
    size_t currentChild_;
    
public:
    explicit SequenceNode(const std::string& name = "Sequence") 
        : CompositeNode(name), currentChild_(0) {}
    
    NodeStatus execute(Agent* agent) override;
    void reset() override;
    std::string getType() const override { return "Sequence"; }
};

// Selector Node - Executes children until one succeeds
class SelectorNode : public CompositeNode {
private:
    size_t currentChild_;
    
public:
    explicit SelectorNode(const std::string& name = "Selector") 
        : CompositeNode(name), currentChild_(0) {}
    
    NodeStatus execute(Agent* agent) override;
    void reset() override;
    std::string getType() const override { return "Selector"; }
};

// Parallel Node - Executes all children simultaneously
class ParallelNode : public CompositeNode {
private:
    int successThreshold_;
    int failureThreshold_;
    std::vector<NodeStatus> childStates_;
    
public:
    explicit ParallelNode(const std::string& name = "Parallel", 
                         int successThreshold = 1, int failureThreshold = 1)
        : CompositeNode(name), successThreshold_(successThreshold), 
          failureThreshold_(failureThreshold) {}
    
    NodeStatus execute(Agent* agent) override;
    void reset() override;
    std::string getType() const override { return "Parallel"; }
};

// Decorator Nodes
class DecoratorNode : public BehaviorNode {
protected:
    std::shared_ptr<BehaviorNode> child_;
    
public:
    DecoratorNode(const std::string& name, std::shared_ptr<BehaviorNode> child)
        : BehaviorNode(name), child_(child) {}
    
    void setChild(std::shared_ptr<BehaviorNode> child) { child_ = child; }
    std::shared_ptr<BehaviorNode> getChild() const { return child_; }
    
    void reset() override {
        if (child_) child_->reset();
    }
};

// Inverter Decorator - Inverts the result of its child
class InverterNode : public DecoratorNode {
public:
    InverterNode(const std::string& name, std::shared_ptr<BehaviorNode> child)
        : DecoratorNode(name, child) {}
    
    NodeStatus execute(Agent* agent) override;
    std::string getType() const override { return "Inverter"; }
};

// Repeater Decorator - Repeats its child a specified number of times
class RepeaterNode : public DecoratorNode {
private:
    int maxRepeats_;
    int currentRepeats_;
    
public:
    RepeaterNode(const std::string& name, std::shared_ptr<BehaviorNode> child, int maxRepeats = -1)
        : DecoratorNode(name, child), maxRepeats_(maxRepeats), currentRepeats_(0) {}
    
    NodeStatus execute(Agent* agent) override;
    void reset() override;
    std::string getType() const override { return "Repeater"; }
};

// Action Nodes (Leaf Nodes)
class ActionNode : public BehaviorNode {
protected:
    std::function<NodeStatus(Agent*)> action_;
    
public:
    ActionNode(const std::string& name, std::function<NodeStatus(Agent*)> action)
        : BehaviorNode(name), action_(action) {}
    
    NodeStatus execute(Agent* agent) override {
        return action_ ? action_(agent) : NodeStatus::FAILURE;
    }
    
    std::string getType() const override { return "Action"; }
};

// Condition Nodes (Leaf Nodes)
class ConditionNode : public BehaviorNode {
protected:
    std::function<bool(Agent*)> condition_;
    
public:
    ConditionNode(const std::string& name, std::function<bool(Agent*)> condition)
        : BehaviorNode(name), condition_(condition) {}
    
    NodeStatus execute(Agent* agent) override {
        if (!condition_) return NodeStatus::FAILURE;
        return condition_(agent) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }
    
    std::string getType() const override { return "Condition"; }
};

// Behavior Tree Class
class BehaviorTree {
private:
    std::shared_ptr<BehaviorNode> root_;
    Agent* owner_;
    bool isRunning_;
    std::string name_;
    
public:
    BehaviorTree(const std::string& name, Agent* owner = nullptr);
    ~BehaviorTree() = default;
    
    // Tree management
    void setRoot(std::shared_ptr<BehaviorNode> root) { root_ = root; }
    std::shared_ptr<BehaviorNode> getRoot() const { return root_; }
    
    void setOwner(Agent* owner) { owner_ = owner; }
    Agent* getOwner() const { return owner_; }
    
    std::string getName() const { return name_; }
    void setName(const std::string& name) { name_ = name; }
    
    // Execution
    NodeStatus tick();
    void reset();
    void stop();
    
    bool isRunning() const { return isRunning_; }
    
    // Tree construction helpers
    std::shared_ptr<SequenceNode> createSequence(const std::string& name = "Sequence");
    std::shared_ptr<SelectorNode> createSelector(const std::string& name = "Selector");
    std::shared_ptr<ParallelNode> createParallel(const std::string& name = "Parallel", 
                                                int successThreshold = 1, int failureThreshold = 1);
    
    std::shared_ptr<ActionNode> createAction(const std::string& name, 
                                           std::function<NodeStatus(Agent*)> action);
    std::shared_ptr<ConditionNode> createCondition(const std::string& name, 
                                                  std::function<bool(Agent*)> condition);
    
    std::shared_ptr<InverterNode> createInverter(const std::string& name, 
                                               std::shared_ptr<BehaviorNode> child);
    std::shared_ptr<RepeaterNode> createRepeater(const std::string& name, 
                                               std::shared_ptr<BehaviorNode> child, 
                                               int maxRepeats = -1);
    
    // Debug and visualization
    std::string toString() const;
    void printTree() const;
    
private:
    std::string nodeToString(std::shared_ptr<BehaviorNode> node, int depth = 0) const;
};
