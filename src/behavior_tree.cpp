#include "behavior_tree.hpp"
#include "agent.hpp"
#include "config.hpp"
#include <iostream>
#include <sstream>

// SequenceNode Implementation
NodeStatus SequenceNode::execute(Agent* agent) {
    for (size_t i = currentChild_; i < children_.size(); ++i) {
        NodeStatus status = children_[i]->execute(agent);
        
        if (status == NodeStatus::FAILURE) {
            reset();
            return NodeStatus::FAILURE;
        }
        
        if (status == NodeStatus::RUNNING) {
            currentChild_ = i;
            return NodeStatus::RUNNING;
        }
        
        // Continue to next child if SUCCESS
    }
    
    reset();
    return NodeStatus::SUCCESS;
}

void SequenceNode::reset() {
    currentChild_ = 0;
    CompositeNode::reset();
}

// SelectorNode Implementation
NodeStatus SelectorNode::execute(Agent* agent) {
    for (size_t i = currentChild_; i < children_.size(); ++i) {
        NodeStatus status = children_[i]->execute(agent);
        
        if (status == NodeStatus::SUCCESS) {
            reset();
            return NodeStatus::SUCCESS;
        }
        
        if (status == NodeStatus::RUNNING) {
            currentChild_ = i;
            return NodeStatus::RUNNING;
        }
        
        // Continue to next child if FAILURE
    }
    
    reset();
    return NodeStatus::FAILURE;
}

void SelectorNode::reset() {
    currentChild_ = 0;
    CompositeNode::reset();
}

// ParallelNode Implementation
NodeStatus ParallelNode::execute(Agent* agent) {
    if (childStates_.size() != children_.size()) {
        childStates_.resize(children_.size(), NodeStatus::RUNNING);
    }
    
    int successCount = 0;
    int failureCount = 0;
    int runningCount = 0;
    
    for (size_t i = 0; i < children_.size(); ++i) {
        if (childStates_[i] != NodeStatus::RUNNING) {
            // Child already completed
            if (childStates_[i] == NodeStatus::SUCCESS) {
                successCount++;
            } else {
                failureCount++;
            }
            continue;
        }
        
        NodeStatus status = children_[i]->execute(agent);
        childStates_[i] = status;
        
        if (status == NodeStatus::SUCCESS) {
            successCount++;
        } else if (status == NodeStatus::FAILURE) {
            failureCount++;
        } else {
            runningCount++;
        }
    }
    
    // Check success threshold
    if (successCount >= successThreshold_) {
        reset();
        return NodeStatus::SUCCESS;
    }
    
    // Check failure threshold
    if (failureCount >= failureThreshold_) {
        reset();
        return NodeStatus::FAILURE;
    }
    
    return NodeStatus::RUNNING;
}

void ParallelNode::reset() {
    childStates_.clear();
    CompositeNode::reset();
}

// InverterNode Implementation
NodeStatus InverterNode::execute(Agent* agent) {
    if (!child_) {
        return NodeStatus::FAILURE;
    }
    
    NodeStatus status = child_->execute(agent);
    
    switch (status) {
        case NodeStatus::SUCCESS:
            return NodeStatus::FAILURE;
        case NodeStatus::FAILURE:
            return NodeStatus::SUCCESS;
        case NodeStatus::RUNNING:
            return NodeStatus::RUNNING;
    }
    
    return NodeStatus::FAILURE;
}

// RepeaterNode Implementation
NodeStatus RepeaterNode::execute(Agent* agent) {
    if (!child_) {
        return NodeStatus::FAILURE;
    }
    
    // Infinite repeats if maxRepeats_ is negative
    if (maxRepeats_ >= 0 && currentRepeats_ >= maxRepeats_) {
        return NodeStatus::SUCCESS;
    }
    
    NodeStatus status = child_->execute(agent);
    
    if (status != NodeStatus::RUNNING) {
        currentRepeats_++;
        child_->reset();
        
        // Check if we've reached the repeat limit
        if (maxRepeats_ >= 0 && currentRepeats_ >= maxRepeats_) {
            reset();
            return NodeStatus::SUCCESS;
        }
        
        // Continue repeating
        return NodeStatus::RUNNING;
    }
    
    return NodeStatus::RUNNING;
}

void RepeaterNode::reset() {
    currentRepeats_ = 0;
    DecoratorNode::reset();
}

// BehaviorTree Implementation
BehaviorTree::BehaviorTree(const std::string& name, Agent* owner)
    : root_(nullptr), owner_(owner), isRunning_(false), name_(name) {}

NodeStatus BehaviorTree::tick() {
    if (!root_) {
        return NodeStatus::FAILURE;
    }
    
    isRunning_ = true;
    NodeStatus status = root_->execute(owner_);
    
    if (status != NodeStatus::RUNNING) {
        isRunning_ = false;
    }
    
    if (Config::DEBUG_BEHAVIOR_TREE && owner_) {
        // Debug output could be added here
    }
    
    return status;
}

void BehaviorTree::reset() {
    if (root_) {
        root_->reset();
    }
    isRunning_ = false;
}

void BehaviorTree::stop() {
    reset();
    isRunning_ = false;
}

std::shared_ptr<SequenceNode> BehaviorTree::createSequence(const std::string& name) {
    return std::make_shared<SequenceNode>(name);
}

std::shared_ptr<SelectorNode> BehaviorTree::createSelector(const std::string& name) {
    return std::make_shared<SelectorNode>(name);
}

std::shared_ptr<ParallelNode> BehaviorTree::createParallel(const std::string& name, 
                                                          int successThreshold, 
                                                          int failureThreshold) {
    return std::make_shared<ParallelNode>(name, successThreshold, failureThreshold);
}

std::shared_ptr<ActionNode> BehaviorTree::createAction(const std::string& name, 
                                                     std::function<NodeStatus(Agent*)> action) {
    return std::make_shared<ActionNode>(name, action);
}

std::shared_ptr<ConditionNode> BehaviorTree::createCondition(const std::string& name, 
                                                           std::function<bool(Agent*)> condition) {
    return std::make_shared<ConditionNode>(name, condition);
}

std::shared_ptr<InverterNode> BehaviorTree::createInverter(const std::string& name, 
                                                         std::shared_ptr<BehaviorNode> child) {
    return std::make_shared<InverterNode>(name, child);
}

std::shared_ptr<RepeaterNode> BehaviorTree::createRepeater(const std::string& name, 
                                                         std::shared_ptr<BehaviorNode> child, 
                                                         int maxRepeats) {
    return std::make_shared<RepeaterNode>(name, child, maxRepeats);
}

std::string BehaviorTree::toString() const {
    if (!root_) {
        return "Empty Behavior Tree: " + name_;
    }
    
    return "Behavior Tree: " + name_ + "\n" + nodeToString(root_);
}

void BehaviorTree::printTree() const {
    std::cout << toString() << std::endl;
}

std::string BehaviorTree::nodeToString(std::shared_ptr<BehaviorNode> node, int depth) const {
    if (!node) {
        return "";
    }
    
    std::string indent(depth * 2, ' ');
    std::ostringstream oss;
    
    oss << indent << "├─ " << node->getType() << ": " << node->getName() << "\n";
    
    // Handle composite nodes
    if (auto composite = std::dynamic_pointer_cast<CompositeNode>(node)) {
        for (size_t i = 0; i < composite->getChildCount(); ++i) {
            oss << nodeToString(composite->getChild(i), depth + 1);
        }
    }
    
    // Handle decorator nodes
    if (auto decorator = std::dynamic_pointer_cast<DecoratorNode>(node)) {
        if (decorator->getChild()) {
            oss << nodeToString(decorator->getChild(), depth + 1);
        }
    }
    
    return oss.str();
}
