#include "config.hpp"
#include "grid.hpp"
#include "agent.hpp"
#include "utils.hpp"
#include "visualizer.hpp"
#include "gui_visualizer.hpp"
#include <iostream>
#include <vector>
#include <memory>
#include <thread>
#include <chrono>
#include <map>
#include <signal.h>

class MultiAgentSimulation {
private:
    std::unique_ptr<Grid> grid_;
    std::vector<std::shared_ptr<Agent>> agents_;
    Utils::Timer simulationTimer_;
    std::unique_ptr<TerminalVisualizer> terminalVisualizer_;
    std::unique_ptr<GUIVisualizer> guiVisualizer_;
    int currentStep_;
    bool isRunning_;
    bool useGUI_;
    
public:
    MultiAgentSimulation(bool useGUI = true) 
        : grid_(std::make_unique<Grid>()), 
          currentStep_(0), 
          isRunning_(false),
          useGUI_(useGUI) {
        
        if (useGUI_) {
            guiVisualizer_ = std::make_unique<GUIVisualizer>(1200, 700);  // Reduced height from 900 to 700
        } else {
            terminalVisualizer_ = std::make_unique<TerminalVisualizer>(Config::GRID_WIDTH, Config::GRID_HEIGHT, true);
        }
        
        initializeSimulation();
    }
    
    ~MultiAgentSimulation() = default;
    
    void initializeSimulation() {
        std::cout << "Initializing Multi-Agent Patrol Simulation..." << std::endl;
        
        Utils::RandomGenerator rng;
        
        for (int i = 0; i < Config::NUM_AGENTS; ++i) {
            // keep trying until we find a good spot
            Utils::GridCoord startPos;
            int attempts = 0;
            
            do {
                int x = rng.randomInt(1, grid_->getWidth() - 2);
                int y = rng.randomInt(1, grid_->getHeight() - 2);
                startPos = Utils::GridCoord(x, y);
                attempts++;
            } while (!grid_->isTraversable(startPos) && attempts < 100);
            
            // Create agent
            auto agent = std::make_shared<Agent>(i, startPos.toVector2D(), grid_.get());
            
            // Set up patrol waypoints for the agent
            std::vector<Utils::GridCoord> patrolWaypoints;
            const auto& gridWaypoints = grid_->getWaypoints();
            
            if (!gridWaypoints.empty()) {
                // All agents patrol ALL waypoints - this creates proper patrol circuits
                patrolWaypoints = gridWaypoints;
                
                // Optional: Start each agent at a different waypoint in the circuit
                // to spread them out initially
                if (patrolWaypoints.size() > 1) {
                    // Rotate the waypoint list so each agent starts at a different point
                    int startOffset = i % patrolWaypoints.size();
                    std::rotate(patrolWaypoints.begin(), 
                              patrolWaypoints.begin() + startOffset, 
                              patrolWaypoints.end());
                }
            }
            
            agent->setPatrolWaypoints(patrolWaypoints);
            agent->startPatrol();
            
            agents_.push_back(agent);
            
            std::cout << "Created Agent " << i << " at " << Utils::formatGridCoord(startPos) 
                      << " with " << patrolWaypoints.size() << " patrol waypoints" << std::endl;
        }
        
        std::cout << "Simulation initialized with " << agents_.size() << " agents" << std::endl;
        std::cout << "Grid size: " << grid_->getWidth() << "x" << grid_->getHeight() << std::endl;
        std::cout << "Obstacles: " << grid_->getObstacleCount() << " (" 
                  << (grid_->getObstacleDensity() * 100.0) << "%)" << std::endl;
        std::cout << "Waypoints: " << grid_->getWaypointCount() << std::endl;
    }
    
    void run() {
        std::cout << "\nStarting simulation..." << std::endl;
        if (!useGUI_) {
            std::cout << "Press Ctrl+C to stop\n" << std::endl;
        }
        
        isRunning_ = true;
        simulationTimer_.reset();
        
        while (isRunning_ && currentStep_ < Config::MAX_SIMULATION_STEPS) {
            // Handle GUI events if using GUI
            if (useGUI_) {
                if (!guiVisualizer_->isWindowOpen()) {
                    break;
                }
                guiVisualizer_->handleEvents();
            }
            
            update();
            render();
            
            // Control simulation speed
            if (Config::ENABLE_VISUALIZATION && !useGUI_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(Config::VISUALIZATION_DELAY_MS));
            } else if (useGUI_) {
                // GUI runs at 60 FPS, so small delay
                std::this_thread::sleep_for(std::chrono::milliseconds(16));
            }
            
            currentStep_++;
        }
        
        std::cout << "\nSimulation completed!" << std::endl;
        printStatistics();
    }
    
    void update() {
        double deltaTime = Config::SIMULATION_TIMESTEP;
        
        // Collect all agent pointers for collision detection
        std::vector<Agent*> allAgents;
        for (auto& agent : agents_) {
            if (agent) {
                allAgents.push_back(agent.get());
            }
        }
        
        // Update all agents
        for (auto& agent : agents_) {
            if (agent) {
                agent->updateAgentDetection(allAgents);
                agent->update(deltaTime);
            }
        }
    }
    
    void render() {
        if (!Config::ENABLE_VISUALIZATION) {
            return;
        }

        // Calculate FPS
        static auto lastFrameTime = std::chrono::high_resolution_clock::now();
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto deltaTime = std::chrono::duration<double>(currentTime - lastFrameTime).count();
        double fps = deltaTime > 0 ? 1.0 / deltaTime : 0.0;
        lastFrameTime = currentTime;

        if (useGUI_) {
            // Convert shared_ptr vector to unique_ptr vector for GUI
            std::vector<std::unique_ptr<Agent>> agentPtrs;
            for (const auto& agent : agents_) {
                agentPtrs.push_back(std::unique_ptr<Agent>(agent.get()));
            }
            
            guiVisualizer_->render(*grid_, agentPtrs, currentStep_, fps);
            
            // Release the unique_ptrs without deleting the objects
            for (auto& ptr : agentPtrs) {
                ptr.release();
            }
        } else {
            terminalVisualizer_->render(*grid_, agents_, currentStep_, fps);
        }
    }
    
    void printStatistics() {
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "SIMULATION STATISTICS" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        
        double totalRuntime = simulationTimer_.elapsed();
        
        std::cout << "Runtime: " << Utils::formatTime(totalRuntime) << std::endl;
        std::cout << "Total Steps: " << currentStep_ << std::endl;
        std::cout << "Steps per Second: " << (currentStep_ / totalRuntime) << std::endl;
        std::cout << "Agents: " << agents_.size() << std::endl;
        
        // Agent statistics
        std::map<std::string, int> stateCount;
        for (const auto& agent : agents_) {
            if (agent) {
                stateCount[agent->getStateString()]++;
            }
        }
        
        std::cout << "\nFinal Agent States:" << std::endl;
        for (const auto& pair : stateCount) {
            std::cout << "  " << pair.first << ": " << pair.second << std::endl;
        }
        
        // Environment statistics
        std::cout << "\nEnvironment:" << std::endl;
        std::cout << "  Grid Size: " << grid_->getWidth() << "x" << grid_->getHeight() << std::endl;
        std::cout << "  Obstacles: " << grid_->getObstacleCount() 
                  << " (" << (grid_->getObstacleDensity() * 100.0) << "%)" << std::endl;
        std::cout << "  Waypoints: " << grid_->getWaypointCount() << std::endl;
    }
    
    void stop() {
        isRunning_ = false;
    }
    
    // Debug functions
    void printAgentDetails() {
        std::cout << "\nDetailed Agent Information:" << std::endl;
        for (const auto& agent : agents_) {
            if (agent) {
                std::cout << agent->getDebugInfo() << std::endl;
            }
        }
    }
    
    void runSingleStep() {
        std::cout << "Running single simulation step..." << std::endl;
        update();
        render();
        currentStep_++;
    }
};

// Signal handler for graceful shutdown
MultiAgentSimulation* g_simulation = nullptr;

void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ". Shutting down gracefully..." << std::endl;
    if (g_simulation) {
        g_simulation->stop();
    }
}

int main(int argc, char* argv[]) {
    std::cout << "Multi-Agent Autonomous Patrol Simulator" << std::endl;
    std::cout << "=======================================" << std::endl;
    std::cout << "A C++ demonstration of autonomous multi-agent coordination" << std::endl;
    std::cout << "using behavior trees and pathfinding algorithms." << std::endl << std::endl;
    
    // Parse command line arguments
    bool useGUI = true;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--terminal" || arg == "-t") {
            useGUI = false;
        } else if (arg == "--gui" || arg == "-g") {
            useGUI = true;
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
            std::cout << "Options:" << std::endl;
            std::cout << "  --gui, -g      Use GUI visualization (default)" << std::endl;
            std::cout << "  --terminal, -t Use terminal visualization" << std::endl;
            std::cout << "  --help, -h     Show this help message" << std::endl;
            return 0;
        }
    }
    
    std::cout << "Visualization Mode: " << (useGUI ? "GUI" : "Terminal") << std::endl;
    if (useGUI) {
        std::cout << "Close the window or press ESC to exit" << std::endl;
    } else {
        std::cout << "Press Ctrl+C to exit" << std::endl;
    }
    std::cout << std::endl;
    
    // Set up signal handling
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    try {
        MultiAgentSimulation simulation(useGUI);
        g_simulation = &simulation;
        
        std::cout << "Configuration:" << std::endl;
        std::cout << "  Grid Size: " << Config::GRID_WIDTH << "x" << Config::GRID_HEIGHT << std::endl;
        std::cout << "  Agents: " << Config::NUM_AGENTS << std::endl;
        std::cout << "  Agent Speed: " << Config::AGENT_SPEED << " cells/second" << std::endl;
        std::cout << "  Vision Range: " << Config::AGENT_VISION_RANGE << " cells" << std::endl;
        std::cout << "  Timestep: " << Config::SIMULATION_TIMESTEP << " seconds" << std::endl;
        std::cout << "  Max Steps: " << Config::MAX_SIMULATION_STEPS << std::endl;
        
        if (Config::ENABLE_VISUALIZATION) {
            if (useGUI) {
                std::cout << "  Visualization: GUI Mode (60 FPS)" << std::endl;
            } else {
                std::cout << "  Visualization: Terminal Mode (" << Config::VISUALIZATION_DELAY_MS << "ms delay)" << std::endl;
            }
        } else {
            std::cout << "  Visualization: Disabled" << std::endl;
        }
        
        std::cout << std::endl;
        
        simulation.run();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
