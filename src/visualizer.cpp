#include "visualizer.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>

TerminalVisualizer::TerminalVisualizer(int width, int height, bool colors) 
    : width_(width), height_(height), useColors_(colors) {
    // Hide cursor for smoother animation
    std::cout << "\033[?25l";
    std::cout.flush();
}

TerminalVisualizer::~TerminalVisualizer() {
    // Restore cursor on exit
    std::cout << "\033[?25h";
    std::cout.flush();
}

void TerminalVisualizer::clearScreen() {
    std::cout << "\033[2J\033[H";  // Clear entire screen and move cursor to top-left
    std::cout.flush();
}

void TerminalVisualizer::moveCursor(int x, int y) {
    std::cout << "\033[" << (y + 1) << ";" << (x + 1) << "H";
}

std::string TerminalVisualizer::getAgentSymbol(int agentId, AgentState state) {
    if (!useColors_) return "A" + std::to_string(agentId);
    
    // Different colors and symbols based on state
    std::string color;
    std::string symbol;
    
    switch (state) {
        case AgentState::PATROLLING:
            color = "\033[1;32m";  // Bright green
            symbol = "●";
            break;
        case AgentState::AVOIDING_OBSTACLE:
            color = "\033[1;31m";  // Bright red  
            symbol = "◆";
            break;
        case AgentState::AVOIDING_AGENT:
            color = "\033[1;33m";  // Bright yellow
            symbol = "▲";
            break;
        case AgentState::MOVING:
            color = "\033[1;36m";  // Cyan
            symbol = "●";
            break;
        case AgentState::PLANNING_PATH:
            color = "\033[1;35m";  // Magenta
            symbol = "◐";
            break;
        default:
            color = "\033[1;37m";  // White
            symbol = "○";
    }
    
    return color + symbol + std::to_string(agentId) + "\033[0m";
}

std::string TerminalVisualizer::getTrailSymbol(int intensity) {
    if (!useColors_) return "·";
    
    // Fading trail from bright to dim
    int colorCode = 30 + (intensity % 8);  // Gray scale
    return "\033[0;" + std::to_string(colorCode) + "m·\033[0m";
}

void TerminalVisualizer::render(const Grid& grid, const std::vector<std::shared_ptr<Agent>>& agents, int step, double fps) {
    // Position cursor at top-left and clear from cursor to end of screen
    std::cout << "\033[H\033[J";
    
    // Build complete output in a string buffer first
    std::ostringstream output;
    
    // Print title and metrics at the top
    output << "\033[1;36m═══ Multi-Agent Patrol Simulation ═══\033[0m\n";
    output << "Step: \033[1;33m" << step << "\033[0m | FPS: \033[1;32m" 
           << std::fixed << std::setprecision(1) << fps << "\033[0m\n\n";
    
    // Create a buffer to draw everything
    std::vector<std::string> display(grid.getHeight());
    for (int i = 0; i < grid.getHeight(); ++i) {
        display[i] = std::string(grid.getWidth() * 2, ' ');
    }
    
    // Draw grid background
    for (int y = 0; y < grid.getHeight(); ++y) {
        for (int x = 0; x < grid.getWidth(); ++x) {
            GridCoord coord(x, y);
            std::string cell;
            
            if (grid.isObstacle(coord)) {
                cell = useColors_ ? "\033[0;37m██\033[0m" : "██";
            } else if (grid.isWaypoint(coord)) {
                cell = useColors_ ? "\033[1;33m◆ \033[0m" : "W ";
            } else {
                cell = useColors_ ? "\033[0;90m· \033[0m" : ". ";
            }
            
            display[y].replace(x * 2, 2, cell);
        }
    }
    
    // Draw agent trails (fading effect)
    for (const auto& agent : agents) {
        const auto& trail = agent->getTrail();
        for (size_t i = 0; i < trail.size() - 1; ++i) {  // Skip current position
            int x = static_cast<int>(trail[i].x);
            int y = static_cast<int>(trail[i].y);
            
            if (x >= 0 && x < grid.getWidth() && y >= 0 && y < grid.getHeight()) {
                int intensity = (i * 6) / std::max(1, static_cast<int>(trail.size()));
                std::string trailChar = getTrailSymbol(intensity + 2);
                // Replace only the first character to avoid corrupting adjacent cells
                display[y].replace(x * 2, 1, trailChar.substr(0, 1));
            }
        }
    }
    
    // Draw planned paths as dashed lines
    for (const auto& agent : agents) {
        const auto& path = agent->getCurrentPath();
        for (size_t i = agent->getPathIndex() + 1; i < path.size(); ++i) {  // Skip current step
            int x = path[i].x;
            int y = path[i].y;
            
            if (x >= 0 && x < grid.getWidth() && y >= 0 && y < grid.getHeight()) {
                std::string pathChar = useColors_ ? "\033[0;35m-\033[0m" : "-";
                // Only draw if cell is relatively empty
                if (display[y].substr(x * 2, 1) == "·" || display[y].substr(x * 2, 1) == ".") {
                    display[y].replace(x * 2, 1, pathChar);
                }
            }
        }
    }
    
    // Draw current agent positions (on top)
    for (const auto& agent : agents) {
        Utils::Vector2D pos = agent->getPosition();
        int x = static_cast<int>(pos.x);
        int y = static_cast<int>(pos.y);
        
        if (x >= 0 && x < grid.getWidth() && y >= 0 && y < grid.getHeight()) {
            std::string agentSymbol = getAgentSymbol(agent->getId(), agent->getState());
            // Always replace exactly 2 characters (visual space) regardless of ANSI codes
            display[y].replace(x * 2, 2, agentSymbol);
        }
    }
    
    // Draw border and output the complete frame
    output << "+" << std::string(grid.getWidth() * 2, '-') << "+\n";
    for (const auto& line : display) {
        output << "|" << line << "|\n";
    }
    output << "+" << std::string(grid.getWidth() * 2, '-') << "+\n";
    
    // Show agent status inline
    output << "\n\033[1;37mAgents:\033[0m ";
    for (const auto& agent : agents) {
        output << "\033[1;32m" << agent->getId() << "\033[0m:" 
               << agent->getStateString().substr(0, 3) << " "
               << "(" << std::fixed << std::setprecision(1) 
               << agent->getPosition().x << "," << agent->getPosition().y << ") ";
    }
    output << "\n";
    
    // Compact legend
    if (useColors_) {
        output << "\033[0;90mLegend: \033[1;32m●\033[0mPatrol \033[1;31m◆\033[0mAvoidObs \033[1;33m▲\033[0mAvoidAgent \033[0;35m-\033[0mPath \033[1;33m◆\033[0mWaypoint\033[0m\n";
    }
    
    // Output everything at once with no extra lines
    std::cout << output.str();
    std::cout.flush();  // Ensure immediate output
}


