#pragma once
#include "grid.hpp"
#include "agent.hpp"
#include <string>
#include <vector>
#include <memory>

class TerminalVisualizer {
private:
    int width_, height_;
    bool useColors_;
    
public:
    TerminalVisualizer(int width, int height, bool colors = true);
    ~TerminalVisualizer();
    
    void render(const Grid& grid, const std::vector<std::shared_ptr<Agent>>& agents, int step, double fps);
    void clearScreen();
    
private:
    std::string getAgentSymbol(int agentId, AgentState state);
    std::string getTrailSymbol(int intensity);
    void moveCursor(int x, int y);
};
