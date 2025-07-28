#pragma once
#include <SFML/Graphics.hpp>
#include <memory>
#include <vector>
#include <deque>
#include <unordered_map>
#include "grid.hpp"
#include "agent.hpp"

class GUIVisualizer {
public:
    GUIVisualizer(int windowWidth = 1200, int windowHeight = 900);
    ~GUIVisualizer();

    // Main update and render functions
    bool isWindowOpen() const;
    void handleEvents();
    void render(const Grid& grid, const std::vector<std::unique_ptr<Agent>>& agents, 
                int step, float fps);

    // Configuration
    void setCellSize(float size);
    void setTrailLength(size_t length);

private:
    struct TrailPoint {
        sf::Vector2f position;
        sf::Color color;
        float alpha;
        int age;
    };

    // SFML components
    sf::RenderWindow window;
    sf::Font font;
    sf::Text stepText;
    sf::Text fpsText;
    sf::Text agentText;

    // Rendering parameters
    float cellSize;
    sf::Vector2f gridOffset;
    size_t maxTrailLength;

    // Agent trails
    std::unordered_map<int, std::deque<TrailPoint>> agentTrails;

    // Colors
    sf::Color backgroundColor;
    sf::Color obstacleColor;
    sf::Color waypointColor;
    sf::Color gridLineColor;

    // Agent state colors
    sf::Color patrolColor;
    sf::Color avoidObstacleColor;
    sf::Color avoidAgentColor;

    // Helper functions
    sf::Vector2f gridToScreen(int x, int y) const;
    sf::Color getAgentColor(AgentState state) const;
    sf::Color getTrailColor(const sf::Color& baseColor, float alpha) const;
    void updateTrails(const std::vector<std::unique_ptr<Agent>>& agents);
    void renderGrid(const Grid& grid);
    void renderAgents(const std::vector<std::unique_ptr<Agent>>& agents);
    void renderTrails();
    void renderUI(int step, float fps, const std::vector<std::unique_ptr<Agent>>& agents);
    void loadFont();
};
