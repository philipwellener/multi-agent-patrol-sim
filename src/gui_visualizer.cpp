#include "gui_visualizer.hpp"
#include "utils.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>

GUIVisualizer::GUIVisualizer(int windowWidth, int windowHeight)
    : window(sf::VideoMode({static_cast<unsigned int>(windowWidth), static_cast<unsigned int>(windowHeight)}), "Multi-Agent Patrol Simulation", 
             sf::Style::Titlebar | sf::Style::Close),
      stepText(font),
      fpsText(font),
      agentText(font),
      cellSize(25.0f),
      gridOffset(162.5f, 125.0f),  // Adjusted for 35x18 grid in 1200x700 window
      maxTrailLength(20),
      backgroundColor(sf::Color(20, 20, 30)),
      obstacleColor(sf::Color(80, 80, 80)),
      waypointColor(sf::Color(255, 215, 0, 180)),
      gridLineColor(sf::Color(60, 60, 80, 100)),
      patrolColor(sf::Color(0, 255, 0)),
      avoidObstacleColor(sf::Color(255, 255, 0)),
      avoidAgentColor(sf::Color(255, 100, 100))
{
    window.setFramerateLimit(60);
    loadFont();
    
    // Initialize UI text
    stepText.setCharacterSize(20);
    stepText.setFillColor(sf::Color::White);
    stepText.setPosition({10, 10});

    fpsText.setCharacterSize(20);
    fpsText.setFillColor(sf::Color::White);
    fpsText.setPosition({10, 35});

    agentText.setCharacterSize(16);
    agentText.setFillColor(sf::Color::White);
    agentText.setPosition({10, 65});
}

GUIVisualizer::~GUIVisualizer() = default;

bool GUIVisualizer::isWindowOpen() const {
    return window.isOpen();
}

void GUIVisualizer::handleEvents() {
    while (auto event = window.pollEvent()) {
        if (event->is<sf::Event::Closed>()) {
            window.close();
        }
        else if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>()) {
            if (keyPressed->code == sf::Keyboard::Key::Escape) {
                window.close();
            }
        }
    }
}

void GUIVisualizer::render(const Grid& grid, const std::vector<std::unique_ptr<Agent>>& agents,
                          int step, float fps) {
    window.clear(backgroundColor);
    
    updateTrails(agents);
    renderGrid(grid);
    renderTrails();
    renderAgents(agents);
    renderUI(step, fps, agents);
    
    window.display();
}

void GUIVisualizer::setCellSize(float size) {
    cellSize = size;
}

void GUIVisualizer::setTrailLength(size_t length) {
    maxTrailLength = length;
}

sf::Vector2f GUIVisualizer::gridToScreen(int x, int y) const {
    return sf::Vector2f(gridOffset.x + x * cellSize, gridOffset.y + y * cellSize);
}

sf::Color GUIVisualizer::getAgentColor(AgentState state) const {
    switch (state) {
        case AgentState::PATROLLING:
            return patrolColor;
        case AgentState::AVOIDING_OBSTACLE:
            return avoidObstacleColor;
        case AgentState::AVOIDING_AGENT:
            return avoidAgentColor;
        default:
            return sf::Color::White;
    }
}

sf::Color GUIVisualizer::getTrailColor(const sf::Color& baseColor, float alpha) const {
    sf::Color trailColor = baseColor;
    trailColor.a = static_cast<unsigned char>(alpha * 255);
    return trailColor;
}

void GUIVisualizer::updateTrails(const std::vector<std::unique_ptr<Agent>>& agents) {
    // Update trails for each agent
    for (const auto& agent : agents) {
        int agentId = agent->getId();
        auto& trail = agentTrails[agentId];
        
        // Add current position to trail
        Utils::Vector2D pos = agent->getPosition();
        sf::Vector2f currentPos = gridToScreen(
            static_cast<int>(pos.x), 
            static_cast<int>(pos.y)
        );
        
        // Only add if position has changed significantly
        bool addPoint = trail.empty();
        if (!trail.empty()) {
            sf::Vector2f lastPos = trail.back().position;
            float distance = std::sqrt(
                (currentPos.x - lastPos.x) * (currentPos.x - lastPos.x) +
                (currentPos.y - lastPos.y) * (currentPos.y - lastPos.y)
            );
            addPoint = distance > cellSize * 0.3f; // Only add if moved at least 30% of a cell
        }
        
        if (addPoint) {
            TrailPoint point;
            point.position = currentPos;
            point.color = getAgentColor(agent->getState());
            point.alpha = 1.0f;
            point.age = 0;
            trail.push_back(point);
        }
        
        // Age existing trail points and remove old ones
        for (auto& point : trail) {
            point.age++;
            point.alpha = 1.0f - (static_cast<float>(point.age) / static_cast<float>(maxTrailLength));
        }
        
        // Remove old trail points
        while (trail.size() > maxTrailLength) {
            trail.pop_front();
        }
    }
}

void GUIVisualizer::renderGrid(const Grid& grid) {
    int width = grid.getWidth();
    int height = grid.getHeight();
    
    // Draw grid lines
    for (int x = 0; x <= width; ++x) {
        sf::Vertex line[] = {
            sf::Vertex{gridToScreen(x, 0), gridLineColor},
            sf::Vertex{gridToScreen(x, height), gridLineColor}
        };
        window.draw(line, 2, sf::PrimitiveType::Lines);
    }
    
    for (int y = 0; y <= height; ++y) {
        sf::Vertex line[] = {
            sf::Vertex{gridToScreen(0, y), gridLineColor},
            sf::Vertex{gridToScreen(width, y), gridLineColor}
        };
        window.draw(line, 2, sf::PrimitiveType::Lines);
    }
    
    // Draw obstacles and waypoints
    sf::RectangleShape cell(sf::Vector2f(cellSize - 2, cellSize - 2));
    sf::CircleShape waypoint(cellSize * 0.3f);
    waypoint.setOrigin({cellSize * 0.3f, cellSize * 0.3f});
    waypoint.setFillColor(waypointColor);
    
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            sf::Vector2f screenPos = gridToScreen(x, y);
            GridCoord coord(x, y);
            
            if (grid.isObstacle(coord)) {
                cell.setPosition({screenPos.x + 1, screenPos.y + 1});
                cell.setFillColor(obstacleColor);
                window.draw(cell);
            }
            
            if (grid.isWaypoint(coord)) {
                waypoint.setPosition({screenPos.x + cellSize * 0.5f, screenPos.y + cellSize * 0.5f});
                window.draw(waypoint);
            }
        }
    }
}

void GUIVisualizer::renderTrails() {
    sf::CircleShape trailPoint(3.0f);
    trailPoint.setOrigin({3.0f, 3.0f});
    
    for (const auto& [agentId, trail] : agentTrails) {
        for (const auto& point : trail) {
            if (point.alpha > 0.1f) {
                trailPoint.setPosition({point.position.x + cellSize * 0.5f, 
                                     point.position.y + cellSize * 0.5f});
                trailPoint.setFillColor(getTrailColor(point.color, point.alpha));
                window.draw(trailPoint);
            }
        }
    }
}

void GUIVisualizer::renderAgents(const std::vector<std::unique_ptr<Agent>>& agents) {
    sf::CircleShape agentShape(cellSize * 0.4f);
    agentShape.setOrigin({cellSize * 0.4f, cellSize * 0.4f});
    
    sf::Text agentIdText(font);
    agentIdText.setCharacterSize(12);
    agentIdText.setFillColor(sf::Color::Black);
    
    for (const auto& agent : agents) {
        Utils::Vector2D pos = agent->getPosition();
        sf::Vector2f screenPos = gridToScreen(
            static_cast<int>(pos.x), 
            static_cast<int>(pos.y)
        );
        
        // Calculate smooth position for sub-grid movement
        float fractionalX = pos.x - static_cast<int>(pos.x);
        float fractionalY = pos.y - static_cast<int>(pos.y);
        sf::Vector2f smoothPos(
            screenPos.x + fractionalX * cellSize + cellSize * 0.5f,
            screenPos.y + fractionalY * cellSize + cellSize * 0.5f
        );
        
        // Draw agent circle
        agentShape.setPosition(smoothPos);
        agentShape.setFillColor(getAgentColor(agent->getState()));
        agentShape.setOutlineThickness(2.0f);
        agentShape.setOutlineColor(sf::Color::White);
        window.draw(agentShape);
        
        // Draw agent ID
        agentIdText.setString(std::to_string(agent->getId()));
        sf::FloatRect textBounds = agentIdText.getLocalBounds();
        agentIdText.setPosition({
            smoothPos.x - textBounds.size.x * 0.5f,
            smoothPos.y - textBounds.size.y * 0.5f - 2
        });
        window.draw(agentIdText);
    }
}

void GUIVisualizer::renderUI(int step, float fps, const std::vector<std::unique_ptr<Agent>>& agents) {
    // Step counter
    stepText.setString("Step: " + std::to_string(step));
    window.draw(stepText);
    
    // FPS counter
    std::ostringstream fpsStream;
    fpsStream << "FPS: " << std::fixed << std::setprecision(1) << fps;
    fpsText.setString(fpsStream.str());
    window.draw(fpsText);
    
    // Agent status
    std::ostringstream agentStream;
    agentStream << "Agents: ";
    for (size_t i = 0; i < agents.size(); ++i) {
        if (i > 0) agentStream << ", ";
        agentStream << agents[i]->getId() << ":";
        switch (agents[i]->getState()) {
            case AgentState::PATROLLING:
                agentStream << "PAT";
                break;
            case AgentState::AVOIDING_OBSTACLE:
                agentStream << "AVO";
                break;
            case AgentState::AVOIDING_AGENT:
                agentStream << "AVA";
                break;
            default:
                agentStream << "UNK";
                break;
        }
    }
    agentText.setString(agentStream.str());
    window.draw(agentText);
    
    // Legend
    sf::Text legend(font);
    legend.setCharacterSize(14);
    legend.setFillColor(sf::Color::White);
    legend.setPosition({10, static_cast<float>(window.getSize().y - 80)});
    legend.setString("Legend:\nGreen = Patrolling\nYellow = Avoiding Obstacle\nRed = Avoiding Agent\nGold = Waypoints");
    window.draw(legend);
}

void GUIVisualizer::loadFont() {
    // Try multiple macOS system font paths
    std::vector<std::string> fontPaths = {
        "/System/Library/Fonts/Helvetica.ttc",
        "/System/Library/Fonts/Arial.ttf",
        "/Library/Fonts/Arial.ttf",
        "/System/Library/Fonts/Times.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "C:/Windows/Fonts/arial.ttf"
    };
    
    bool fontLoaded = false;
    for (const auto& path : fontPaths) {
        if (font.openFromFile(path)) {
            std::cout << "Successfully loaded font: " << path << std::endl;
            fontLoaded = true;
            break;
        }
    }
    
    if (!fontLoaded) {
        std::cout << "ERROR: Could not load any system font! Text will not be visible." << std::endl;
        std::cout << "Available font paths attempted:" << std::endl;
        for (const auto& path : fontPaths) {
            std::cout << "  - " << path << std::endl;
        }
    }
}
