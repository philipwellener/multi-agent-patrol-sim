#include "utils.hpp"
#include <cmath>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <algorithm>

namespace Utils {
    
    // Vector2D implementations
    double Vector2D::magnitude() const {
        return std::sqrt(x * x + y * y);
    }
    
    double Vector2D::distance(const Vector2D& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    Vector2D Vector2D::normalize() const {
        double mag = magnitude();
        if (mag > 0.0) {
            return Vector2D(x / mag, y / mag);
        }
        return Vector2D(0.0, 0.0);
    }
    
    // RandomGenerator implementations
    RandomGenerator::RandomGenerator() : generator(std::random_device{}()) {}
    
    RandomGenerator::RandomGenerator(unsigned int seed) : generator(seed) {}
    
    int RandomGenerator::randomInt(int min, int max) const {
        std::uniform_int_distribution<int> dist(min, max);
        return dist(generator);
    }
    
    double RandomGenerator::randomDouble(double min, double max) const {
        std::uniform_real_distribution<double> dist(min, max);
        return dist(generator);
    }
    
    bool RandomGenerator::randomBool(double probability) const {
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        return dist(generator) < probability;
    }
    
    // Timer implementations
    Timer::Timer() {
        reset();
    }
    
    void Timer::reset() {
        start_time = std::chrono::high_resolution_clock::now();
    }
    
    double Timer::elapsed() const {
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time);
        return duration.count() / 1000000.0;
    }
    
    // Distance functions
    double manhattanDistance(const GridCoord& a, const GridCoord& b) {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    }
    
    double euclideanDistance(const GridCoord& a, const GridCoord& b) {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    double euclideanDistance(const Vector2D& a, const Vector2D& b) {
        return a.distance(b);
    }
    
    // String formatting functions
    std::string formatTime(double seconds) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << seconds << "s";
        return oss.str();
    }
    
    std::string formatPosition(const Vector2D& pos) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << "(" << pos.x << ", " << pos.y << ")";
        return oss.str();
    }
    
    std::string formatGridCoord(const GridCoord& coord) {
        return "(" + std::to_string(coord.x) + ", " + std::to_string(coord.y) + ")";
    }
    
    // Math utilities
    double clamp(double value, double min, double max) {
        return std::max(min, std::min(max, value));
    }
    
    double lerp(double a, double b, double t) {
        return a + t * (b - a);
    }
    
    int sign(double value) {
        if (value > 0.0) return 1;
        if (value < 0.0) return -1;
        return 0;
    }
    
    // Grid utilities
    bool isValidGridCoord(const GridCoord& coord, int width, int height) {
        return coord.x >= 0 && coord.x < width && coord.y >= 0 && coord.y < height;
    }
    
    std::vector<GridCoord> getNeighbors(const GridCoord& coord, int width, int height) {
        std::vector<GridCoord> neighbors;
        const int dx[] = {0, 1, 0, -1};  // Right, Down, Left, Up
        const int dy[] = {1, 0, -1, 0};
        
        for (int i = 0; i < 4; ++i) {
            GridCoord neighbor(coord.x + dx[i], coord.y + dy[i]);
            if (isValidGridCoord(neighbor, width, height)) {
                neighbors.push_back(neighbor);
            }
        }
        
        return neighbors;
    }
    
    std::vector<GridCoord> getNeighbors8(const GridCoord& coord, int width, int height) {
        std::vector<GridCoord> neighbors;
        
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;  // Skip the center cell
                
                GridCoord neighbor(coord.x + dx, coord.y + dy);
                if (isValidGridCoord(neighbor, width, height)) {
                    neighbors.push_back(neighbor);
                }
            }
        }
        
        return neighbors;
    }
}
