#pragma once

#include <vector>
#include <string>
#include <random>

namespace Utils {
    // 2D Vector structure for positions and directions
    struct Vector2D {
        double x, y;
        
        Vector2D(double x = 0.0, double y = 0.0) : x(x), y(y) {}
        
        Vector2D operator+(const Vector2D& other) const {
            return Vector2D(x + other.x, y + other.y);
        }
        
        Vector2D operator-(const Vector2D& other) const {
            return Vector2D(x - other.x, y - other.y);
        }
        
        Vector2D operator*(double scalar) const {
            return Vector2D(x * scalar, y * scalar);
        }
        
        double magnitude() const;
        double distance(const Vector2D& other) const;
        Vector2D normalize() const;
        
        // Grid coordinate conversion
        int gridX() const { return static_cast<int>(x); }
        int gridY() const { return static_cast<int>(y); }
    };
    
    // Grid coordinate structure
    struct GridCoord {
        int x, y;
        
        GridCoord(int x = 0, int y = 0) : x(x), y(y) {}
        
        bool operator==(const GridCoord& other) const {
            return x == other.x && y == other.y;
        }
        
        bool operator!=(const GridCoord& other) const {
            return !(*this == other);
        }
        
        Vector2D toVector2D() const {
            return Vector2D(static_cast<double>(x), static_cast<double>(y));
        }
    };
    
    // Random number generation utilities
    class RandomGenerator {
    private:
        mutable std::mt19937 generator;
        
    public:
        RandomGenerator();
        explicit RandomGenerator(unsigned int seed);
        
        int randomInt(int min, int max) const;
        double randomDouble(double min, double max) const;
        bool randomBool(double probability = 0.5) const;
    };
    
    // Timing utilities
    class Timer {
    private:
        std::chrono::high_resolution_clock::time_point start_time;
        
    public:
        Timer();
        void reset();
        double elapsed() const;  // Returns elapsed time in seconds
    };
    
    // Utility functions
    double manhattanDistance(const GridCoord& a, const GridCoord& b);
    double euclideanDistance(const GridCoord& a, const GridCoord& b);
    double euclideanDistance(const Vector2D& a, const Vector2D& b);
    
    // String formatting utilities
    std::string formatTime(double seconds);
    std::string formatPosition(const Vector2D& pos);
    std::string formatGridCoord(const GridCoord& coord);
    
    // Math utilities
    double clamp(double value, double min, double max);
    double lerp(double a, double b, double t);
    int sign(double value);
    
    // Grid utilities
    bool isValidGridCoord(const GridCoord& coord, int width, int height);
    std::vector<GridCoord> getNeighbors(const GridCoord& coord, int width, int height);
    std::vector<GridCoord> getNeighbors8(const GridCoord& coord, int width, int height); // 8-directional
}
