#!/bin/bash

# Multi-Agent Patrol Simulator Build Script
# This script builds the project using CMake

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if CMake is installed
if ! command -v cmake &> /dev/null; then
    print_error "CMake is not installed. Please install CMake first."
    exit 1
fi

# Check if a C++ compiler is available
if ! command -v g++ &> /dev/null && ! command -v clang++ &> /dev/null; then
    print_error "No C++ compiler found. Please install g++ or clang++."
    exit 1
fi

print_info "Building Multi-Agent Patrol Simulator..."

# Create build directory
if [ ! -d "build" ]; then
    print_info "Creating build directory..."
    mkdir build
fi

cd build

# Build type (default to Release)
BUILD_TYPE=${1:-Release}

if [ "$BUILD_TYPE" != "Debug" ] && [ "$BUILD_TYPE" != "Release" ]; then
    print_warning "Invalid build type '$BUILD_TYPE'. Using 'Release'."
    BUILD_TYPE="Release"
fi

print_info "Build type: $BUILD_TYPE"

# Configure with CMake
print_info "Configuring project with CMake..."
if cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE ..; then
    print_success "CMake configuration completed"
else
    print_error "CMake configuration failed"
    exit 1
fi

# Build the project
print_info "Building project..."
if make -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4); then
    print_success "Build completed successfully!"
else
    print_error "Build failed"
    exit 1
fi

# Check if executable exists
if [ -f "MultiAgentPatrolSim" ]; then
    print_success "Executable 'MultiAgentPatrolSim' created successfully"
    print_info "To run the simulation: ./build/MultiAgentPatrolSim"
else
    print_error "Executable not found"
    exit 1
fi

print_info "Build script completed!"
