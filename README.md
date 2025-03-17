# SWARM Algorithm Implementation

A multi-agent swarm control system implementing formation control and path planning using artificial potential fields (APF).

## Features

- V-formation and sequential formation control
- Obstacle avoidance using artificial potential fields
- Dynamic formation switching for tunnel navigation
- MAVLink communication support for autopilot integration
- Kinematic model with velocity and orientation control

## Implementations

The algorithm is implemented in multiple languages:

- MATLAB (.m files) - For simulation and visualization
- C++ - For real-time control implementation
- C - For MAVLink communication with autopilot

## Key Components

- `formation.m` - Main formation control algorithm
- `swarm.cpp` - C++ implementation of the swarm control
- `lyponuv.m` - Lyapunov stability analysis
- `readautopilot.c`/`send_autopilot.c` - MAVLink communication interface
- `udp_connection.c` - Network communication handling

## Usage

1. Run MATLAB simulations for algorithm verification
2. Compile and run C++ implementation for real-time control
3. Use C components for autopilot integration

## Requirements

- MATLAB
- C++ compiler
- MAVLink library
- UDP network support