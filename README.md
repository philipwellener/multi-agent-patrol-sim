# Multi-Agent Patrol Sim

A little C++ project I put together to explore autonomous agents with behavior trees and pathfinding.

![Demo](assets/demo.gif)

*Note: The GIF compression makes the visualization appear choppy - the actual simulation runs much smoother when viewed live or in video format.*

## What it does

- Multiple agents patrol around independently (currently 4 of them)
- Each agent has its own "brain" using behavior trees 
- A* pathfinding so they don't walk into walls
- They try to avoid bumping into each other (mostly works)
- Real-time ASCII visualization because why not
- Tweakable settings if you want to mess around with parameters

## Project structure

```
multi-agent-patrol-sim/
├── src/
│   ├── main.cpp               # main simulation loop
│   ├── config.hpp             # all the tunable parameters
│   ├── grid.hpp/cpp           # grid world management 
│   ├── agent.hpp/cpp          # agent logic + behavior trees
│   ├── behavior_tree.hpp/cpp  # the actual behavior tree stuff
│   ├── pathfinding.hpp/cpp    # A* implementation
│   └── utils.hpp/cpp          # random helper functions
├── assets/                    # was gonna add graphics but... maybe later
├── CMakeLists.txt            # build config
└── README.md                 # you are here
```

## Building and Running

```bash
# standard cmake dance
mkdir build && cd build
cmake ..
make

# run it
./MultiAgentPatrolSim
```

If you want debug info or optimizations:
```bash
cmake -DCMAKE_BUILD_TYPE=Debug ..     # for debugging
cmake -DCMAKE_BUILD_TYPE=Release ..   # for speed
cmake -DENABLE_ASAN=ON ..             # if things are crashing weirdly
```

## How to use it

Just run it and watch the agents do their thing. They show up as A0, A1, A2 etc, obstacles are those █ blocks. 

The agents will wander around to their patrol points and try not to crash into stuff. Sometimes they get a bit confused but that's half the fun.

Edit `src/config.hpp` - you can change grid size, number of agents, how fast they move, etc.

## The technical bits

### Behavior Trees
Ended up going with behavior trees due to flexiblility.
- Composite nodes (sequence, selector) 
- Some decorators for retry logic
- Leaf nodes for actual actions
- Easy enough to add new behaviors without breaking everything

### Pathfinding  
A* seemed like the obvious choice - works well enough for this grid setup. Does dynamic replanning when agents get in each other's way, though sometimes they still do a little dance around each other.

### Multi-agent stuff
No central coordinator - each agent just does its own thing and tries to be polite to the others. Collision detection is... adequate. Could probably be smarter about it but hey, it works.

### Visualization & Display
The terminal output turned out way better than expected. You get real-time updates showing:
- Agent positions and current states (● for patrolling, ◆ for avoiding obstacles, ▲ for avoiding other agents)
- Live pathfinding with visual indicators for waypoints and planned routes
- Color-coded agents and obstacles for easier tracking
- Performance metrics like FPS and step counts
- Clean status display showing what each agent is currently doing

Also added support for both terminal and GUI modes. The SFML graphics mode gives you a proper window with smooth movement and better visual feedback, though the ASCII version has its own charm.

### Recent Updates
Been tweaking the simulation based on what I've observed:
- Improved stuck detection - agents now recognize when they're not making progress and try alternative routes
- Better coordination between agents to reduce those awkward "after you, no after you" moments
- Added configurable simulation speed so you can watch the action in slow motion or speed it up
- More detailed statistics tracking for analyzing agent behavior patterns
- Grid generation got smarter about creating interesting but navigable obstacle layouts

The configuration system is pretty flexible now - you can adjust everything from grid size and agent count to movement speed and vision range without recompiling.

## Lessons learned

### What went well:
- Behavior trees are actually pretty nice to work with once you get the hang of them
- A* is solid - just works out of the box most of the time  
- Terminal visualization is surprisingly satisfying to watch
- Modular design paid off when debugging (could test pathfinding separately)

### What was trickier than expected:
- Agent collision avoidance. First attempt was basically "stop and wait" which led to deadlocks
- Getting the timing right - too fast and you can't see what's happening, too slow and it's boring
- Debugging multi-agent interactions is a pain - adding print statements everywhere made it worse
- C++ templates for the behavior tree stuff... took a few tries to get right

### If I did this again:
- Would probably start with fewer agents and work up 
- Maybe use a proper logging system instead of count spam
- The obstacle avoidance could be way smarter - right now it's pretty naive
- Should have written tests earlier
- Grid visualization could show agent paths/intentions somehow

### Weird issues I ran into:
- Agents would sometimes get "stuck" not because of obstacles, but because of floating point precision
- Had a bug where agents would prefer moving diagonally even when straight lines were better
- Memory management with the behavior trees was messier than expected

The biggest thing was realizing that "simple" multi-agent systems get complex fast. Even with just a few agents, you start seeing emergent behaviors you didn't expect. Sometimes good (they form little convoys), sometimes not so good (traffic jams at narrow passages).


## TODO

- Machine learning so agents get better over time
- Network support for distributed sims
- Better performance metrics
- 3D? (probably overkill but could be cool)