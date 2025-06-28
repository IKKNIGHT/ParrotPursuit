## Parrot Pursuit - FTC Pure Pursuit Implementation

A complete Pure Pursuit path following system for FTC robots using Java.

### Features
- **True Pure Pursuit Algorithm** - Not just PID to waypoints, but actual lookahead point following
- **Multiple Path Types** - LinearPath, CubicPath, PolyLine, and PathChain support  
- **Adaptive Control** - Velocity adjustment based on path curvature
- **Multiple Localizers** - Support for Pinpoint, SparkFun OTOS, and Three Dead Wheel
- **Real-time Tuning** - FTC Dashboard integration for parameter adjustment
- **Comprehensive Examples** - Ready-to-use OpModes for testing and learning

### Quick Start

1. **Basic Usage:**
```java
// Initialize drive system
MecanumDrive drive = new MecanumDrive(hardwareMap, telemetry, startPose);

// Create a path
LinearPath path = new LinearPath(
    new Point(0, 0),    // Start point
    new Point(48, 24)   // End point
);

// Follow the path
drive.followPath(path);

// Main loop
while (opModeIsActive() && drive.isFollowingPath()) {
    drive.updateLocalizer();
    drive.updatePurePursuit();
    sleep(20); // 50Hz update rate
}
```

2. **Configuration:**
   - Set your localizer type in `DriveConstants.LOCALIZER_CLASS`
   - Configure motor names in `DriveConstants.MotorConfig`
   - Tune parameters in `DriveConstants.TunableParams`

3. **Testing:**
   - Run `SimplePurePursuitTest` for basic functionality
   - Use `PurePursuitTuner` with FTC Dashboard for parameter optimization
   - Try `PurePursuitExample` for advanced scenarios

### Path Types

- **LinearPath** - Straight line between two points
- **CubicPath** - Smooth curved path using 4 control points  
- **PolyLine** - Multi-segment path through waypoints
- **PathChain** - Combine multiple paths into one sequence

### Key Parameters

- `LOOKAHEAD_DISTANCE` - How far ahead to look on the path (6-24 inches)
- `MAX_VELOCITY` - Maximum robot speed (inches/second)
- `CURVATURE_VELOCITY_SCALING` - How much to slow down for curves
- `PATH_COMPLETION_TOLERANCE` - How close to path end to stop

### Tuning Guide

1. Start with `SimplePurePursuitTest` to verify basic functionality
2. Use `PurePursuitTuner` with FTC Dashboard for real-time parameter adjustment
3. Adjust `LOOKAHEAD_DISTANCE`: 
   - Too small = oscillation and instability
   - Too large = cutting corners and overshoot
4. Tune `MAX_VELOCITY` based on your robot's capabilities
5. Adjust `CURVATURE_VELOCITY_SCALING` for smooth cornering

### Advanced Features

- **Adaptive Lookahead** - Automatically adjusts based on robot speed
- **Path Visualization** - Real-time path display on FTC Dashboard
- **Progress Tracking** - Monitor path completion percentage  
- **Smooth Velocity Control** - Automatic speed adjustment for curves

### API Reference

**Main Methods:**
- `followPath(Path path)` - Start following a path
- `updatePurePursuit()` - Update control (call in main loop)
- `stopPathFollowing()` - Stop path following
- `isFollowingPath()` - Check if currently following a path
- `getPathProgress()` - Get completion percentage (0.0 to 1.0)

**Legacy Support:**
- Original waypoint methods still available for backwards compatibility
- `setTarget(WayPoint)` and `updatePIDS()` work alongside pure pursuit

### Requirements

- FTC SDK 8.0+
- FTC Dashboard (for tuning)
- Supported localizer (Pinpoint recommended)
- Mecanum drive system

### Support

This implementation is designed for FTC teams wanting professional-grade path following. The pure pursuit algorithm provides smooth, predictable robot movement that's essential for competitive autonomous routines.