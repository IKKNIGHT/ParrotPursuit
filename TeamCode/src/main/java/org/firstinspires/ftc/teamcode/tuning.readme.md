# Pure Pursuit Tuning Guide

## Table of Contents
1. [Quick Start](#quick-start)
2. [Understanding Pure Pursuit](#understanding-pure-pursuit)
3. [Key Parameters](#key-parameters)
4. [Step-by-Step Tuning](#step-by-step-tuning)
5. [Common Issues & Solutions](#common-issues--solutions)
6. [Advanced Tuning](#advanced-tuning)
7. [Performance Optimization](#performance-optimization)
8. [Troubleshooting](#troubleshooting)

## Quick Start

### 1. Initial Setup
```java
// 1. Set your localizer in DriveConstants.java
public static final Class LOCALIZER_CLASS = PinpointLocalizer.class;

// 2. Configure motor names in DriveConstants.MotorConfig
public static final String LEFT_FRONT = "frontleft";
// ... etc

// 3. Start with these conservative parameters:
public static double LOOKAHEAD_DISTANCE = 12.0;
public static double MAX_VELOCITY = 24.0;
public static double CURVATURE_VELOCITY_SCALING = 0.5;
```

### 2. First Test
Run `SimplePurePursuitTest` to verify basic functionality:
- Robot should move forward 36 inches smoothly
- No oscillation or jerky motion
- Should complete within 3-5 seconds

### 3. Basic Tuning
Use `PurePursuitTuner` with FTC Dashboard:
1. Connect to `192.168.43.1:8080/dash`
2. Adjust `LOOKAHEAD_DISTANCE` first
3. Then tune `MAX_VELOCITY`
4. Finally adjust `CURVATURE_VELOCITY_SCALING`

## Understanding Pure Pursuit

### Algorithm Overview
Pure Pursuit works by:
1. **Finding Closest Point**: Locate the point on the path closest to the robot
2. **Lookahead Point**: Find a point on the path exactly `LOOKAHEAD_DISTANCE` away
3. **Curvature Calculation**: Calculate steering curvature: `κ = 2sin(α)/L`
4. **Velocity Control**: Adjust speed based on path curvature
5. **Motor Commands**: Convert to motor powers

### Key Concepts

#### Lookahead Distance
- **Purpose**: How far ahead the robot "looks" along the path
- **Effect**: Controls smoothness vs responsiveness
- **Analogy**: Like looking ahead while driving a car

#### Curvature
- **Definition**: How sharply the robot needs to turn (1/radius)
- **Calculation**: Based on geometry to lookahead point
- **Usage**: Determines turning rate and velocity

#### Cross-Track Error
- **Definition**: Perpendicular distance from robot to path
- **Importance**: Primary accuracy metric
- **Target**: Keep below 3-4 inches for good performance

## Key Parameters

### Primary Parameters (Tune These First)

#### LOOKAHEAD_DISTANCE
- **Range**: 6-24 inches for FTC robots
- **Default**: 12 inches
- **Effect**: 
  - Too small: Oscillation, instability
  - Too large: Corner cutting, overshoot
- **Tuning**: Start with 12", adjust based on robot behavior

#### MAX_VELOCITY
- **Range**: 12-60 inches/second
- **Default**: 24 inches/second  
- **Effect**: Overall speed of path following
- **Tuning**: Set to ~80% of robot's maximum safe speed

#### CURVATURE_VELOCITY_SCALING
- **Range**: 0.0-1.0
- **Default**: 0.5
- **Effect**: How much to slow down for curves
- **Tuning**: 0.3 for gentle slowdown, 0.7 for aggressive

### Secondary Parameters (Fine-Tuning)

#### PATH_COMPLETION_TOLERANCE
- **Range**: 1-6 inches
- **Default**: 2 inches
- **Effect**: When to consider path complete
- **Tuning**: Smaller for precision, larger for speed

#### MIN_VELOCITY
- **Range**: 3-12 inches/second
- **Default**: 6 inches/second
- **Effect**: Prevents robot from stopping completely
- **Tuning**: Set to minimum speed that prevents stalling

### Advanced Parameters

#### ADAPTIVE_LOOKAHEAD_GAIN
- **Range**: 0.0-1.0
- **Default**: 0.3
- **Effect**: How much lookahead adapts to velocity
- **Usage**: Advanced teams only

#### MAX_CURVATURE
- **Range**: 0.05-0.5
- **Default**: 0.2
- **Effect**: Limits maximum turning rate
- **Usage**: Prevents impossible turns

## Step-by-Step Tuning

### Phase 1: Basic Functionality (15 minutes)

1. **Verify Hardware**
   ```java
   // Run SimplePurePursuitTest
   // Robot should move forward smoothly
   // Check for any hardware issues
   ```

2. **Test Localizer**
   ```java
   // Verify position tracking is accurate
   // Check that robot position updates correctly
   // Ensure no NaN values in telemetry
   ```

3. **Basic Motion**
   ```java
   // Should complete 36" straight line in 2-4 seconds
   // No oscillation or jerky motion
   // Path progress should reach 100%
   ```

### Phase 2: Lookahead Tuning (20 minutes)

1. **Start Conservative**
   ```java
   LOOKAHEAD_DISTANCE = 15.0; // Larger is safer
   ```

2. **Test Straight Line**
   - Should be smooth and direct
   - Reduce if robot is sluggish
   - Increase if robot oscillates

3. **Test Curves**
   ```java
   // Use PurePursuitExample S-curve test
   // Adjust until smooth following without corner cutting
   ```

4. **Find Sweet Spot**
   - Typical range: 8-16 inches for most FTC robots
   - Smaller robots: 6-10 inches
   - Larger robots: 12-18 inches

### Phase 3: Velocity Tuning (15 minutes)

1. **Set Maximum Velocity**
   ```java
   // Start at 50% of robot's max speed
   MAX_VELOCITY = robot_max_speed * 0.5;
   ```

2. **Test Performance**
   - Increase gradually until performance degrades
   - Watch for wheel slipping or instability
   - Typical FTC range: 20-40 inches/second

3. **Tune Curve Slowdown**
   ```java
   // Test with S-curve or figure-8
   CURVATURE_VELOCITY_SCALING = 0.5; // Start here
   // Increase for more aggressive slowdown
   // Decrease for gentler slowdown
   ```

### Phase 4: Fine-Tuning (15 minutes)

1. **Completion Tolerance**
   ```java
   // Test with various path endpoints
   PATH_COMPLETION_TOLERANCE = 2.0; // Good default
   // Reduce for precision applications
   // Increase for speed applications
   ```

2. **Minimum Velocity**
   ```java
   // Ensure robot doesn't stall on tight curves
   MIN_VELOCITY = 6.0; // Start here
   // Increase if robot stops during curves
   ```

3. **Validation**
   ```java
   // Run PathShapeTests to verify all scenarios
   // All tests should pass with <3" max error
   ```

## Common Issues & Solutions

### Robot Oscillates Back and Forth

**Cause**: Lookahead distance too small
**Solution**: 
```java
LOOKAHEAD_DISTANCE = 15.0; // Increase from current value
```

### Robot Cuts Corners on Curves

**Cause**: Lookahead distance too large
**Solution**:
```java
LOOKAHEAD_DISTANCE = 8.0; // Decrease from current value
```

### Robot Moves Too Slowly

**Cause**: Max velocity too low or excessive curve slowdown
**Solution**:
```java
MAX_VELOCITY = 36.0; // Increase
CURVATURE_VELOCITY_SCALING = 0.3; // Reduce slowdown
```

### Robot Overshoots Path End

**Cause**: Completion tolerance too large
**Solution**:
```java
PATH_COMPLETION_TOLERANCE = 1.0; // Reduce tolerance
```

### Robot Stops Before Path End

**Cause**: Completion tolerance too small or path issues
**Solution**:
```java
PATH_COMPLETION_TOLERANCE = 3.0; // Increase tolerance
// Or check path definition for issues
```

### Jerky or Unstable Motion

**Causes & Solutions**:
1. **Hardware Issues**
   ```java
   // Check motor directions
   // Verify encoder connections
   // Ensure adequate power supply
   ```

2. **Tuning Issues**
   ```java
   // Reduce MAX_VELOCITY
   // Increase VELOCITY_SMOOTHING
   // Check control loop frequency
   ```

3. **Path Issues**
   ```java
   // Avoid sharp discontinuities in paths
   // Use appropriate path types (Linear vs Cubic)
   // Ensure paths are reasonable length
   ```

## Advanced Tuning

### Adaptive Lookahead

For experienced teams wanting velocity-dependent lookahead:

```java
// Enable adaptive lookahead
public void updateAdaptiveLookahead() {
    double velocityFactor = currentVelocity / MAX_VELOCITY;
    double adaptiveLookahead = MIN_LOOKAHEAD_DISTANCE + 
        (MAX_LOOKAHEAD_DISTANCE - MIN_LOOKAHEAD_DISTANCE) * 
        velocityFactor * ADAPTIVE_LOOKAHEAD_GAIN;
    setLookaheadDistance(adaptiveLookahead);
}
```

### Path-Specific Tuning

Different paths may need different parameters:

```java
// For tight turns
drive.setLookaheadDistance(8.0);

// For high-speed straights  
drive.setLookaheadDistance(18.0);

// For precision work
drive.setLookaheadDistance(6.0);
```

### Velocity Profiling

Advanced velocity control based on path characteristics:

```java
private double calculateAdvancedVelocity(double curvature, double pathProgress) {
    // Slow down at path beginning and end
    double progressFactor = Math.min(pathProgress * 4, (1 - pathProgress) * 4);
    progressFactor = Math.min(1.0, progressFactor);
    
    // Slow down for high curvature
    double curvatureFactor = 1.0 / (1.0 + Math.abs(curvature) * CURVATURE_VELOCITY_SCALING);
    
    return MAX_VELOCITY * progressFactor * curvatureFactor;
}
```

## Performance Optimization

### Target Benchmarks

- **Update Rate**: >30Hz consistently
- **Cross-Track Error**: <3 inches for straight paths, <4 inches for curves
- **Completion Rate**: >95% for well-designed paths
- **Execution Time**: Within 20% of theoretical minimum

### Optimization Tips

1. **Reduce Path Resolution**
   ```java
   PATH_RESOLUTION = 0.02; // Increase from 0.01 if performance issues
   ```

2. **Optimize Control Loop**
   ```java
   // Run at consistent 50Hz
   sleep(20); // Exactly 20ms between updates
   ```

3. **Efficient Telemetry**
   ```java
   // Throttle telemetry updates
   if (updateCount % 10 == 0) {
       telemetry.update();
   }
   ```

## Troubleshooting

### Diagnostic Checklist

1. **Hardware Verification**
   - [ ] All motors respond correctly
   - [ ] Localizer provides valid position data
   - [ ] No NaN values in telemetry
   - [ ] Adequate power supply

2. **Software Verification**
   - [ ] Correct localizer class configured
   - [ ] Motor names match configuration
   - [ ] Path definitions are valid
   - [ ] Update loop running at correct frequency

3. **Tuning Verification**
   - [ ] Lookahead distance appropriate for robot size
   - [ ] Max velocity within robot capabilities
   - [ ] Completion tolerance reasonable
   - [ ] No extreme parameter values

### Performance Analysis

Use the built-in performance reporting:

```java
String report = drive.getPerformanceReport();
telemetry.addLine(report);

String validation = drive.validateTuningParameters();
telemetry.addLine(validation);
```

### Common Error Messages

- **"THE INPUT IS NAN"**: Localizer issue, check hardware connections
- **"Could not find closest point on path"**: Path definition issue
- **"Could not find lookahead point"**: Path too short or lookahead too large
- **"Path timeout exceeded"**: Increase timeout or check for robot stuck

### Getting Help

1. **Use Validation Tools**
   ```java
   // Run PurePursuitTuner for real-time parameter testing
   // Run PathShapeTests for comprehensive validation
   // Run PurePursuitStressTests for edge case testing
   ```

2. **Check Performance Metrics**
   - Cross-track error should be <4 inches
   - Update frequency should be >30Hz
   - Path completion should be >95%

3. **Community Resources**
   - FTC Discord channels
   - Team forums and wikis
   - Competition mentor guidance

## Conclusion

Pure pursuit is a powerful algorithm that, when properly tuned, provides smooth and reliable autonomous path following. Start with conservative parameters, tune methodically, and validate thoroughly. The investment in proper tuning will pay off with consistent autonomous performance in competition.

Remember: **Smooth and consistent is better than fast and unreliable.**