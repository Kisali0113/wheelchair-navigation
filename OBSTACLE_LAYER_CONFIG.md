# Ultrasonic Obstacle Layer Configuration

## Overview
The wheelchair navigation system uses three ultrasonic sensors (front, left, right) to detect obstacles and feed them into Nav2's costmap.

## Data Flow

```
Ultrasonic Sensors (Hardware)
    ↓
    └─→ /ultrasonic/front (Range messages)
    └─→ /ultrasonic/left  (Range messages)
    └─→ /ultrasonic/right (Range messages)
    
    ↓ [UltraSonicSensor.py - Maps to global coordinates]
    
    └─→ /obstacle/front (PointStamped in 'map' frame)
    └─→ /obstacle/left  (PointStamped in 'map' frame)
    └─→ /obstacle/right (PointStamped in 'map' frame)
    
    ↓ [Nav2 Obstacle Layer - Marking operation]
    
    └─→ Local Costmap (applies lethal cost)
    
    ↓ [Nav2 Inflation Layer]
    
    └─→ Safety Buffer (inflation_radius = 0.7m)
```

## Configuration Details (nav2_params.yaml)

### Ultrasonic Layer
- **Type**: ObstacleLayer (not RangeSensorLayer)
- **Input Topics**: 
  - `/obstacle/front` - Front sensor obstacle coordinates
  - `/obstacle/left` - Left sensor obstacle coordinates
  - `/obstacle/right` - Right sensor obstacle coordinates
- **Data Type**: PointCloud2 (converts PointStamped to point cloud internally)
- **Marking**: Enabled (places lethal cost at obstacle locations)
- **Clearing**: Disabled (ultrasonic obstacles are only marked, not cleared)

### Inflation Layer
- **cost_scaling_factor**: 3.0 (how steeply cost decreases with distance)
- **inflation_radius**: 0.7m (safety buffer around obstacles)
  - Creates a gradual cost field extending 0.7m around marked obstacles
  - Path planner steers away to avoid high-cost areas

## How It Works

1. **UltraSonicSensor.py** runs continuously:
   - Subscribes to raw `/ultrasonic/*` topics (Range messages)
   - Transforms each reading into global map coordinates
   - Publishes PointStamped obstacles to `/obstacle/*` topics

2. **Nav2 Obstacle Layer**:
   - Listens to `/obstacle/*` topics
   - Places "lethal cost" grid cells at exact obstacle coordinates
   - Updates local costmap in real-time

3. **Nav2 Inflation Layer**:
   - Expands the lethal cost to create a safety radius
   - 0.7m inflation ensures wheelchair (radius ~0.3m) clears obstacles
   - Path planner sees inflated costs and avoids the region

4. **Path Planning**:
   - MPPI controller uses the inflated costmap
   - ObstaclesCritic penalizes trajectories through high-cost areas
   - Wheelchair autonomously steers around detected obstacles

## Configuration Parameters

### Local Costmap (rolling window, real-time)
```yaml
ultrasonic_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  observation_sources: ultrasonic_front ultrasonic_left ultrasonic_right
  
  # Each sensor topic configuration
  ultrasonic_front:
    topic: /obstacle/front
    marking: true
    clearing: false
    data_type: "PointCloud2"
    obstacle_max_range: 2.0m
```

### Inflation Configuration
```yaml
inflation_layer:
  plugin: "nav2_costmap_2d::InflationLayer"
  cost_scaling_factor: 3.0   # Steepness of cost falloff
  inflation_radius: 0.7m     # Safety buffer distance
```

## Testing

1. **Verify topics are publishing**:
   ```bash
   ros2 topic list | grep obstacle
   ros2 topic echo /obstacle/front
   ```

2. **Check costmap visualization** (RViz):
   - Local Costmap layer should show inflated obstacles
   - Green areas = free, Red/Yellow = high cost, Dark = lethal

3. **Verify path planning avoids obstacles**:
   - Set navigation goal
   - Observe wheelchair path bends around inflated obstacles

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| Obstacles not appearing in costmap | UltraSonicSensor.py not running | `ros2 run wheelchair_mapping_pkg ultrasonic_mapper` |
| Wheelchair hits obstacles | Inflation radius too small | Increase `inflation_radius` in nav2_params.yaml |
| Path too conservative | Inflation radius too large | Decrease `inflation_radius` |
| Stale obstacle data | Sensor timeout | Check `/obstacle/*` topics for active publishing |

## Performance Tuning

- **Inflation Radius**: 0.7m balances safety vs. path efficiency
- **Cost Scaling**: 3.0 creates smooth cost gradients
- **Observation Sources**: Three sensors provide 270° coverage (front/left/right)
- **Update Frequency**: local_costmap updates at 5Hz

## Related Nodes

- **UltraSonicSensor.py**: Transforms raw Range → PointStamped
- **Nav2 Costmap**: Ingests obstacles and creates costmap
- **Path Planner**: Uses costmap to avoid obstacles
- **Controller**: Executes collision-free trajectories
