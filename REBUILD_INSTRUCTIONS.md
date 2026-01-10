# Rebuild and Test Instructions

## Quick Rebuild

```bash
# Stop existing containers
./watod down robot

# Build all packages (including new safety_filter)
./watod build robot

# Start everything
./watod up robot
```

## What Was Changed

### New Package: safety_filter
- Location: `src/robot/safety_filter/`
- Purpose: Filters `/cmd_vel_planned` based on lidar scans to prevent collisions
- Publishes: `/cmd_vel` (safe velocity commands)

### Modified Packages
1. **costmap**: Added ROS parameter for `inflation_radius` (default 0.6m)
2. **map_memory**: Changed frame_id to `sim_world`
3. **planner**: Updated to use `sim_world` frame for paths
4. **control**: 
   - Now publishes to `/cmd_vel_planned` (instead of `/cmd_vel`)
   - Added `/executed_path` publisher to track robot trajectory

### Launch File
- Added safety_filter node to `robot.launch.py`

### Foxglove Config
- Added `/executed_path` visualization

## Testing

1. **Start system**: `./watod up`
2. **In Foxglove 3D panel**: Click anywhere to publish goal
3. **Verify**:
   - Robot navigates to goal
   - `/path` shows planned path (green)
   - `/executed_path` shows actual trajectory (pink)
   - Robot maintains safe clearance from walls
   - Safety filter prevents collisions

## Topic Flow

```
Control publishes → /cmd_vel_planned → Safety Filter → /cmd_vel → Robot
                                                          ↑
                                                    /lidar (checks)
```

## If Build Fails

Check that `package.xml` in `safety_filter` has `<name>` not `<n>`. If needed, manually edit:
```xml
<name>safety_filter</name>
```
