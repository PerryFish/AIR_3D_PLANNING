# Garage V1 Gazebo Camera And Proxy Usability

## Problem

Gazebo loaded the garage world, but the default camera could open at an unhelpful angle. The wall proxy also made the structure visible, but if the proxy looked too bright or thick it could dominate the scene and make zoom/pan inspection feel worse.

## Fixes

`src/virtual_env/garage_v1/worlds/garage_v1.world` now includes:

```xml
<gui fullscreen='0'>
  <camera name='garage_v1_overview'>
    <pose>0 -92 52 0 0.62 1.5708</pose>
    <view_controller>orbit</view_controller>
  </camera>
</gui>
```

The wall proxy remains static and visual-only:

- no `<collision>` blocks
- thinner walls
- darker grey material
- `<cast_shadows>false</cast_shadows>`

## Usability Result

The proxy should no longer act like a selectable collision shell, and the default camera starts from an overview of the garage footprint and UAV area.

The camera distance was widened because the real TARE/Gazebo garage mesh spans roughly `128 m x 143 m` after recentering. A close camera made zoom/pan feel awkward and hid the fact that the model was much larger than the earlier proxy.

## Remaining Gap

Gazebo Classic camera handling can still depend on the GUI state stored under the user's Gazebo home. The run scripts set `HOME` to a workspace-local `logs/gazebo_home` during tests, but manual GUI sessions may reuse user-level Gazebo state if launched outside the wrapper.
