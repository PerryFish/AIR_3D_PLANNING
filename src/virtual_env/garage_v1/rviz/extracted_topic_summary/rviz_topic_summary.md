# TARE V1 RViz Topic Summary

Generated from copied TARE_V1 RViz files under `tare_v1_original_rviz/`.

## `tare_v1_original_rviz/install/tare_planner/share/tare_planner/tare_planner_ground.rviz`

- Fixed Frame: `map`
- Display count parsed: `20`

| Display | Type | Enabled | Topic | Color | Size | Alpha | Queue | Decay | Style | Line Width | Likely meaning |
|---|---|---|---|---|---|---|---|---|---|---|---|
| KeyposeGraphNodes | rviz_default_plugins/PointCloud2 | false | /keypose_graph_cloud | 255; 255; 255 | 0.30000001192092896 |  |  | 0 | Boxes |  | keypose graph / branch history |
| KeyposeGraphEdge | rviz_default_plugins/Marker | false | /keypose_graph_edge_marker |  |  |  |  |  |  |  | keypose graph / branch history |
| ExploringSubspaces | rviz_default_plugins/Marker | true | /tare_visualizer/exploring_subspaces |  |  |  |  |  |  |  | marker / planning overlay |
| GlobalPath | rviz_default_plugins/Path | true | /global_path | 98; 240; 231 |  |  |  |  |  | 0.20000000298023224 | global path |
| OverallObjectSurfaces | rviz_default_plugins/PointCloud2 | false | /planner_cloud | 255; 255; 255 | 0.10000000149011612 |  |  | 0 | Flat Squares |  | planner internal/local collision cloud |
| OverallMap | rviz_default_plugins/PointCloud2 | true | /overall_map | 255; 255; 255 | 0.009999999776482582 |  |  | 0 | Points |  | overall accumulated map |
| ExploredAreas | rviz_default_plugins/PointCloud2 | true | /explored_areas | 0; 170; 255 | 0.009999999776482582 |  |  | 0 | Points |  | point cloud display |
| Global | rviz_default_plugins/Polygon | true | /navigation_boundary | 25; 255; 0 |  |  |  |  |  |  |  |
| LocalPlanningHorizon | rviz_default_plugins/Marker | true | /tare_visualizer/local_planning_horizon |  |  |  |  |  |  |  | marker / planning overlay |
| ViewpointCandidates | rviz_default_plugins/PointCloud2 | false | /viewpoint_vis_cloud | 237; 212; 0 | 0.5 |  |  | 0 | Spheres |  | candidate or selected viewpoints |
| SelectedViewPoints | rviz_default_plugins/PointCloud2 | true | /selected_viewpoint_vis_cloud | 245; 121; 0 | 1 |  |  | 0 | Spheres |  | candidate or selected viewpoints |
| ViewpointToTrack | rviz_default_plugins/PointCloud2 | false | /lookahead_point_cloud | 255; 255; 255 | 2 |  |  | 0 | Spheres |  | candidate or selected viewpoints |
| LocalPath | rviz_default_plugins/Path | true | /local_path | 45; 66; 253 |  |  |  |  |  | 0.20000000298023224 | local TSP/planning path |
| ObjectSurfacesToCover | rviz_default_plugins/PointCloud2 | true | /uncovered_cloud | 204; 0; 0 | 0.20000000298023224 |  |  | 0 | Points |  | point cloud display |
| FrontierSurfacesToCover | rviz_default_plugins/PointCloud2 | true | /uncovered_frontier_cloud | 204; 0; 0 | 0.30000001192092896 |  |  | 0 | Points |  | point cloud display |
| Waypoint | rviz_default_plugins/PointStamped | true | /way_point | 204; 41; 204 |  |  |  |  |  |  | current waypoint |
| Vehicle | rviz_default_plugins/Axes | true |  |  |  |  |  |  |  |  | vehicle simulator marker/path |
| FreePaths | rviz_default_plugins/PointCloud2 | false | /free_paths | 255; 255; 255 | 0.009999999776482582 |  |  | 0 | Flat Squares |  | point cloud display |
| root | rviz_default_plugins/Path | true | /path | 25; 255; 0 |  |  |  |  |  | 0.10000000149011612 | path / trajectory display |
|  | rviz_default_plugins/TF |  |  |  |  |  |  |  |  |  |  |

## `tare_v1_original_rviz/install/vehicle_simulator/share/vehicle_simulator/rviz/vehicle_simulator.rviz`

- Fixed Frame: `map`
- Display count parsed: `15`

| Display | Type | Enabled | Topic | Color | Size | Alpha | Queue | Decay | Style | Line Width | Likely meaning |
|---|---|---|---|---|---|---|---|---|---|---|---|
| Vehicle | rviz_default_plugins/Axes | true |  |  |  |  |  |  |  |  | vehicle simulator marker/path |
| Image | rviz_default_plugins/Image | true | /camera/image_raw |  |  |  |  |  |  |  |  |
| RegScan | rviz_default_plugins/PointCloud2 | true | /registered_scan | 255; 255; 255 | 0.05000000074505806 |  |  | 10 | Points |  | current registered scan / live sensor structure |
| SensorScan | rviz_default_plugins/PointCloud2 | false | /sensor_scan | 255; 255; 255 | 0.10000000149011612 |  |  | 10 | Points |  | point cloud display |
| TerrainMap | rviz_default_plugins/PointCloud2 | false | /terrain_map | 255; 255; 255 | 0.03999999910593033 |  |  | 0 | Points |  | terrain structure map |
| TerrainMapExt | rviz_default_plugins/PointCloud2 | false | /terrain_map_ext | 255; 255; 255 | 0.03999999910593033 |  |  | 0 | Points |  | extended terrain / collision map |
| Path | rviz_default_plugins/Path | true | /path | 25; 255; 0 |  |  |  |  |  | 0.05000000074505806 | path / trajectory display |
| FreePaths | rviz_default_plugins/PointCloud2 | true | /free_paths | 0; 170; 255 | 0.019999999552965164 |  |  | 0 | Points |  | point cloud display |
| Waypoint | rviz_default_plugins/PointStamped | true | /way_point | 204; 41; 204 |  |  |  |  |  |  | current waypoint |
| Boundary | rviz_default_plugins/Polygon | true | /navigation_boundary | 0; 255; 0 |  |  |  |  |  |  |  |
| AddedObstacles | rviz_default_plugins/PointCloud2 | true | /added_obstacles | 255; 25; 0 | 0.05000000074505806 |  |  | 0 | Points |  | point cloud display |
| OverallMap | rviz_default_plugins/PointCloud2 | false | /overall_map | 255; 255; 255 | 0.009999999776482582 |  |  | 0 | Points |  | overall accumulated map |
| ExploredAreas | rviz_default_plugins/PointCloud2 | false | /explored_areas | 0; 170; 255 | 0.009999999776482582 |  |  | 0 | Points |  | point cloud display |
| root | rviz_default_plugins/PointCloud2 | true | /trajectory | 255; 255; 255 | 0.009999999776482582 |  |  | 0 | Points |  | point cloud display |
|  | rviz_default_plugins/TF |  |  |  |  |  |  |  |  |  |  |

## `tare_v1_original_rviz/install/velodyne_description/share/velodyne_description/rviz/example.rviz`

- Fixed Frame: `lidar`
- Display count parsed: `5`

| Display | Type | Enabled | Topic | Color | Size | Alpha | Queue | Decay | Style | Line Width | Likely meaning |
|---|---|---|---|---|---|---|---|---|---|---|---|
| Grid | rviz_default_plugins/Grid | true |  | 160; 160; 164 |  |  |  |  |  | 0.029999999329447746 |  |
| TF | rviz_default_plugins/TF | true |  |  |  |  |  |  |  |  |  |
| VLP-16 | rviz_default_plugins/PointCloud2 | true | /velodyne_points | 255; 255; 255 | 0.009999999776482582 |  |  | 0 | Points |  | point cloud display |
| root | rviz_default_plugins/PointCloud2 | true | /velodyne_points2 | 255; 255; 255 | 0.009999999776482582 |  |  | 0 | Points |  | point cloud display |
|  | rviz_default_plugins/TF |  |  |  |  |  |  |  |  |  |  |

## `tare_v1_original_rviz/src/autonomous_exploration_development_environment/src/vehicle_simulator/rviz/vehicle_simulator.rviz`

- Fixed Frame: `map`
- Display count parsed: `15`

| Display | Type | Enabled | Topic | Color | Size | Alpha | Queue | Decay | Style | Line Width | Likely meaning |
|---|---|---|---|---|---|---|---|---|---|---|---|
| Vehicle | rviz_default_plugins/Axes | true |  |  |  |  |  |  |  |  | vehicle simulator marker/path |
| Image | rviz_default_plugins/Image | true | /camera/image_raw |  |  |  |  |  |  |  |  |
| RegScan | rviz_default_plugins/PointCloud2 | true | /registered_scan | 255; 255; 255 | 0.05000000074505806 |  |  | 10 | Points |  | current registered scan / live sensor structure |
| SensorScan | rviz_default_plugins/PointCloud2 | false | /sensor_scan | 255; 255; 255 | 0.10000000149011612 |  |  | 10 | Points |  | point cloud display |
| TerrainMap | rviz_default_plugins/PointCloud2 | false | /terrain_map | 255; 255; 255 | 0.03999999910593033 |  |  | 0 | Points |  | terrain structure map |
| TerrainMapExt | rviz_default_plugins/PointCloud2 | false | /terrain_map_ext | 255; 255; 255 | 0.03999999910593033 |  |  | 0 | Points |  | extended terrain / collision map |
| Path | rviz_default_plugins/Path | true | /path | 25; 255; 0 |  |  |  |  |  | 0.05000000074505806 | path / trajectory display |
| FreePaths | rviz_default_plugins/PointCloud2 | true | /free_paths | 0; 170; 255 | 0.019999999552965164 |  |  | 0 | Points |  | point cloud display |
| Waypoint | rviz_default_plugins/PointStamped | true | /way_point | 204; 41; 204 |  |  |  |  |  |  | current waypoint |
| Boundary | rviz_default_plugins/Polygon | true | /navigation_boundary | 0; 255; 0 |  |  |  |  |  |  |  |
| AddedObstacles | rviz_default_plugins/PointCloud2 | true | /added_obstacles | 255; 25; 0 | 0.05000000074505806 |  |  | 0 | Points |  | point cloud display |
| OverallMap | rviz_default_plugins/PointCloud2 | false | /overall_map | 255; 255; 255 | 0.009999999776482582 |  |  | 0 | Points |  | overall accumulated map |
| ExploredAreas | rviz_default_plugins/PointCloud2 | false | /explored_areas | 0; 170; 255 | 0.009999999776482582 |  |  | 0 | Points |  | point cloud display |
| root | rviz_default_plugins/PointCloud2 | true | /trajectory | 255; 255; 255 | 0.009999999776482582 |  |  | 0 | Points |  | point cloud display |
|  | rviz_default_plugins/TF |  |  |  |  |  |  |  |  |  |  |

## `tare_v1_original_rviz/src/autonomous_exploration_development_environment/src/velodyne_simulator/velodyne_description/rviz/example.rviz`

- Fixed Frame: `lidar`
- Display count parsed: `5`

| Display | Type | Enabled | Topic | Color | Size | Alpha | Queue | Decay | Style | Line Width | Likely meaning |
|---|---|---|---|---|---|---|---|---|---|---|---|
| Grid | rviz_default_plugins/Grid | true |  | 160; 160; 164 |  |  |  |  |  | 0.029999999329447746 |  |
| TF | rviz_default_plugins/TF | true |  |  |  |  |  |  |  |  |  |
| VLP-16 | rviz_default_plugins/PointCloud2 | true | /velodyne_points | 255; 255; 255 | 0.009999999776482582 |  |  | 0 | Points |  | point cloud display |
| root | rviz_default_plugins/PointCloud2 | true | /velodyne_points2 | 255; 255; 255 | 0.009999999776482582 |  |  | 0 | Points |  | point cloud display |
|  | rviz_default_plugins/TF |  |  |  |  |  |  |  |  |  |  |

## `tare_v1_original_rviz/src/tare_planner/src/tare_planner/rviz/tare_planner_ground.rviz`

- Fixed Frame: `map`
- Display count parsed: `20`

| Display | Type | Enabled | Topic | Color | Size | Alpha | Queue | Decay | Style | Line Width | Likely meaning |
|---|---|---|---|---|---|---|---|---|---|---|---|
| KeyposeGraphNodes | rviz_default_plugins/PointCloud2 | false | /keypose_graph_cloud | 255; 255; 255 | 0.30000001192092896 |  |  | 0 | Boxes |  | keypose graph / branch history |
| KeyposeGraphEdge | rviz_default_plugins/Marker | false | /keypose_graph_edge_marker |  |  |  |  |  |  |  | keypose graph / branch history |
| ExploringSubspaces | rviz_default_plugins/Marker | true | /tare_visualizer/exploring_subspaces |  |  |  |  |  |  |  | marker / planning overlay |
| GlobalPath | rviz_default_plugins/Path | true | /global_path | 98; 240; 231 |  |  |  |  |  | 0.20000000298023224 | global path |
| OverallObjectSurfaces | rviz_default_plugins/PointCloud2 | false | /planner_cloud | 255; 255; 255 | 0.10000000149011612 |  |  | 0 | Flat Squares |  | planner internal/local collision cloud |
| OverallMap | rviz_default_plugins/PointCloud2 | true | /overall_map | 255; 255; 255 | 0.009999999776482582 |  |  | 0 | Points |  | overall accumulated map |
| ExploredAreas | rviz_default_plugins/PointCloud2 | true | /explored_areas | 0; 170; 255 | 0.009999999776482582 |  |  | 0 | Points |  | point cloud display |
| Global | rviz_default_plugins/Polygon | true | /navigation_boundary | 25; 255; 0 |  |  |  |  |  |  |  |
| LocalPlanningHorizon | rviz_default_plugins/Marker | true | /tare_visualizer/local_planning_horizon |  |  |  |  |  |  |  | marker / planning overlay |
| ViewpointCandidates | rviz_default_plugins/PointCloud2 | false | /viewpoint_vis_cloud | 237; 212; 0 | 0.5 |  |  | 0 | Spheres |  | candidate or selected viewpoints |
| SelectedViewPoints | rviz_default_plugins/PointCloud2 | true | /selected_viewpoint_vis_cloud | 245; 121; 0 | 1 |  |  | 0 | Spheres |  | candidate or selected viewpoints |
| ViewpointToTrack | rviz_default_plugins/PointCloud2 | false | /lookahead_point_cloud | 255; 255; 255 | 2 |  |  | 0 | Spheres |  | candidate or selected viewpoints |
| LocalPath | rviz_default_plugins/Path | true | /local_path | 45; 66; 253 |  |  |  |  |  | 0.20000000298023224 | local TSP/planning path |
| ObjectSurfacesToCover | rviz_default_plugins/PointCloud2 | true | /uncovered_cloud | 204; 0; 0 | 0.20000000298023224 |  |  | 0 | Points |  | point cloud display |
| FrontierSurfacesToCover | rviz_default_plugins/PointCloud2 | true | /uncovered_frontier_cloud | 204; 0; 0 | 0.30000001192092896 |  |  | 0 | Points |  | point cloud display |
| Waypoint | rviz_default_plugins/PointStamped | true | /way_point | 204; 41; 204 |  |  |  |  |  |  | current waypoint |
| Vehicle | rviz_default_plugins/Axes | true |  |  |  |  |  |  |  |  | vehicle simulator marker/path |
| FreePaths | rviz_default_plugins/PointCloud2 | false | /free_paths | 255; 255; 255 | 0.009999999776482582 |  |  | 0 | Flat Squares |  | point cloud display |
| root | rviz_default_plugins/Path | true | /path | 25; 255; 0 |  |  |  |  |  | 0.10000000149011612 | path / trajectory display |
|  | rviz_default_plugins/TF |  |  |  |  |  |  |  |  |  |  |

## Unique Topics

- `/added_obstacles`
- `/camera/image_raw`
- `/explored_areas`
- `/free_paths`
- `/global_path`
- `/keypose_graph_cloud`
- `/keypose_graph_edge_marker`
- `/local_path`
- `/lookahead_point_cloud`
- `/navigation_boundary`
- `/overall_map`
- `/path`
- `/planner_cloud`
- `/registered_scan`
- `/selected_viewpoint_vis_cloud`
- `/sensor_scan`
- `/tare_visualizer/exploring_subspaces`
- `/tare_visualizer/local_planning_horizon`
- `/terrain_map`
- `/terrain_map_ext`
- `/trajectory`
- `/uncovered_cloud`
- `/uncovered_frontier_cloud`
- `/velodyne_points`
- `/velodyne_points2`
- `/viewpoint_vis_cloud`
- `/way_point`
