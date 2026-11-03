# V2V Framework — ER Database Model

> **Status (decision D6, 2026-11-03):** no database is implemented and none is planned. This ER model is a
> **conceptual** description only. The **physical schema is the run directory** specified in
> `REFACTOR_HANDOFF.md` §5.6 (Phase 7, **not implemented yet**):
> `runs/<UTC-timestamp>_<short-sha>_<name>/` with `manifest.json`, `events.jsonl`, `v2x_links.jsonl`,
> `stations.jsonl`, `messages.jsonl`, `lidar_meta.jsonl` and optional `lidar/<sensor_id>/<frame>.bin`.
> Entity and field names below predate the V2V→V2X merge (`v2v_*`, BSM `latitude/longitude`); when Phase 7 lands,
> this diagram must be regenerated from the real fields. Do not treat the field lists as an implemented contract.


```mermaid
erDiagram
    SIMULATION_RUN {
        string run_id PK
        string carla_host
        int carla_port
        string map
        int duration_s
        int num_vehicles
        float v2v_range_m
        bool lidar_enabled
        int random_seed
        int total_frames
        float real_time_s
        float simulated_time_s
        int v2v_messages_sent
        float avg_neighbors
        int max_neighbors
    }

    SIMULATION_CONFIG {
        int config_id PK
        string run_id FK
        float fixed_delta_seconds
        float timeout
        int random_seed
        int tm_port
        bool use_hybrid_physics
        int v2v_update_interval_frames
        float v2v_max_range_m
        float v2v_update_interval_s
        int ego_vehicle_id
    }

    DSRC_CONFIG {
        int config_id PK
        string run_id FK
        float tx_power_dbm
        float sensitivity_dbm
        float path_loss_exp_los
        float path_loss_exp_nlos
        float shadowing_std_los_db
        float shadowing_std_nlos_db
        float channel_busy_ratio
        float prr_sigmoid_steepness
    }

    VEHICLE {
        int vehicle_id PK
        string run_id FK
        string blueprint_id
        string vehicle_type
        bool is_ego
    }

    BSM_CORE {
        int bsm_id PK
        int vehicle_id FK
        string run_id FK
        float timestamp
        int msg_count
        float latitude
        float longitude
        float elevation
        float position_accuracy
        float speed
        float heading
        float steering_angle
        float yaw_rate
        float longitudinal_accel
        float lateral_accel
        float vertical_accel
        float vehicle_length
        float vehicle_width
        float vehicle_height
        string brake_status
        float brake_pressure
        string transmission_state
        float speed_confidence
        float heading_confidence
        float position_confidence
    }

    BSM_PART2 {
        int part2_id PK
        int bsm_id FK
        string path_history
        string path_prediction
        bool is_emergency
        string exterior_lights
        bool cooperative_status
    }

    V2V_ENHANCED_MESSAGE {
        int message_id PK
        int bsm_id FK
        string run_id FK
        float transmission_time
        float tx_power_dbm
        float channel_busy_ratio
        string detected_objects
        int lidar_points_count
        string camera_detections
        int priority
    }

    CAM {
        int cam_id PK
        int bsm_id FK
        int station_id
        float generation_time
        float ref_lat
        float ref_lon
        float ref_alt
        float heading
        float speed
        string drive_direction
        string vehicle_role
        string path_history
    }

    DENM {
        int denm_id PK
        int station_id FK
        int action_id
        float detection_time
        float reference_time
        float event_lat
        float event_lon
        float event_alt
        int cause_code
        int subcause_code
        float information_quality
        float relevance_distance
        string relevance_traffic_direction
        float event_speed
        float event_heading
    }

    NEIGHBOR_RELATIONSHIP {
        int relationship_id PK
        string run_id FK
        int source_vehicle_id FK
        int target_vehicle_id FK
        float timestamp
        float distance_m
        int threat_level
        float ttc_s
    }

    POTHOLE {
        int pothole_id PK
        string run_id FK
        float x
        float y
        float z
        float radius
        int severity
        int road_id
        int lane_id
        float s
    }

    POTHOLE_EVENT {
        int event_id PK
        int pothole_id FK
        int vehicle_id FK
        string run_id FK
        int frame
        float sim_timestamp
        float wall_timestamp
        string event_type
        float distance_m
        float ego_speed_kmh
        float ego_x
        float ego_y
    }
    TRAFFIC_PARTICIPANT {
      int   
    }
    EGO_TELEMETRY_FRAME {
        int frame_id PK
        string run_id FK
        int frame
        float sim_timestamp
        float wall_timestamp
        float pos_x
        float pos_y
        float pos_z
        float vel_x
        float vel_y
        float vel_z
        float speed_kmh
        float yaw
        float pitch
        float roll
        float throttle
        float brake
        float steer
        int v2v_neighbors
        int threat_count
        float min_ttc_s
        float bsm_heading
        float bsm_accel
        int lidar_points
        int nearby_potholes
        int closest_pothole_id FK
        float closest_pothole_dist_m
        int closest_pothole_severity
    }

    SIMULATION_RUN ||--|| SIMULATION_CONFIG : "parameterized by"
    SIMULATION_RUN ||--|| DSRC_CONFIG : "uses channel"
    SIMULATION_RUN ||--o{ VEHICLE : "contains"
    SIMULATION_RUN ||--o{ POTHOLE : "has"
    SIMULATION_RUN ||--o{ EGO_TELEMETRY_FRAME : "records"
    SIMULATION_RUN ||--o{ POTHOLE_EVENT : "logs"
    SIMULATION_RUN ||--o{ NEIGHBOR_RELATIONSHIP : "tracks"

    VEHICLE ||--o{ BSM_CORE : "broadcasts"
    VEHICLE ||--o{ DENM : "sends"
    VEHICLE ||--o{ POTHOLE_EVENT : "experiences"

    BSM_CORE ||--o| BSM_PART2 : "extended by"
    BSM_CORE ||--o| V2V_ENHANCED_MESSAGE : "carried in"
    BSM_CORE ||--o| CAM : "converted to"

    VEHICLE ||--o{ NEIGHBOR_RELATIONSHIP : "is source"
    VEHICLE ||--o{ NEIGHBOR_RELATIONSHIP : "is target"

    POTHOLE ||--o{ POTHOLE_EVENT : "triggers"
    POTHOLE ||--o{ EGO_TELEMETRY_FRAME : "referenced as closest"
```

## Key Design Decisions

| Choice | Rationale |
|--------|-----------|
| `SIMULATION_RUN` as root | All entities scoped to a run via FK, matching the `data/runs/<run_id>/` directory structure |
| `BSM_CORE` → `V2V_ENHANCED_MESSAGE` / `CAM` as separate tables | `V2VEnhancedMessage` and `CAM` are derived views of a BSM; FK avoids field duplication |
| `BSM_PART2` optional (zero-or-one) | Matches `bsm_part2: Optional[BSMPartII]` in the dataclass |
| `NEIGHBOR_RELATIONSHIP` with two FKs to `VEHICLE` | Models bidirectional detection with `source_vehicle_id` / `target_vehicle_id` and stores computed `threat_level` + `ttc_s` |
| `DSRC_CONFIG` separate from `SIMULATION_CONFIG` | Mirrors the split between `DSRCConfig` and `SimulationConfig` in `src/config.py` |
| `POTHOLE_EVENT.event_type` | Captures `ENTER`/`EXIT` lifecycle events from `pothole_events.csv` |
