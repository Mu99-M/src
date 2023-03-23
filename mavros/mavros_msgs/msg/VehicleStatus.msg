# Represent Vehicle Status.
#
# VEHICLE_Status message

std_msgs/Header header
uint64 armed_time # Arming timestamp (microseconds)
uint64 takeoff_time # Takeoff timestamp (microseconds)

uint8 arming_state

uint8 latest_arming_reason
uint8 latest_disarming_reason

uint64 nav_state_timestamp # time when current nav_state activated

uint8 nav_state_user_intention                  # Mode that the user selected (might be different from nav_state in a failsafe situation)

uint8 nav_state                                 # Currently active mode

# Bitmask of detected failures
uint16 failure_detector_status

uint8 hil_state

# If it's a VTOL, then the value will be VEHICLE_TYPE_ROTARY_WING while flying as a multicopter, and VEHICLE_TYPE_FIXED_WING when flying as a fixed-wing
uint8 vehicle_type

bool failsafe # true if system is in failsafe state (e.g.:RTL, Hover, Terminate, ...)
bool failsafe_and_user_took_over # true if system is in failsafe state but the user took over control

# Link loss
bool gcs_connection_lost              # datalink to GCS lost
uint8 gcs_connection_lost_counter     # counts unique GCS connection lost events
bool high_latency_data_link_lost # Set to true if the high latency data link (eg. RockBlock Iridium 9603 telemetry module) is lost

# VTOL flags
bool is_vtol             # True if the system is VTOL capable
bool is_vtol_tailsitter  # True if the system performs a 90° pitch down rotation during transition from MC to FW
bool in_transition_mode  # True if VTOL is doing a transition
bool in_transition_to_fw # True if VTOL is doing a transition from MC to FW
