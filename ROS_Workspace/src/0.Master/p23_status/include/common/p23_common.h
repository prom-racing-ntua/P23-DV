#ifndef P23_COMMON
#define P23_COMMON

#include <string>
#include <unordered_map>


namespace p23
{
// List of available Missions
typedef enum Mission {
    MISSION_UNLOCKED = 0,
    ACCELERATION = 1,
    SKIDPAD = 2,
    AUTOX = 3,
    TRACKDRIVE = 4,
    EBS_TEST = 5,
    INSPECTION = 6,
    MANUAL = 7
} Mission;

const std::unordered_map<Mission, std::string> mission_list{
    { MISSION_UNLOCKED, "MISSION_UNLOCKED" },
    { ACCELERATION, "ACCELERATION" },
    { SKIDPAD, "SKIDPAD" },
    { AUTOX, "AUTOX" },
    { TRACKDRIVE, "TRACKDRIVE" },
    { EBS_TEST,"EBS_TEST" },
    { INSPECTION, "INSPECTION" },
    { MANUAL, "MANUAL" }
};

// List of available Autonomous System Statuses
typedef enum AS_Status {
    AS_OFF = 1,
    AS_READY = 2,
    AS_DRIVING = 3,
    AS_FINISHED = 4,
    AS_EMERGENCY = 5
} AS_Status;

const std::unordered_map<AS_Status, std::string> autonomous_status_list{
    { AS_OFF, "AS_OFF" },
    { AS_READY, "AS_READY" },
    { AS_DRIVING, "AS_DRIVING" },
    { AS_FINISHED, "AS_FINISHED" },
    { AS_EMERGENCY, "AS_EMERGENCY" }
};

// List of available Driverless System Statuses
typedef enum DV_Status {
    STARTUP = 0,
    LV_ON = 1,
    MISSION_SELECTED = 2,
    DV_READY = 3,
    DV_DRIVING = 4,
    MISSION_FINISHED = 5,
    NODE_PROBLEM = 6
} DV_Status;

const std::unordered_map<DV_Status, std::string> driverless_status_list{
    { STARTUP, "STARTUP" },
    { LV_ON, "LV_ON" },
    { MISSION_SELECTED, "MISSION_SELECTED" },
    { DV_READY, "DV_READY" },
    { DV_DRIVING, "DV_DRIVING" },
    { MISSION_FINISHED, "MISSION_FINISHED" },
    { NODE_PROBLEM, "NODE_PROBLEM" }
};

// List of available transitions of the DV System
typedef enum DV_Transitions {
    ON_STARTUP = 0,
    SHUTDOWN_NODES = 1,
    ON_MISSION_LOCKED = 2,
    ON_MISSION_UNLOCKED = 3,
    ON_AS_READY = 4,
    ON_AS_DRIVING = 5,
} DV_Transitions;

const std::unordered_map<DV_Transitions, std::string> transition_list{
    { ON_STARTUP, "ON_STARTUP" },
    { SHUTDOWN_NODES, "SHUTDOWN_NODES" },
    { ON_MISSION_LOCKED, "ON_MISSION_LOCKED" },
    { ON_MISSION_UNLOCKED, "ON_MISSION_UNLOCKED" },
    { ON_AS_READY, "ON_AS_READY" },
    { ON_AS_DRIVING, "ON_AS_DRIVING" }
};

}
#endif // P23_COMMON