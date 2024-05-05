#include <future>
#include <iostream>
#include <thread>
#include <sstream>
#include <chrono>
#include <cmath>
#include <map>
#include <iomanip>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/info/info.h>

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;
bool return_launch = false;

enum Command {
    UPLOAD,
    UPLOAD_NO_RTL,
    STATUS,
    ADD_WAYPOINT,
    CLEAR,
    RESUME,
    START,
    PAUSE,
    RTL,
    TAKEOFF,
    LAND,
    UNKNOWN
};

std::map<std::string, Command> command_map = {
    {"upload", UPLOAD},
    {"upload_no_rtl", UPLOAD_NO_RTL},
    {"status", STATUS},
    {"add_waypoint", ADD_WAYPOINT},
    {"clear", CLEAR},
    {"resume", RESUME},
    {"start", START},
    {"pause", PAUSE},
    {"rtl", RTL},
    {"takeoff", TAKEOFF},
    {"land", LAND}};

Mission::MissionItem make_mission_item(
    double latitude_deg,
    double longitude_deg,
    float relative_altitude_m,
    float speed_m_s,
    bool is_fly_through,
    float gimbal_pitch_deg,
    float gimbal_yaw_deg,
    Mission::MissionItem::CameraAction camera_action)
{
    Mission::MissionItem new_item{};
    new_item.latitude_deg = latitude_deg;
    new_item.longitude_deg = longitude_deg;
    new_item.relative_altitude_m = relative_altitude_m;
    new_item.speed_m_s = speed_m_s;
    new_item.is_fly_through = is_fly_through;
    new_item.gimbal_pitch_deg = gimbal_pitch_deg;
    new_item.gimbal_yaw_deg = gimbal_yaw_deg;
    new_item.camera_action = camera_action;
    return new_item;
}

void upload_mission(Mission& mission, std::vector<Mission::MissionItem>& mission_items)
{
    std::cout << "Uploading mission...\n";
    Mission::MissionPlan mission_plan{};
    mission_plan.mission_items = mission_items;
    const Mission::Result upload_result = mission.upload_mission(mission_plan);

    if (upload_result != Mission::Result::Success) {
        std::cerr << "Mission upload failed: " << upload_result << ", exiting.\n";
    }
}

void clear_mission(Mission& mission, std::vector<Mission::MissionItem>& mission_items)
{
    auto prom = std::make_shared<std::promise<Mission::Result>>();
    auto future_result = prom->get_future();

    std::cout << "Clearing mission..." << std::endl;
    mission.clear_mission_async([prom](Mission::Result result) { prom->set_value(result); });

    const Mission::Result result = future_result.get();
    if (result != Mission::Result::Success) {
        std::cout << "Failed to clear mission (" << result << ")" << std::endl;
    } else {
        std::cout << "Mission cleared." << std::endl;
        mission_items.clear(); // Clear the mission_items vector
    }
}

void start_mission(Mission& mission)
{
    auto prom = std::make_shared<std::promise<Mission::Result>>();
    auto future_result = prom->get_future();
    mission.start_mission_async([prom](Mission::Result result) { prom->set_value(result); });

    const Mission::Result result = future_result.get(); // Wait on result
    if (result != Mission::Result::Success) {
        std::cout << "Mission start failed (" << result << "), exiting." << std::endl;
    }
    std::cout << "Started mission." << std::endl;
}

void pause_mission(Mission& mission)
{
    auto prom = std::make_shared<std::promise<Mission::Result>>();
    auto future_result = prom->get_future();

    std::cout << "Pausing mission..." << std::endl;
    mission.pause_mission_async([prom](Mission::Result result) { prom->set_value(result); });

    const Mission::Result result = future_result.get();
    if (result != Mission::Result::Success) {
        std::cout << "Failed to pause mission (" << result << ")" << std::endl;
    } else {
        std::cout << "Mission paused." << std::endl;
    }
}

void return_to_launch(Action& action)
{
    const Action::Result rtl_result = action.return_to_launch();
    if (rtl_result != Action::Result::Success) {
        // RTL failed, so exit (in reality might send kill command.)
        return;
    }
    std::cout << "Returning to launch..." << std::endl;
}

void get_position_info(Telemetry& telemetry)
{
    Telemetry::Position position_info;
    position_info = telemetry.position();
    std::cout << "Position: " << std::setprecision(17) << position_info.latitude_deg << ", "
              << position_info.longitude_deg << std::endl;
}

void add_waypoint(
    Mission& mission,
    std::vector<Mission::MissionItem>& mission_items,
    double latitude_deg,
    double longitude_deg,
    float relative_altitude_m = 10.0f,
    float speed_m_s = 5.0f)
{
    std::cout << "Adding waypoint..." << std::endl;
    Mission::MissionItem new_waypoint = make_mission_item(
        latitude_deg,
        longitude_deg,
        relative_altitude_m,
        speed_m_s,
        true,
        20.0f,
        60.0f,
        Mission::MissionItem::CameraAction::None);

    mission_items.push_back(new_waypoint);
}

void resume_mission(Mission& mission)
{
    auto prom = std::make_shared<std::promise<Mission::Result>>();
    auto future_result = prom->get_future();

    std::cout << "Resuming mission..." << std::endl;
    mission.start_mission_async([prom](Mission::Result result) { prom->set_value(result); });

    const Mission::Result result = future_result.get();
    if (result != Mission::Result::Success) {
        std::cout << "Failed to resume mission (" << result << ")" << std::endl;
    } else {
        std::cout << "Mission resumed." << std::endl;
    }
}

void drone_status(Telemetry& telemetry, Action& action, Mission& mission)
{
    get_position_info(telemetry);
    std::cout << "Loitering: " << telemetry.in_air() << std::endl;

    std::pair<Mission::Result, bool> mission_finished = mission.is_mission_finished();
    std::cout << "Mission Finished: " << mission_finished.second << std::endl;
    std::cout << "Mission Result: " << mission_finished.first << std::endl;

    if (telemetry.in_air() && mission_finished.second && !return_launch) {
        std::cout << "Mission Finished... landing" << std::endl;
        const auto land_result = action.land();
        if (land_result != Action::Result::Success) {
            std::cerr << "Landing failed: " << land_result << '\n';
        }
    }
}

void process_movement_command(
    const std::string& command,
    Action& action,
    Mission& mission,
    Telemetry& telemetry,
    std::vector<Mission::MissionItem>& mission_items)
{
    std::istringstream iss(command);
    std::string cmd;
    iss >> cmd;
    std::cout << "Command: " << cmd << std::endl;

    Command cmd_value = command_map.count(cmd) ? command_map[cmd] : UNKNOWN;

    switch (cmd_value) {
        case UPLOAD: {
            mission.set_return_to_launch_after_mission(true);
            upload_mission(mission, mission_items);
            return_launch = true;
            break;
        }
        case UPLOAD_NO_RTL: {
            mission.set_return_to_launch_after_mission(false);
            upload_mission(mission, mission_items);
            return_launch = false;
            break;
        }
        case STATUS: {
            drone_status(telemetry, action, mission);
            break;
        }
        case ADD_WAYPOINT: {
            double latitude_deg, longitude_deg;
            float relative_altitude_m, speed_m_s;
            iss >> latitude_deg >> longitude_deg >> relative_altitude_m >> speed_m_s;
            add_waypoint(
                mission,
                mission_items,
                latitude_deg,
                longitude_deg,
                relative_altitude_m,
                speed_m_s);
            break;
        }
        case CLEAR: {
            clear_mission(mission, mission_items);
            break;
        }
        case RESUME: {
            resume_mission(mission);
            break;
        }
        case START: {
            start_mission(mission);
            break;
        }
        case PAUSE: {
            pause_mission(mission);
            break;
        }
        case RTL: {
            return_to_launch(action);
            break;
        }
        case TAKEOFF: {
            std::cout << "Arming..." << std::endl;
            const Action::Result arm_result = action.arm();
            if (arm_result != Action::Result::Success) {
                std::cout << "Arming failed: " << arm_result << std::endl;
            }
            action.set_takeoff_altitude(5.f);
            std::cout << "Taking off..." << std::endl;
            while (true) {
                const Action::Result takeoff_result = action.takeoff();
                if (takeoff_result != Action::Result::Success) {
                    std::cout << "Takeoff failed!: " << takeoff_result << std::endl;
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    continue;
                }
                break;
            }
            break;
        }
        case LAND: {
            const Action::Result land_result = action.land();
            if (land_result != Action::Result::Success) {
                std::cerr << "Landing failed: " << land_result << '\n';
            }
            break;
        }
        default: {
            std::cout << "Unknown command: " << cmd << std::endl;
        }
    }
}

void usage(const std::string& bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udp://:14540\n";
}

int main(int argc, char** argv)
{
    if (argc != 2) {
        usage(argv[0]);
        return 1;
    }

    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::GroundStation}};
    ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << std::endl;
        return 1;
    }

    auto system = mavsdk.first_autopilot(3.0);
    if (!system) {
        std::cerr << "System not found" << std::endl;
        return 1;
    }

    auto action = Action{*system};
    auto telemetry = Telemetry{*system};
    auto mission = Mission{*system};

    while (!telemetry.health_all_ok()) {
        std::cout << "Waiting for system to be ready" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::cout << "System is ready" << std::endl;
    std::vector<Mission::MissionItem> mission_items;
    Mission::MissionProgress mission_progress;

    get_position_info(telemetry);
    while (true) {
        std::cout << "Drone Status: " << telemetry.landed_state() << std::endl;
        std::string command;
        std::getline(std::cin, command);
        process_movement_command(command, action, mission, telemetry, mission_items);
    }

    return 0;
}