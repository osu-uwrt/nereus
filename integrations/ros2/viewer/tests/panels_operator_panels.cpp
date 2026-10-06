#include "nereus/ros_viewer/panels/composition.hpp"
#include "nereus/ros_viewer/panels/ros_providers.hpp"
#include <cassert>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#ifdef NEREUS_VIEWER_HAVE_CHAMELEON
#include <chameleon_tf_msgs/action/model_frame.hpp>
#endif
#include <functional>
#include <imgui.h>
#include <imgui_internal.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <riptide_msgs2/action/mag_cal.hpp>
#include <riptide_msgs2/action/tare_gyro.hpp>
#include <riptide_msgs2/msg/actuator_status.hpp>
#include <riptide_msgs2/msg/battery_status.hpp>
#include <riptide_msgs2/msg/electrical_command.hpp>
#include <riptide_msgs2/msg/gyro_status.hpp>
#include <riptide_msgs2/msg/mapping_target_info.hpp>
#include <riptide_msgs2/msg/u_int8_stamped.hpp>
#include <riptide_msgs2/srv/mapping_target.hpp>
#include <riptide_msgs2/srv/query_imu_serial.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>
#include <unistd.h>
#include <visualization_msgs/msg/marker_array.hpp>
#ifdef NEREUS_VIEWER_HAVE_ZED
#include <zed_msgs/srv/start_svo_rec.hpp>
#endif
using namespace nereus::ros_viewer::panels;
using namespace std::chrono_literals;
#ifdef NEREUS_VIEWER_HAVE_CHAMELEON // the tag-calibration action is optional
using Cal = chameleon_tf_msgs::action::ModelFrame;
using Goal = rclcpp_action::ServerGoalHandle<Cal>;
#endif
using Target = riptide_msgs2::srv::MappingTarget;
using Reset = std_srvs::srv::Trigger;
int main(int argc, char **argv) {
    setenv("ROS_DOMAIN_ID", "184", 1);
    setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp", 1);
    rclcpp::init(argc, argv);
    auto ns = "new_panels_" + std::to_string(getpid());
    auto node = std::make_shared<rclcpp::Node>("mock", "/" + ns);
    Registry registry;
    registerPanels(registry);
    RosProviders ros;
    ros.registerFactories(registry);
    // Host items (scene_settings, detections, ...) live in the application; stand in for them here. The item
    // after the simulation tool ("pool_viewer") records the simulation button's rectangle.
    ImVec2 simulationButtonMin, simulationButtonMax;
    struct Stub final : Panel {
        std::function<void()> bar;
        void toolbar() override {
            if (bar)
                bar();
        }
        void draw() override {}
    };
    auto config = YAML::LoadFile(argv[1]);
    for (const char *group : {"toolbar", "panels"})
        for (const auto &item : config[group]) {
            const auto type = item["type"].as<std::string>();
            if (registry.panels.count(type))
                continue;
            registry.panels.emplace(type, ViewFactory<Panel>{Kind::Motion, [](const YAML::Node &) {},
                                                             [&, type](const Binding &) {
                                                                 auto stub = std::make_unique<Stub>();
                                                                 if (type == "pool_viewer")
                                                                     stub->bar = [&] {
                                                                         simulationButtonMin = ImGui::GetItemRectMin();
                                                                         simulationButtonMax = ImGui::GetItemRectMax();
                                                                     };
                                                                 return std::unique_ptr<Panel>(std::move(stub));
                                                             },
                                                             true, false});
        }
    const auto toolsConfig = YAML::LoadFile(argv[2]);
    for (const auto &entry : toolsConfig["providers"])
        config["providers"][entry.first.as<std::string>()] = YAML::Clone(entry.second);
    // The shipped toolbar, with the simulation tool moved just before the recording stub ("pool_viewer").
    YAML::Node toolbar(YAML::NodeType::Sequence), simulationTool;
    for (const auto &item : toolsConfig["toolbar"])
        if (item["type"].as<std::string>() == "simulation")
            simulationTool = YAML::Clone(item);
    assert(simulationTool);
    for (const auto &item : toolsConfig["toolbar"]) {
        const auto type = item["type"].as<std::string>();
        if (type == "pool_viewer")
            toolbar.push_back(simulationTool);
        if (type != "simulation")
            toolbar.push_back(YAML::Clone(item));
    }
    config["toolbar"] = toolbar;
    for (auto target : config["providers"]["bags"]["options"]["targets"]) // bag targets: never ssh to the robot
        target["ssh"] = YAML::Load("[\"false\"]");
    config["providers"]["simulation"]["options"]["node"] = "mock";
    config["providers"]["simulation"]["options"]["request_timeout"] = .75;
    node->declare_parameter<double>("real_time_factor", 1.0);
    int syncRequests = 0, resetSimRequests = 0;
    auto syncService = node->create_service<Reset>("sync_sim_to_estimate",
                                                   [&](Reset::Request::SharedPtr, Reset::Response::SharedPtr reply) {
                                                       ++syncRequests;
                                                       reply->success = true;
                                                       reply->message = "Aligned to estimate";
                                                   });
    std::shared_ptr<rmw_request_id_t> resetSimHeader;
    auto resetSimService = node->create_service<Reset>(
        "reset_sim_to_start", [&](std::shared_ptr<rmw_request_id_t> header, Reset::Request::SharedPtr) {
            ++resetSimRequests;
            resetSimHeader = header;
        });
    int speedRequests = 0;
    auto parameterCallback =
        node->add_on_set_parameters_callback([&](const std::vector<rclcpp::Parameter> &parameters) {
            rcl_interfaces::msg::SetParametersResult response;
            response.successful = true;
            for (const auto &parameter : parameters)
                if (parameter.get_name() == "real_time_factor") {
                    ++speedRequests;
                    response.successful = parameter.as_double() != 3;
                    response.reason = response.successful ? "" : "test rejection";
                }
            return response;
        });
    // Only the providers exercised here; remap all mapping endpoints to prove configurability.
    config["providers"].remove("motion");
    config["providers"].remove("mission");
    config.remove("overlays");
    config.remove("ownership");
    config["panels"] = YAML::Load(R"([
      {id: mapping, type: mapping, provider: mapping, options: {parent_frame: test_world, tag_frame: test_tag}},
      {id: actuators, type: actuators, provider: actuators},
      {id: telemetry, type: telemetry, provider: telemetry},
      {id: recording, type: recording, provider: recording,
       options: {path: "~/svos/test", home: /home/test, timestamp: false}},
      {id: electrical, type: electrical, provider: electrical}])");
    // Telemetry from the test namespace only (the configured CPU source is the global /diagnostics_agg).
    auto readings = config["providers"]["telemetry"]["options"]["readings"];
    readings[0]["timeout"] = .3;
    readings[1]["topic"] = "diagnostics_agg";
    readings[1]["timeout"] = .3;
    readings[2]["timeout"] = .3;
    readings[3]["timeout"] = .3;
    config["providers"]["recording"]["options"]["request_timeout"] = .3;
    auto mapCfg = config["providers"]["mapping"]["options"];
    mapCfg["calibration_action"] = "calibration";
    mapCfg["reset_service"] = "reset";
    mapCfg["target_service"] = "target";
    mapCfg["status_topic"] = "mapping_state";
    mapCfg["request_timeout"] = .25;
    mapCfg["status_timeout"] = .3;
    config["providers"]["actuators"]["options"]["status_timeout"] = .3;
    config["providers"]["run"]["options"]["status_timeout"] = .3;
    Context ctx{ns, "test_world", false, false};
    ctx.documents["task"] = YAML::Load(R"(
ui:
  title: Test scorecard
  run_options:
    - {key: role, label: Role, type: choice, default: repair, choices: [{value: repair, label: Repair}]}
    - {key: coin, label: Coin, type: bool, default: true}
    - {key: amount, label: Amount, type: number, default: 1}
  manual_adjustment: true
)");
    ctx.initialWindows = {"run"};
    auto composition = std::make_unique<Composition>(config, ctx, registry);
    auto mapping = std::dynamic_pointer_cast<Mapping>(composition->providers().at("mapping"));
    auto actuators = std::dynamic_pointer_cast<Actuators>(composition->providers().at("actuators"));
    auto run = std::dynamic_pointer_cast<Run>(composition->providers().at("run"));
    auto simulation = std::dynamic_pointer_cast<Simulation>(composition->providers().at("simulation"));
    auto telemetry = std::dynamic_pointer_cast<Telemetry>(composition->providers().at("telemetry"));
    auto recording = std::dynamic_pointer_cast<Recording>(composition->providers().at("recording"));
    auto electrical = std::dynamic_pointer_cast<Electrical>(composition->providers().at("electrical"));
    // Electrical: the RViz electrical panel's endpoints.
    using MagCal = riptide_msgs2::action::MagCal;
    using TareGyro = riptide_msgs2::action::TareGyro;
    using ImuSerial = riptide_msgs2::srv::QueryImuSerial;
    std::vector<uint8_t> electricalCommands, ivcSent;
    std::vector<int> pingerFrequencies;
    int pingerEnables = 0;
    bool lastPingerEnable = false;
    auto electricalSub = node->create_subscription<riptide_msgs2::msg::ElectricalCommand>(
        "command/electrical", 10,
        [&](const riptide_msgs2::msg::ElectricalCommand &m) { electricalCommands.push_back(m.command); });
    auto pingerEnableSub =
        node->create_subscription<std_msgs::msg::Bool>("ivc/pinger/enable", 10, [&](const std_msgs::msg::Bool &m) {
            ++pingerEnables;
            lastPingerEnable = m.data;
        });
    auto pingerFrequencySub = node->create_subscription<std_msgs::msg::Int32>(
        "ivc/pinger/set_freq_broker_khz", 10,
        [&](const std_msgs::msg::Int32 &m) { pingerFrequencies.push_back(m.data); });
    auto pingerSelectedPub = node->create_publisher<std_msgs::msg::Int32>("ivc/pinger/selected_freq_khz", 10);
    auto pingerAmplitudePub = node->create_publisher<std_msgs::msg::Float32>("ivc/pinger/selected_freq_amp_stream", 10);
    auto ivcTxSub = node->create_subscription<std_msgs::msg::UInt8>(
        "ivc/tx", 10, [&](const std_msgs::msg::UInt8 &m) { ivcSent.push_back(m.data); });
    auto ivcRxPub = node->create_publisher<std_msgs::msg::UInt8>("ivc/rx", 10);
    auto ivcConfirmPub = node->create_publisher<riptide_msgs2::msg::UInt8Stamped>("ivc/tx_success", 10);
    std::vector<std::string> imuRequests;
    std::string imuReply = "$VNRRG,05,115200*58";
    auto imuService = node->create_service<ImuSerial>(
        "vectornav/config", [&](ImuSerial::Request::SharedPtr request, ImuSerial::Response::SharedPtr reply) {
            imuRequests.push_back(request->request);
            reply->response = imuReply;
        });
    std::shared_ptr<rclcpp_action::ServerGoalHandle<MagCal>> magGoal;
    auto magServer = rclcpp_action::create_server<MagCal>(
        node, "vectornav/mag_cal", [](const auto &, auto) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
        [](auto) { return rclcpp_action::CancelResponse::ACCEPT; }, [&](auto accepted) { magGoal = accepted; });
    std::shared_ptr<rclcpp_action::ServerGoalHandle<TareGyro>> tareGoal;
    auto tareServer = rclcpp_action::create_server<TareGyro>(
        node, "gyro/tare", [](const auto &, auto) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
        [](auto) { return rclcpp_action::CancelResponse::ACCEPT; }, [&](auto accepted) { tareGoal = accepted; });
    const auto reading = [&](const std::string &id) {
        for (const auto &r : telemetry->state().readings)
            if (r.id == id)
                return r;
        assert(false);
        return Reading{};
    };
    assert(reading("fog").level == Level::Stale && reading("fog").value == "--" && reading("cpu").value == "--");
    auto gyroPub = node->create_publisher<riptide_msgs2::msg::GyroStatus>("gyro/status", 10);
    auto diagPub = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics_agg", 10);
    riptide_msgs2::msg::GyroStatus gyroMsg;
    gyroMsg.connected = gyroMsg.temp_good = gyroMsg.vsupply_good = gyroMsg.sldcurrent_good = true;
    gyroMsg.diagsignal_good = gyroMsg.temp_within_cal = true;
    gyroMsg.temperature = 41.54;
    using Battery = riptide_msgs2::msg::BatteryStatus;
    auto batteryPub = node->create_publisher<Battery>("state/battery", 10);
    Battery portMsg, stbdMsg, unknownMsg; // both sides share the topic, told apart by `detect`
    portMsg.detect = Battery::DETECT_PORT;
    portMsg.soc = 87;
    portMsg.pack_voltage = 24.1f;
    portMsg.pack_current = -12.5f;
    portMsg.time_to_dischg = 63;
    portMsg.cell_name = "port_pack";
    stbdMsg.detect = Battery::DETECT_STBD;
    stbdMsg.soc = 35;
    unknownMsg.detect = Battery::DETECT_NONE;
    unknownMsg.soc = 1;
    diagnostic_msgs::msg::DiagnosticArray diagMsg;
    diagMsg.status.resize(2);
    diagMsg.status[0].name = "/Robot Diagnostics/Computers"; // other statuses in the array are ignored
    diagMsg.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    auto &core = diagMsg.status[1];
    core.name = "/Robot Diagnostics/Computers/Core Temperature";
    core.message = "Max core temp 72.00 C";
    for (const auto &[key, value] : std::initializer_list<std::pair<const char *, const char *>>{
             {"CPU Temperature", "58.41 C"}, {"SOC 0 Temperature", "72.00 C"}}) {
        diagnostic_msgs::msg::KeyValue pair;
        pair.key = key;
        pair.value = value;
        core.values.push_back(pair);
    }
    // Recording: stop services (DFC never answers), the picture taker, and SVO start when zed_msgs exists.
    int ffcStops = 0, captures = 0;
    auto ffcStop = node->create_service<Reset>("ffc/zed_node/stop_svo_rec",
                                               [&](Reset::Request::SharedPtr, Reset::Response::SharedPtr reply) {
                                                   ++ffcStops;
                                                   reply->success = true;
                                               });
    std::shared_ptr<rmw_request_id_t> dfcStopHeader;
    auto dfcStop = node->create_service<Reset>(
        "dfc/zed_node/stop_svo_rec",
        [&](std::shared_ptr<rmw_request_id_t> header, Reset::Request::SharedPtr) { dfcStopHeader = header; });
    auto captureService =
        node->create_service<Reset>("capture_image", [&](Reset::Request::SharedPtr, Reset::Response::SharedPtr reply) {
            ++captures;
            reply->success = true;
            reply->message = "Images saved - \nLeft: /tmp/left.png";
        });
#ifdef NEREUS_VIEWER_HAVE_ZED
    using StartSvo = zed_msgs::srv::StartSvoRec;
    std::string startedFile;
    auto ffcStart = node->create_service<StartSvo>(
        "ffc/zed_node/start_svo_rec", [&](StartSvo::Request::SharedPtr request, StartSvo::Response::SharedPtr reply) {
            startedFile = request->svo_filename;
            reply->success = true;
        });
#endif
    auto mappingPub = node->create_publisher<riptide_msgs2::msg::MappingTargetInfo>("mapping_state", 10);
    auto actuatorPub =
        node->create_publisher<riptide_msgs2::msg::ActuatorStatus>("state/actuator/status", rclcpp::SensorDataQoS());
    auto scorePub = node->create_publisher<std_msgs::msg::String>("simulator/run_score", 10);
    auto lightPub = node->create_publisher<visualization_msgs::msg::MarkerArray>("simulator/magnet_lights", 10);
    auto eventPub = node->create_publisher<std_msgs::msg::String>("simulator/task_events", 10);
    auto taskScorePub = node->create_publisher<std_msgs::msg::String>("simulator/task_score", 10);
    auto jointsPub = node->create_publisher<std_msgs::msg::Float64MultiArray>("simulator/claw_joints", 10);
    riptide_msgs2::msg::MappingTargetInfo mappingMsg;
    mappingMsg.target_object = "observed";
    mappingMsg.lock_map = true;
    riptide_msgs2::msg::ActuatorStatus actuatorMsg;
    actuatorMsg.torpedo_available_count = 2;
    std_msgs::msg::String scoreMsg;
    scoreMsg.data = R"({"running":false,"elapsed":2.5,"total":12,"rows":[{"label":"Award","points":12}]})";
    int armCount = 0, fireCount = 0, resetCount = 0;
    bool lastArm = false;
    std::vector<std::string> runCommands;
    auto armSub =
        node->create_subscription<std_msgs::msg::Bool>("command/actuator/arm", 10, [&](const std_msgs::msg::Bool &m) {
            ++armCount;
            lastArm = m.data;
        });
    auto fireSub = node->create_subscription<std_msgs::msg::Empty>("command/actuator/torpedo", 10,
                                                                   [&](const std_msgs::msg::Empty &) { ++fireCount; });
    auto resetSub = node->create_subscription<std_msgs::msg::Empty>(
        "simulator/reset_tasks", 10, [&](const std_msgs::msg::Empty &) { ++resetCount; });
    auto runSub = node->create_subscription<std_msgs::msg::String>(
        "simulator/run_command", 10, [&](const std_msgs::msg::String &m) { runCommands.push_back(m.data); });
    Target::Request targetRequest;
    int targetCount = 0;
    auto targetService =
        node->create_service<Target>("target", [&](Target::Request::SharedPtr req, Target::Response::SharedPtr) {
            targetRequest = *req;
            ++targetCount;
        });
    std::shared_ptr<rmw_request_id_t> resetHeader;
    auto resetService = node->create_service<Reset>(
        "reset", [&](std::shared_ptr<rmw_request_id_t> header, Reset::Request::SharedPtr) { resetHeader = header; });
    int calCount = 0;
#ifdef NEREUS_VIEWER_HAVE_CHAMELEON
    std::shared_ptr<Goal> goal;
    bool reject = false;
    auto server = rclcpp_action::create_server<Cal>(
        node, "calibration",
        [&](const auto &, auto) {
            return reject ? rclcpp_action::GoalResponse::REJECT : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](auto) { return rclcpp_action::CancelResponse::ACCEPT; },
        [&](auto accepted) {
            goal = accepted;
            ++calCount;
        });
#endif
    ros.start();
    auto spin = [&](double seconds, bool publish = true) {
        auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
        while (std::chrono::steady_clock::now() < end) {
            if (publish) {
                mappingPub->publish(mappingMsg);
                actuatorPub->publish(actuatorMsg);
                scorePub->publish(scoreMsg);
                gyroPub->publish(gyroMsg);
                diagPub->publish(diagMsg);
                batteryPub->publish(portMsg);
                batteryPub->publish(stbdMsg);
                batteryPub->publish(unknownMsg);
            }
            rclcpp::spin_some(node);
#ifdef NEREUS_VIEWER_HAVE_CHAMELEON
            if (goal && goal->is_canceling()) {
                goal->canceled(std::make_shared<Cal::Result>());
                goal.reset();
            }
#endif
            std::this_thread::sleep_for(5ms);
        }
    };
    spin(1.2);
#ifdef NEREUS_VIEWER_HAVE_CHAMELEON
    assert(mapping->state().fresh && mapping->state().calibrationReady);
#else
    assert(mapping->state().fresh);
#endif
    assert(run->state().fresh && actuators->state().fresh);
    // FOG: temperature and the driver's flags; CPU: the hottest core value with the status level.
    assert(reading("fog").value == "41.5\u00B0C" && reading("fog").level == Level::Ok);
    assert(reading("cpu").value == "72.0\u00B0C" && reading("cpu").level == Level::Warn);
    assert(reading("cpu").detail.find("SOC 0 Temperature: 72.00 C") != std::string::npos);
    assert(reading("port").value == "87%" && reading("port").level == Level::Ok);
    assert(reading("port").detail.find("24.10 V, -12.50 A") != std::string::npos &&
           reading("port").detail.find("63 min") != std::string::npos);
    assert(reading("stbd").value == "35%" && reading("stbd").level == Level::Warn);
    stbdMsg.soc = 19;
    spin(.1);
    assert(reading("stbd").level == Level::Error && reading("port").value == "87%");
    // Both packs back to back, as the robot sends them: neither may displace the other.
    portMsg.soc = 64;
    stbdMsg.soc = 63;
    for (int burst = 0; burst < 3; ++burst) {
        batteryPub->publish(portMsg);
        batteryPub->publish(stbdMsg);
        spin(.1, false);
    }
    assert(reading("port").value == "64%" && reading("stbd").value == "63%");
    stbdMsg.soc = 19;
    gyroMsg.temperature = 60;
    spin(.1);
    assert(reading("fog").level == Level::Warn);
    gyroMsg.temperature = 41;
    gyroMsg.temp_within_cal = false;
    spin(.1);
    assert(reading("fog").level == Level::Warn && reading("fog").detail.find("calibration") != std::string::npos);
    gyroMsg.temp_good = false;
    spin(.1);
    assert(reading("fog").level == Level::Error && reading("fog").detail.rfind("Overheating", 0) == 0);
    gyroMsg.connected = false;
    spin(.1);
    assert(reading("fog").level == Level::Error && reading("fog").value == "--");
    gyroMsg.connected = gyroMsg.temp_good = gyroMsg.temp_within_cal = true;
    core.values.clear();
    core.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    spin(.1);
    assert(reading("cpu").value == "--" && reading("cpu").level == Level::Error && reading("fog").level == Level::Ok);
    core.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    spin(.1);
    assert(reading("cpu").level == Level::Stale);
    // Recording and capture.
    auto rec = recording->state();
    assert(rec.captureReady && rec.cameras.size() == 2 && rec.cameras[0].stopReady && rec.cameras[1].stopReady);
    recording->capture();
    recording->capture(); // one request at a time
    spin(.15);
    rec = recording->state();
    assert(captures == 1 && !rec.capturing && rec.captureMessage.rfind("Images saved", 0) == 0);
    recording->stop("ffc");
    spin(.15);
    assert(ffcStops == 1 && recording->state().cameras[0].message == "Stopped");
    recording->stop("dfc");
    spin(.1);
    assert(dfcStopHeader && recording->state().cameras[1].pending);
    spin(.35);
    assert(!recording->state().cameras[1].pending &&
           recording->state().cameras[1].message.find("timed out") != std::string::npos);
#ifdef NEREUS_VIEWER_HAVE_ZED
    assert(recording->state().svoSupported && recording->state().cameras[0].startReady &&
           !recording->state().cameras[1].startReady);
    recording->start("ffc", "/home/test/svos/test_ffc.svo2");
    spin(.15);
    rec = recording->state();
    assert(startedFile == "/home/test/svos/test_ffc.svo2" && rec.cameras[0].recording &&
           rec.cameras[0].file == startedFile);
    recording->start("ffc", "/other.svo2"); // already recording
    spin(.1);
    assert(startedFile == "/home/test/svos/test_ffc.svo2");
    recording->stop("ffc");
    spin(.15);
    assert(ffcStops == 2 && !recording->state().cameras[0].recording);
#else
    recording->start("ffc", "/home/test/svos/test_ffc.svo2");
    assert(!recording->state().svoSupported && !recording->state().cameras[0].pending);
#endif
    // Electrical: power commands publish their configured value; power cuts are marked for confirmation.
    auto elec = electrical->state();
    assert(elec.commands.size() == 11 && elec.hasImu && elec.hasTare && elec.hasPinger && elec.hasIvc);
    assert(elec.magCalReady && elec.registerReady && elec.tareReady);
    for (const auto &item : elec.commands)
        assert(item.confirm ==
               (item.id == "cycle_computer" || item.id == "cycle_robot" || item.id == "kill_robot_power"));
    electrical->command("kill_robot_power");
    electrical->command("enable_leds");
    electrical->command("nope");
    spin(.15);
    assert((electricalCommands == std::vector<uint8_t>{4, 9}));
    // The pinger enable state is re-sent every second, and at once when it changes.
    assert(pingerEnables >= 1 && lastPingerEnable);
    electrical->setPingerEnabled(false);
    spin(.15);
    const int enables = pingerEnables;
    assert(!lastPingerEnable);
    spin(1.1);
    assert(pingerEnables > enables && !lastPingerEnable);
    electrical->setPingerEnabled(true);
    electrical->setPingerFrequency(30);
    std_msgs::msg::Int32 selected;
    selected.data = 30;
    pingerSelectedPub->publish(selected);
    std_msgs::msg::Float32 amplitude;
    amplitude.data = .25f;
    pingerAmplitudePub->publish(amplitude);
    spin(.15);
    elec = electrical->state();
    assert(pingerFrequencies == std::vector<int>{30} && elec.pingerSelected == 30 && elec.pingerAmplitude == .25f);
    // IVC: header in the top 3 bits, status / command in the low 5; out-of-range sends are dropped.
    electrical->sendIvc(1, 2);
    electrical->sendIvc(2, 17);
    electrical->sendIvc(1, 5); // only 5 statuses
    electrical->sendIvc(3, 0); // only 3 headers
    std_msgs::msg::UInt8 rx;
    rx.data = 0x01; // tank_cmd_tank_status: talos_state_deploy_tank
    ivcRxPub->publish(rx);
    riptide_msgs2::msg::UInt8Stamped confirm;
    confirm.data = 34;
    ivcConfirmPub->publish(confirm);
    spin(.15);
    assert((ivcSent == std::vector<uint8_t>{34, 81}));
    elec = electrical->state();
    const auto logged = [&](const std::string &text) {
        return std::any_of(elec.ivcLog.begin(), elec.ivcLog.end(),
                           [&](const std::string &line) { return line.find(text) != std::string::npos; });
    };
    assert(elec.ivcLog.size() == 4 && logged("SEND  talos_cmd_talos_status: talos_state_tank_go  (1-2)") &&
           logged("SEND  talos_cmd_fish_heading: 17") &&
           logged("RECV  tank_cmd_tank_status: talos_state_deploy_tank") && logged("ACK "));
    // IMU registers: the reply's register fields fill the value; VectorNav error replies are reported.
    electrical->readRegister("05");
    electrical->readRegister("06"); // one request at a time
    spin(.2);
    elec = electrical->state();
    assert((imuRequests == std::vector<std::string>{"$VNRRG,05"}) && elec.registerValue == "115200" &&
           elec.registerMessage == "Read register 05");
    electrical->writeRegister("05", "9600");
    electrical->writeRegister("x5", "9600"); // not a register number
    spin(.2);
    imuReply = "$VNERR,03*72";
    electrical->saveImuSettings();
    spin(.2);
    elec = electrical->state();
    assert(imuRequests.size() == 3 && imuRequests[1] == "$VNWRG,05,9600" && imuRequests[2] == "$VNWNV" &&
           elec.registerMessage.find("IMU error") != std::string::npos && elec.registerValue == "115200");
    // Mag cal: progress from the shrinking deviation; FOG tare: the goal's samples / timeout and abort reason.
    electrical->startMagCal();
    spin(.2);
    assert(magGoal && electrical->state().magCalRunning);
    auto magFeedback = std::make_shared<MagCal::Feedback>();
    magFeedback->curr_avg_dev[0] = 4;
    magGoal->publish_feedback(magFeedback);
    spin(.1);
    magFeedback->curr_avg_dev[0] = 1;
    magGoal->publish_feedback(magFeedback);
    spin(.1);
    assert(std::abs(electrical->state().magCalProgress - .75f) < 1e-4);
    magGoal->succeed(std::make_shared<MagCal::Result>());
    magGoal.reset();
    spin(.15);
    elec = electrical->state();
    assert(!elec.magCalRunning && elec.magCalProgress == 1 && elec.magCalMessage == "Mag cal complete");
    electrical->startTare(5000, 7.5);
    spin(.2);
    assert(tareGoal && tareGoal->get_goal()->num_samples == 5000 && tareGoal->get_goal()->timeout_seconds == 7.5);
    auto tareResult = std::make_shared<TareGyro::Result>();
    tareResult->result = "drift too high";
    tareGoal->abort(tareResult);
    tareGoal.reset();
    spin(.15);
    assert(!electrical->state().tareRunning && electrical->state().tareMessage == "Tare aborted: drift too high");
    assert(calCount == 0 && armCount == 0 && fireCount == 0 && resetCount == 0 && runCommands.empty());
    assert(simulation->state().connected && simulation->state().rate == 1 && speedRequests == 0);
    simulation->setRate(0);
    spin(.7);
    assert(simulation->state().rate == 0 && node->get_parameter("real_time_factor").as_double() == 0);
    simulation->setRate(2);
    spin(.7);
    assert(simulation->state().rate == 2 && speedRequests == 2);
    simulation->setRate(3);
    spin(.7);
    assert(simulation->state().rate == 2 && simulation->state().message.find("test rejection") != std::string::npos);
    simulation->setRate(-1);
    simulation->setRate(100);
    spin(.1);
    assert(speedRequests == 3);
    node->set_parameter(rclcpp::Parameter("real_time_factor", .5));
    spin(.7);
    assert(simulation->state().rate == .5); // observe external speed changes as well
    simulation->setPaused(true);
    spin(.7);
    assert(simulation->state().rate == 0 && simulation->state().resumeRate == .5);
    simulation->sync();
    spin(.15);
    assert(syncRequests == 1 && !simulation->state().operationPending &&
           simulation->state().operationMessage == "Sync: Aligned to estimate");
    simulation->setPaused(false);
    spin(.7);
    assert(simulation->state().rate == .5); // resume the last nonzero speed
    simulation->reset();
    simulation->reset();
    spin(.1);
    assert(resetSimRequests == 1 && simulation->state().operationPending);
    Reset::Response simResetReply;
    simResetReply.success = false;
    simResetReply.message = "not ready";
    resetSimService->send_response(*resetSimHeader, simResetReply);
    spin(.15);
    assert(simulation->state().operationMessage == "Reset failed: not ready");
    simulation->reset();
    spin(.95);
    assert(!simulation->state().operationPending &&
           simulation->state().operationMessage.find("timed out") != std::string::npos);
    simResetReply.success = true;
    resetSimService->send_response(*resetSimHeader, simResetReply);
    spin(.1);
    assert(simulation->state().operationMessage.find("timed out") != std::string::npos);
    // Only Run tracking subscribes to simulation feedback; Actuators does not.
    assert(lightPub->get_subscription_count() == 1 && eventPub->get_subscription_count() == 1 &&
           jointsPub->get_subscription_count() == 1);
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker light;
    light.ns = "test_target";
    light.color.g = 1;
    markers.markers.push_back(light);
    lightPub->publish(markers);
    std_msgs::msg::Float64MultiArray jaws;
    jaws.data = {.01, .02};
    jointsPub->publish(jaws);
    std_msgs::msg::String summary;
    summary.data = "Task completed";
    taskScorePub->publish(summary);
    for (int i = 0; i < 7; ++i) {
        std_msgs::msg::String event;
        event.data = "event " + std::to_string(i);
        eventPub->publish(event);
        spin(.02);
    }
    spin(.1);
    assert(run->state().magnetTargets.size() == 1 && run->state().magnetTargets[0].second);
    assert(run->state().taskSummary == "Task completed" && run->state().events.size() == 5 &&
           run->state().events.front() == "event 6");
    assert(run->state().simulationReadings[0].second == "30 mm");
    for (const auto &reading : actuators->state().readings)
        assert(reading.first != "test_target" && reading.first != "Jaw gap");
    for (const auto &action : actuators->state().actions)
        assert(action.id != "reset");
    std_msgs::msg::String resetEvent;
    resetEvent.data = "{kind: tasks, result: reset, target: all}";
    eventPub->publish(resetEvent);
    spin(.1);
    assert(run->state().events.size() == 1);
    mapping->setTarget("new_target", false);
    spin(.2);
    assert(targetCount == 1 && targetRequest.target_info.target_object == "new_target" &&
           !targetRequest.target_info.lock_map);
    assert(mapping->state().target == "observed" && mapping->state().locked); // request never fabricates observed state
#ifdef NEREUS_VIEWER_HAVE_CHAMELEON
    mapping->calibrate("test_world", "test_tag", 17);
    spin(.15);
    assert(goal && goal->get_goal()->samples == 17 && goal->get_goal()->monitor_child == "test_tag");
    auto feedback = std::make_shared<Cal::Feedback>();
    feedback->sample_count = 4;
    goal->publish_feedback(feedback);
    spin(.1);
    assert(mapping->state().samples == 4);
    auto result = std::make_shared<Cal::Result>();
    result->success = true;
    goal->succeed(result);
    goal.reset();
    spin(.15);
    assert(!mapping->state().calibrating && mapping->state().calibrationMessage == "Tag calibration complete");
    mapping->calibrate("test_world", "test_tag", 10);
    mapping->cancelCalibration();
    spin(.4);
    assert(!goal && !mapping->state().calibrating); // cancel before goal response
    reject = true;
    mapping->calibrate("test_world", "test_tag", 10);
    spin(.2);
    assert(!mapping->state().calibrating && mapping->state().calibrationMessage == "Calibration rejected");
    reject = false;
#endif
    mapping->reset();
    spin(.1);
    assert(resetHeader);
    Reset::Response resetReply;
    resetReply.success = false;
    resetReply.message = "blocked";
    resetService->send_response(*resetHeader, resetReply);
    resetHeader.reset();
    spin(.15);
    assert(mapping->state().resetMessage.find("failed: blocked") != std::string::npos);
    mapping->reset();
    spin(.45);
    assert(!mapping->state().resetting && mapping->state().resetMessage.find("timed out") != std::string::npos);
    resetReply.success = true;
    resetService->send_response(*resetHeader, resetReply);
    resetHeader.reset();
    spin(.1);
    assert(mapping->state().resetMessage.find("timed out") != std::string::npos);
    actuators->command("torpedo");
    actuators->command("arm");
    spin(.15);
    assert(fireCount == 0 && armCount == 1 && lastArm);
    actuatorMsg.actuators_armed = true;
    spin(.1);
    actuators->command("torpedo");
    actuators->command("arm");
    spin(.15);
    assert(fireCount == 1 && armCount == 2 && !lastArm);
    run->command(YAML::Load("{action: start, role: 'quoted \" role', coin: true, amount: 2.5}"));
    run->reset();
    spin(.15);
    assert(runCommands.size() == 1 && resetCount == 1);
    auto command = YAML::Load(runCommands.back());
    assert(command["coin"].as<bool>() && command["amount"].as<double>() == 2.5 &&
           command["role"].as<std::string>() == "quoted \" role");
    assert(runCommands.back().find("\"coin\": true") != std::string::npos &&
           runCommands.back().find("\"amount\": 2.5") != std::string::npos);
    scoreMsg.data = R"({"running":true,"elapsed":3,"total":12})";
    spin(.1);
    run->command(YAML::Load("{action: start}"));
    run->command(YAML::Load("{action: stop}"));
    spin(.1);
    assert(runCommands.size() == 2);
    spin(.5, false);
    assert(!mapping->state().fresh && !actuators->state().fresh && !run->state().fresh);
    assert(reading("fog").level == Level::Stale && reading("fog").value == "41.0\u00B0C"); // last value, muted
    assert(reading("stbd").level == Level::Stale && reading("stbd").value == "19%");
    actuators->command("torpedo");
    run->command(YAML::Load("{action: stop}"));
    run->reset();
    spin(.1, false);
    assert(fireCount == 1 && runCommands.size() == 2 && resetCount == 1);
    scoreMsg.data = "{running: true}";
    spin(.1);
    assert(!run->state().fresh && run->state().message.find("Invalid run score") != std::string::npos);
    // Draw all panels (including empty/preview snapshots) as windows, with the toolbar in a narrow window.
    ImGui::CreateContext();
    auto &io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.DisplaySize = {1000, 900};
    io.DeltaTime = 1.f / 30;
    unsigned char *pixels;
    int width, height;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
    auto draw = [&](Composition &c) {
        ImGui::NewFrame();
        c.drawPanels(); // created before the toolbar window, so they open behind it
        ImGui::SetNextWindowSize({320, 850});
        ImGui::Begin("test");
        c.drawToolbar();
        ImGui::TextUnformatted("header");
        c.drawHeader(300);
        ImGui::NewLine();
        c.drawPinned();
        ImGui::End();
        c.drawWindows();
        ImGui::Render();
    };
    for (int frame = 0; frame < 4; ++frame)
        draw(*composition);
    const auto anchor = simulationButtonMin;
    const auto bottom = simulationButtonMax.y + ImGui::GetStyle().ItemSpacing.y;
    io.AddMousePosEvent(anchor.x + 10, anchor.y + 10);
    draw(*composition);
    io.AddMouseButtonEvent(0, true);
    draw(*composition);
    io.AddMouseButtonEvent(0, false);
    // The Simulation button opens its window under the button, inside the application window; it stays open
    // (unlike the old popup) until the button or its close box is clicked again.
    const auto simulationWindow = [&]() -> ImGuiWindow * {
        for (auto *window : ImGui::GetCurrentContext()->Windows)
            if (window->Active && !window->Hidden &&
                std::string(window->Name).find("###simulation_") != std::string::npos)
                return window;
        return nullptr;
    };
    bool sawWindow = false;
    for (int frame = 0; frame < 4; ++frame) {
        draw(*composition);
        if (const auto *window = simulationWindow()) {
            sawWindow = true;
            assert(std::abs(window->Pos.x - anchor.x) < 1);
            assert(std::abs(window->Pos.y - bottom) < 1);
            assert(std::abs(window->Size.x - 340) < 1);
            assert(window->Pos.y + window->Size.y <= io.DisplaySize.y - 7);
        }
    }
    assert(sawWindow);
    io.AddKeyEvent(ImGuiKey_Escape, true);
    draw(*composition);
    io.AddKeyEvent(ImGuiKey_Escape, false);
    draw(*composition);
    assert(simulationWindow()); // a window, not a popup: Escape does not close it
    io.AddMousePosEvent(anchor.x + 10, anchor.y + 10);
    draw(*composition);
    io.AddMouseButtonEvent(0, true);
    draw(*composition);
    io.AddMouseButtonEvent(0, false);
    draw(*composition);
    draw(*composition);
    assert(!simulationWindow());
    auto checkScorecard = [&] {
        bool found = false;
        for (const auto *window : ImGui::GetCurrentContext()->Windows)
            if (window->Active && !window->Hidden &&
                std::string(window->Name).find("###scorecard_") != std::string::npos) {
                found = true;
                assert(window->Pos.x >= 11 && window->Pos.y >= 11);
                assert(window->Pos.x + window->Size.x <= io.DisplaySize.x - 11);
                assert(window->Pos.y + window->Size.y <= io.DisplaySize.y - 11);
            }
        assert(found);
    };
    draw(*composition);
    checkScorecard();
    io.DisplaySize = {500, 350};
    for (int frame = 0; frame < 3; ++frame)
        draw(*composition);
    checkScorecard();
    io.DisplaySize = {1000, 900};
    ctx.preview = true;
    Composition preview(config, ctx, registry);
    assert(preview.providers().empty());
    draw(preview);
    ctx.documents.clear();
    Composition noProfile(config, ctx, registry);
    draw(noProfile);
    ImGui::DestroyContext();
    ros.stop();
    composition.reset();
    mapping.reset();
    actuators.reset();
    telemetry.reset();
    recording.reset();
    electrical.reset();
    run.reset();
    simulation.reset();
    rclcpp::shutdown();
    std::cout << "Mapping, actuator and run panel checks passed\n";
}
