#pragma once
#include "nereus/ros_viewer/panels/bag_recorder.hpp"
#include <cstdint>
#include <glm/glm.hpp>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace nereus::ros_viewer::panels {
using Pose = glm::mat4;
enum class Kind { Motion, Autonomy, Mapping, Actuators, Run, Simulation, Telemetry, Recording, Electrical, Bagging };
enum class Mode { Disabled, Position, Feedforward };
struct Provider {
    virtual ~Provider() = default;
    virtual Kind kind() const = 0;
    virtual void touch() {} // presentation liveness, never an enable request
};
struct MotionState {
    Pose actual{1}, commanded{1};
    bool fresh = false, enabled = false, pending = false, blocked = false, competing = false;
    bool hasCommand = false, supportsFeedforward = false;
    std::optional<bool> observedKilled;
    Mode mode = Mode::Disabled;
    uint64_t revision = 0;
    std::string frame, message = "Waiting for pose";
};
struct Motion : Provider {
    Kind kind() const final {
        return Kind::Motion;
    }
    virtual MotionState state() = 0;
    virtual void enable() = 0;
    virtual void kill() = 0;
    virtual void activate(Mode mode, const Pose &target) = 0;
    virtual void drag(const Pose &target) = 0;
    virtual void block(bool active) = 0;
};
struct MissionState {
    bool connected = false, refreshing = false, busy = false, pending = false, stackStale = false;
    bool failed = false;
    std::string message = "Waiting for autonomy", activeTree;
    std::vector<std::string> trees, stack;
};
struct Autonomy : Provider {
    Kind kind() const final {
        return Kind::Autonomy;
    }
    virtual MissionState state() = 0;
    virtual void refresh() = 0;
    virtual void start(const std::string &tree) = 0;
    virtual void stop() = 0;
};
struct MappingState {
    bool calibrationReady = false, resetReady = false, targetReady = false, fresh = false;
    bool calibrating = false, canceling = false, resetting = false, settingTarget = false;
    bool locked = false;
    unsigned samples = 0;
    std::string target, calibrationMessage = "Waiting for calibration server", resetMessage, targetMessage;
};
struct Mapping : Provider {
    Kind kind() const final {
        return Kind::Mapping;
    }
    virtual MappingState state() = 0;
    virtual void calibrate(const std::string &parent, const std::string &child, unsigned samples) = 0;
    virtual void cancelCalibration() = 0;
    virtual void reset() = 0;
    virtual void setTarget(const std::string &, bool locked) = 0;
};
struct ActuatorAction {
    std::string id, label;
    bool available = false;
};
struct ActuatorState {
    bool fresh = false, armed = false;
    std::vector<ActuatorAction> actions;
    std::vector<std::pair<std::string, std::string>> readings;
    std::string message = "Waiting for actuator status";
};
struct Actuators : Provider {
    Kind kind() const final {
        return Kind::Actuators;
    }
    virtual ActuatorState state() = 0;
    virtual void command(const std::string &id) = 0;
};
// Run schemas and snapshots are structured documents: competition-specific fields
// remain in the profile and score publisher, never in the viewer or ROS UI layer.
struct RunState {
    bool fresh = false;
    YAML::Node score;
    std::string message = "Waiting for run tracking";
    std::string taskSummary;
    std::vector<std::pair<std::string, bool>> magnetTargets;
    std::vector<std::pair<std::string, std::string>> simulationReadings;
    std::vector<std::string> events;
};
struct Run : Provider {
    Kind kind() const final {
        return Kind::Run;
    }
    virtual RunState state() = 0;
    virtual void command(const YAML::Node &) = 0;
    virtual void reset() = 0;
};
struct SimulationState {
    bool connected = false, pending = false;
    double rate = 1, resumeRate = 1, maxRate = 10;
    bool syncReady = false, resetReady = false, operationPending = false;
    std::string operationMessage;
    std::string message = "Waiting for simulator";
};
struct Simulation : Provider {
    Kind kind() const final {
        return Kind::Simulation;
    }
    virtual SimulationState state() = 0;
    virtual void setRate(double) = 0;
    virtual void setPaused(bool) = 0;
    virtual void sync() = 0;
    virtual void reset() = 0;
};
// Robot health readings shown at a glance (header chips): one value per configured source.
enum class Level { Ok, Warn, Error, Stale };
struct Reading {
    std::string id, label, value = "--", detail = "Waiting for data";
    Level level = Level::Stale;
};
struct TelemetryState {
    std::vector<Reading> readings;
};
struct Telemetry : Provider {
    Kind kind() const final {
        return Kind::Telemetry;
    }
    virtual TelemetryState state() = 0;
};
// Camera recording (SVO) and still capture. `recording` is what this viewer started and saw confirmed; the
// stack does not report it, so Stop stays available whenever the service is.
struct RecordingCamera {
    std::string id, label, file, message;
    bool startReady = false, stopReady = false, recording = false, pending = false;
    double elapsed = 0; // seconds since the confirmed start
};
struct RecordingState {
    std::vector<RecordingCamera> cameras;
    bool svoSupported = false, captureReady = false, capturing = false;
    std::string captureMessage;
};
struct Recording : Provider {
    Kind kind() const final {
        return Kind::Recording;
    }
    virtual RecordingState state() = 0;
    virtual void start(const std::string &camera, const std::string &file) = 0;
    virtual void stop(const std::string &camera) = 0;
    virtual void capture() = 0;
};
// Electrical board and sensor maintenance: power commands, IMU (VectorNav) mag cal and registers, FOG tare,
// the pinger and the inter-vehicle link (IVC). Every section is optional; `has*` says which are configured.
struct ElectricalCommandItem {
    std::string id, label;
    bool confirm = false; // cuts power somewhere: the panel asks first
};
struct ElectricalState {
    std::vector<ElectricalCommandItem> commands;
    std::string commandMessage;
    bool hasImu = false, magCalReady = false, magCalRunning = false;
    float magCalProgress = 0; // 0..1, from the shrinking deviation (as RViz)
    std::string magCalMessage;
    bool registerReady = false, registerPending = false;
    std::string registerValue, registerMessage; // value: the register fields of the last reply
    bool hasTare = false, tareReady = false, tareRunning = false;
    std::string tareMessage;
    bool hasPinger = false, pingerEnabled = true;
    std::vector<int> pingerFrequencies; // kHz
    std::optional<int> pingerSelected;  // kHz, as the board reports it
    std::optional<float> pingerAmplitude;
    bool hasIvc = false;
    std::vector<std::string> ivcHeaders, ivcStatuses;
    int ivcStatusHeaders = 0;        // headers below this index carry a status, the rest a raw 5-bit command
    std::vector<std::string> ivcLog; // newest last
};
struct Electrical : Provider {
    Kind kind() const final {
        return Kind::Electrical;
    }
    virtual ElectricalState state() = 0;
    virtual void command(const std::string &id) = 0;
    virtual void startMagCal() = 0;
    virtual void cancelMagCal() = 0;
    virtual void readRegister(const std::string &reg) = 0;
    virtual void writeRegister(const std::string &reg, const std::string &value) = 0;
    virtual void saveImuSettings() = 0;
    virtual void startTare(int samples, double timeoutSeconds) = 0;
    virtual void cancelTare() = 0;
    virtual void setPingerEnabled(bool) = 0;
    virtual void setPingerFrequency(int khz) = 0;
    virtual void sendIvc(int header, int command) = 0;
};
// Bag recording (ros2 bag record) on the machines of `targets`: this computer, or another (the robot) over ssh. A
// recording runs detached on its machine, so it outlives the viewer and the link; every viewer polling that machine
// sees it, whoever started it.
struct BagTargetState {
    std::string id, label, host, directory; // host: the ssh destination (setHost), empty for this computer
    bool known = false;                     // a reply has come back since the viewer started
    bool reachable = false;                 // the last command reached the machine
    bool recording = false, stopping = false, pending = false; // pending: a start or stop is under way
    std::string bag;                               // the running (or finishing) recording's path on that machine
    double elapsed = 0, bytes = 0, freeBytes = -1; // seconds, bytes; free -1: unknown
    std::string message;                           // the last outcome or problem
    bool failed = false;                           // `message` is a problem
    std::string lastBag, lastNote;                 // the most recent recording that ended there (whoever stopped it)
    double lastBytes = 0;
    bool lastComplete = true; // its metadata.yaml was written (closed cleanly)
};
struct BagPreset {
    std::string name;
    std::vector<std::string> topics; // absolute
};
struct BaggingState {
    std::vector<BagTargetState> targets;
    std::vector<std::pair<std::string, std::string>> topics; // the live graph: name, type; sorted by name
    std::vector<BagPreset> presets;
};
struct Bagging : Provider {
    Kind kind() const final {
        return Kind::Bagging;
    }
    virtual BaggingState state() = 0;
    // request.directory empty: the target's configured directory.
    virtual void start(const std::string &target, const BagRequest &request) = 0;
    virtual void stop(const std::string &target) = 0;
    virtual void kill(const std::string &target) = 0; // a recorder that does not finish after Stop
    // Points a remote target at another ssh destination (user@host); ignored for this computer, while it is
    // busy, or for an empty destination. The target then starts over with the new machine's state.
    virtual void setHost(const std::string &target, const std::string &host) = 0;
};
// SVO file for `camera`: `base` with a leading ~ replaced by `home`, then _<stamp> (when not empty) and
// _<camera> before the extension (.svo2 unless `base` ends in .svo or .svo2), so cameras never share a file.
std::string recordingFile(const std::string &base, const std::string &camera, const std::string &stamp,
                          const std::string &home);
} // namespace nereus::ros_viewer::panels
