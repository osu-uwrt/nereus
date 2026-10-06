// Numbers out of any ROS message without generated types: message type support loaded by name, the fields a type
// offers (for the Topics browser and search), and compiled field paths read as doubles. Paths are dotted member
// names with at most one [index] per segment ("pose.pose.position.z", "data[3]", "esc_telemetry[0].rpm"); below a
// geometry_msgs/Quaternion, "roll", "pitch" and "yaw" read its Euler angles in degrees.
#pragma once
#include <memory>
#include <optional>
#include <rclcpp/serialized_message.hpp>
#include <rcpputils/shared_library.hpp>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <string>
#include <vector>

namespace nereus::ros_viewer::plots {
namespace introspection = rosidl_typesupport_introspection_cpp;

// The C++ type support (for deserializing) and introspection members of one message type.
class MessageType {
  public:
    // "pkg/msg/Name" or "pkg/Name"; cached for the process. Throws std::runtime_error when it is not installed.
    static std::shared_ptr<const MessageType> get(const std::string &name);
    const std::string &name() const {
        return name_;
    }
    const introspection::MessageMembers *members() const {
        return members_;
    }
    const rosidl_message_type_support_t *typeSupport() const {
        return typeSupport_;
    }

  private:
    std::string name_;
    std::shared_ptr<rcpputils::SharedLibrary> cppLibrary_, introspectionLibrary_;
    const rosidl_message_type_support_t *typeSupport_ = nullptr;
    const introspection::MessageMembers *members_ = nullptr;
};

// Initialized message memory of one type, filled by deserialize().
class Message {
  public:
    explicit Message(std::shared_ptr<const MessageType>);
    ~Message();
    Message(const Message &) = delete;
    Message &operator=(const Message &) = delete;
    // False when the bytes do not decode as this type.
    bool deserialize(const rclcpp::SerializedMessage &);
    const void *data() const {
        return data_;
    }
    const MessageType &type() const {
        return *type_;
    }

  private:
    std::shared_ptr<const MessageType> type_;
    void *data_ = nullptr;
};

// One entry of a type's field tree, for browsing.
struct Field {
    std::string name, path, type; // segment ("position", "[3]", "yaw"), full path, short type ("Point", "float64[]")
    bool numeric = false;         // a plottable leaf
    bool expandable = false;      // has fields or elements below
    bool angle = false;           // a quaternion's derived roll / pitch / yaw
};

// The fields directly under `path` ("" for the root). Arrays list their elements: a fixed array its length, a
// sequence as many as `message` holds (none without one), at most `maxElements`. A quaternion lists roll, pitch
// and yaw and then its raw x, y, z, w.
std::vector<Field> fieldsAt(const MessageType &, const std::string &path, const void *message = nullptr,
                            std::size_t maxElements = 64);
// Every numeric leaf path of the type (arrays expanded as fieldsAt does; quaternions as angles only).
std::vector<std::string> numericPaths(const MessageType &, const void *message = nullptr, std::size_t maxElements = 16);

// A compiled field path. read() returns NaN when an index is beyond this message's sequence.
class FieldReader {
  public:
    // Throws std::invalid_argument on a path the type does not have, or one ending on a non-numeric field.
    FieldReader(const MessageType &, const std::string &path);
    double read(const void *message) const;

  private:
    struct Step {
        const introspection::MessageMember *member = nullptr;
        long index = -1;
        int angle = 0;                                             // 1 roll, 2 pitch, 3 yaw of the quaternion reached
        const introspection::MessageMembers *quaternion = nullptr; // its layout, for an angle step
    };
    std::vector<Step> steps_;
};

// The message's header stamp in seconds, when its first member is a std_msgs/Header.
std::optional<double> headerStamp(const MessageType &, const void *message);
// Whether a type is std_msgs/Header-stamped (headerStamp() applies).
bool hasHeader(const MessageType &);
// "pkg/msg/Name" without the package and "msg": the name shown beside a topic.
std::string shortType(const std::string &type);

} // namespace nereus::ros_viewer::plots
