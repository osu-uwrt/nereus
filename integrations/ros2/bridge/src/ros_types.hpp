#pragma once
// Runtime ROS interface access without generated C++ types: message/service type support is
// loaded by name and every field is reached through rosidl introspection. No middleware is
// created here.
#include "native.hpp"

#include <nlohmann/json.hpp>
#include <rcpputils/shared_library.hpp>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>

#include <memory>
#include <string>
#include <vector>

namespace nereus::ros_bridge {
namespace introspection = rosidl_typesupport_introspection_cpp;
using Json = nlohmann::json;

// Field type as parse_type describes it: primitive name or 'pkg/Message', array kind.
struct RosType {
    std::string base;
    int length{-1};       // fixed array length, -1 when not a fixed array
    bool sequence{false}; // unbounded or bounded sequence
    bool isArray() const {
        return length >= 0 || sequence;
    }
    bool isMessage() const {
        return base.find('/') != std::string::npos;
    }
    RosType element() const {
        return RosType{base, -1, false};
    }
};

class MessageType {
  public:
    // "pkg/msg/Name" or "pkg/Name"; cached for the process lifetime. Throws MappingError.
    static std::shared_ptr<const MessageType> get(const std::string &name);
    const std::string &name() const {
        return name_;
    }
    const introspection::MessageMembers *members() const {
        return members_;
    }
    const rosidl_message_type_support_t *typeSupport() const {
        return type_support_;
    }

  private:
    std::string name_;
    std::shared_ptr<rcpputils::SharedLibrary> cpp_library_, introspection_library_;
    const rosidl_message_type_support_t *type_support_{nullptr};
    const introspection::MessageMembers *members_{nullptr};
};

struct ServiceType {
    std::string name;
    std::shared_ptr<rcpputils::SharedLibrary> library;
    const introspection::MessageMembers *request{nullptr}, *response{nullptr};
    // "pkg/srv/Name". Throws MappingError when not installed.
    static std::shared_ptr<const ServiceType> get(const std::string &name);
};

// Initialised message memory owned by this object.
class Message {
  public:
    explicit Message(std::shared_ptr<const MessageType> type);
    ~Message();
    Message(const Message &) = delete;
    Message &operator=(const Message &) = delete;
    void *data() {
        return data_;
    }
    const void *data() const {
        return data_;
    }
    const MessageType &type() const {
        return *type_;
    }
    const std::shared_ptr<const MessageType> &typePtr() const {
        return type_;
    }

  private:
    std::shared_ptr<const MessageType> type_;
    void *data_{nullptr};
};

std::string baseName(const introspection::MessageMembers *members); // "pkg/Name"
RosType memberType(const introspection::MessageMember &member);

// One resolved field path: hops through nested messages, then the addressed field.
struct Hop {
    const introspection::MessageMember *member{nullptr};
    int index{-1}; // element of an array of messages, or -1
};
struct FieldPath {
    std::vector<Hop> hops; // hops.back() is the addressed field (index applies to it)
    RosType type;          // type of the addressed value (after the index)
    const introspection::MessageMember &member() const {
        return *hops.back().member;
    }
    bool indexed() const {
        return hops.back().index >= 0;
    }
    std::string path;
};
// mapping.ros_field: throws MappingError with the same messages.
FieldPath resolveField(const introspection::MessageMembers *owner, const std::string &path, bool writable = false);
// Address of the addressed field (or indexed element) inside `message`. When `readonly` a
// sequence index beyond the size throws MappingError.
void *locate(void *message, const FieldPath &path);
const void *locateConst(const void *message, const FieldPath &path);

// Generic dump for tests and diagnostics; arrays become JSON arrays.
Json messageToJson(const introspection::MessageMembers *members, const void *message);

// Primitive access by introspection type id.
bool isNumericId(std::uint8_t id);
} // namespace nereus::ros_bridge
