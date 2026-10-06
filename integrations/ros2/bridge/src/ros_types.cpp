#include "ros_types.hpp"

#include <rclcpp/typesupport_helpers.hpp>

#include <cstring>
#include <map>
#include <mutex>
#include <new>

namespace nereus::ros_bridge {
namespace {
// "pkg/Name" -> "pkg/<kind>/Name"; other names pass through unchanged.
std::string threePart(const std::string &name, const char *kind) {
    std::vector<std::string> parts;
    std::size_t start = 0;
    while (true) {
        const auto slash = name.find('/', start);
        parts.push_back(name.substr(start, slash == std::string::npos ? slash : slash - start));
        if (slash == std::string::npos)
            break;
        start = slash + 1;
    }
    if (parts.size() == 3)
        return name;
    if (parts.size() == 2)
        return parts[0] + "/" + kind + "/" + parts[1];
    return name;
}

// Guards both type caches.
std::mutex cache_mutex;
} // namespace

// Loads (once per process) the C++ type support for (de)serialization and the introspection members.
std::shared_ptr<const MessageType> MessageType::get(const std::string &name) {
    static std::map<std::string, std::shared_ptr<const MessageType>> cache;
    const std::string full = threePart(name, "msg");
    std::lock_guard<std::mutex> lock(cache_mutex);
    if (const auto found = cache.find(full); found != cache.end())
        return found->second;
    auto type = std::make_shared<MessageType>();
    type->name_ = full;
    try {
        type->cpp_library_ = rclcpp::get_typesupport_library(full, "rosidl_typesupport_cpp");
        type->type_support_ = rclcpp::get_typesupport_handle(full, "rosidl_typesupport_cpp", *type->cpp_library_);
        type->introspection_library_ = rclcpp::get_typesupport_library(full, "rosidl_typesupport_introspection_cpp");
        const auto *handle =
            rclcpp::get_typesupport_handle(full, "rosidl_typesupport_introspection_cpp", *type->introspection_library_);
        type->members_ = static_cast<const introspection::MessageMembers *>(handle->data);
    } catch (const std::exception &error) {
        throw MappingError("ROS message type " + repr(name) + " is not installed (" + error.what() + ")");
    }
    cache.emplace(full, type);
    return type;
}

// The introspection service handle comes from its generated getter symbol, looked up by name.
std::shared_ptr<const ServiceType> ServiceType::get(const std::string &name) {
    static std::map<std::string, std::shared_ptr<const ServiceType>> cache;
    const std::string full = threePart(name, "srv");
    std::lock_guard<std::mutex> lock(cache_mutex);
    if (const auto found = cache.find(full); found != cache.end())
        return found->second;
    auto type = std::make_shared<ServiceType>();
    type->name = full;
    try {
        type->library = rclcpp::get_typesupport_library(full, "rosidl_typesupport_introspection_cpp");
        std::string symbol = "rosidl_typesupport_introspection_cpp__get_service_type_support_handle__";
        for (const char c : full)
            symbol += c == '/' ? std::string("__") : std::string(1, c);
        using Getter = const rosidl_service_type_support_t *(*)();
        const auto getter = reinterpret_cast<Getter>(type->library->get_symbol(symbol));
        const auto *handle = getter();
        const auto *members = static_cast<const introspection::ServiceMembers *>(handle->data);
        type->request = members->request_members_;
        type->response = members->response_members_;
    } catch (const std::exception &error) {
        throw MappingError("ROS service type " + repr(name) + " is not installed (" + error.what() + ")");
    }
    cache.emplace(full, type);
    return type;
}

// Allocates and default-initializes the message (16-byte aligned).
Message::Message(std::shared_ptr<const MessageType> type) : type_(std::move(type)) {
    const auto *members = type_->members();
    data_ = ::operator new(members->size_of_, std::align_val_t{16});
    members->init_function(data_, rosidl_runtime_cpp::MessageInitialization::ALL);
}

Message::~Message() {
    if (data_) {
        type_->members()->fini_function(data_);
        ::operator delete(data_, std::align_val_t{16});
    }
}

std::string baseName(const introspection::MessageMembers *members) {
    std::string space = members->message_namespace_;
    const auto colon = space.find("::");
    if (colon != std::string::npos)
        space = space.substr(0, colon);
    return space + "/" + members->message_name_;
}

namespace {
// Type name of a primitive introspection id, as RosType::base uses it.
const char *primitiveName(std::uint8_t id) {
    using namespace introspection;
    if (id == ROS_TYPE_FLOAT)
        return "float";
    if (id == ROS_TYPE_DOUBLE)
        return "double";
    if (id == ROS_TYPE_LONG_DOUBLE)
        return "long double";
    if (id == ROS_TYPE_CHAR)
        return "char";
    if (id == ROS_TYPE_WCHAR)
        return "wchar";
    if (id == ROS_TYPE_BOOLEAN)
        return "boolean";
    if (id == ROS_TYPE_OCTET)
        return "octet";
    if (id == ROS_TYPE_UINT8)
        return "uint8";
    if (id == ROS_TYPE_INT8)
        return "int8";
    if (id == ROS_TYPE_UINT16)
        return "uint16";
    if (id == ROS_TYPE_INT16)
        return "int16";
    if (id == ROS_TYPE_UINT32)
        return "uint32";
    if (id == ROS_TYPE_INT32)
        return "int32";
    if (id == ROS_TYPE_UINT64)
        return "uint64";
    if (id == ROS_TYPE_INT64)
        return "int64";
    if (id == ROS_TYPE_STRING)
        return "string";
    if (id == ROS_TYPE_WSTRING)
        return "wstring";
    return "unknown";
}

// Introspection members of a nested message field.
const introspection::MessageMembers *nested(const introspection::MessageMember &member) {
    return static_cast<const introspection::MessageMembers *>(member.members_->data);
}
} // namespace

bool isNumericId(std::uint8_t id) {
    using namespace introspection;
    return id == ROS_TYPE_FLOAT || id == ROS_TYPE_DOUBLE || id == ROS_TYPE_CHAR || id == ROS_TYPE_OCTET ||
           id == ROS_TYPE_UINT8 || id == ROS_TYPE_INT8 || id == ROS_TYPE_UINT16 || id == ROS_TYPE_INT16 ||
           id == ROS_TYPE_UINT32 || id == ROS_TYPE_INT32 || id == ROS_TYPE_UINT64 || id == ROS_TYPE_INT64;
}

// Fixed arrays have array_size_ > 0 and no upper bound; everything else is a sequence.
RosType memberType(const introspection::MessageMember &member) {
    RosType type;
    type.base =
        member.type_id_ == introspection::ROS_TYPE_MESSAGE ? baseName(nested(member)) : primitiveName(member.type_id_);
    if (member.is_array_) {
        if (member.array_size_ == 0 || member.is_upper_bound_)
            type.sequence = true;
        else
            type.length = static_cast<int>(member.array_size_);
    }
    return type;
}

FieldPath resolveField(const introspection::MessageMembers *owner, const std::string &path, bool writable) {
    FieldPath result;
    result.path = path;
    std::optional<RosType> current;
    for (const auto &token : parsePath(path)) {
        if (current) {
            if (!current->isMessage() || current->isArray())
                throw MappingError(repr(path) + ": " + repr(token.name) + " is below a non-message field");
            // `owner` was set from the previous hop below.
        }

        // Find the member by name in the current message.
        const introspection::MessageMember *found = nullptr;
        for (std::uint32_t k = 0; k < owner->member_count_; ++k)
            if (token.name == owner->members_[k].name_)
                found = &owner->members_[k];
        if (!found)
            throw MappingError(repr(path) + ": " + owner->message_name_ + " has no field " + repr(token.name));
        RosType type = memberType(*found);

        Hop hop{found, -1};
        // At most one index per token, only on arrays; sequences cannot be written element-wise.
        if (token.indexes.size() > 1)
            throw MappingError(repr(path) + ": " + repr(token.name) + " is not an array");
        for (const int index : token.indexes) {
            if (!type.isArray())
                throw MappingError(repr(path) + ": " + repr(token.name) + " is not an array");
            if (writable && type.sequence)
                throw MappingError(repr(path) + ": cannot assign an element of sequence " + repr(token.name) +
                                   "; assign the whole sequence");
            if (type.length >= 0 && index >= type.length)
                throw MappingError(repr(path) + ": index " + std::to_string(index) + " exceeds " + token.name + "[" +
                                   std::to_string(type.length) + "]");
            hop.index = index;
            type = type.element();
        }

        // Descend into nested messages for the next token.
        result.hops.push_back(hop);
        current = type;
        if (found->type_id_ == introspection::ROS_TYPE_MESSAGE)
            owner = nested(*found);
    }
    result.type = *current;
    return result;
}

// Like locate(), but checks sequence indexes against the received size.
const void *locateConst(const void *message, const FieldPath &path) {
    const char *base = static_cast<const char *>(message);
    const void *address = base;
    for (const auto &hop : path.hops) {
        address = static_cast<const char *>(address) + hop.member->offset_;
        if (hop.index >= 0) {
            if (static_cast<std::size_t>(hop.index) >= hop.member->size_function(address))
                throw MappingError(repr(path.path) + ": message sequence is shorter than the mapped index");
            address = hop.member->get_const_function(address, static_cast<std::size_t>(hop.index));
        }
    }
    return address;
}

// Offsets through each hop; indexes are not bounds-checked (writable paths only index fixed arrays).
void *locate(void *message, const FieldPath &path) {
    void *address = message;
    for (const auto &hop : path.hops) {
        address = static_cast<char *>(address) + hop.member->offset_;
        if (hop.index >= 0)
            address = hop.member->get_function(address, static_cast<std::size_t>(hop.index));
    }
    return address;
}

namespace {
// One primitive field as JSON (null for unsupported types).
Json scalarJson(std::uint8_t id, const void *p) {
    using namespace introspection;
    if (id == ROS_TYPE_FLOAT)
        return *static_cast<const float *>(p);
    if (id == ROS_TYPE_DOUBLE)
        return *static_cast<const double *>(p);
    if (id == ROS_TYPE_BOOLEAN)
        return *static_cast<const bool *>(p);
    if (id == ROS_TYPE_CHAR || id == ROS_TYPE_OCTET || id == ROS_TYPE_UINT8)
        return *static_cast<const std::uint8_t *>(p);
    if (id == ROS_TYPE_INT8)
        return *static_cast<const std::int8_t *>(p);
    if (id == ROS_TYPE_UINT16)
        return *static_cast<const std::uint16_t *>(p);
    if (id == ROS_TYPE_INT16)
        return *static_cast<const std::int16_t *>(p);
    if (id == ROS_TYPE_UINT32)
        return *static_cast<const std::uint32_t *>(p);
    if (id == ROS_TYPE_INT32)
        return *static_cast<const std::int32_t *>(p);
    if (id == ROS_TYPE_UINT64)
        return *static_cast<const std::uint64_t *>(p);
    if (id == ROS_TYPE_INT64)
        return *static_cast<const std::int64_t *>(p);
    if (id == ROS_TYPE_STRING)
        return *static_cast<const std::string *>(p);
    return nullptr;
}
} // namespace

Json messageToJson(const introspection::MessageMembers *members, const void *message) {
    Json out = Json::object();
    for (std::uint32_t k = 0; k < members->member_count_; ++k) {
        const auto &member = members->members_[k];
        const void *field = static_cast<const char *>(message) + member.offset_;
        const auto element = [&](const void *p) {
            return member.type_id_ == introspection::ROS_TYPE_MESSAGE ? messageToJson(nested(member), p)
                                                                      : scalarJson(member.type_id_, p);
        };
        if (!member.is_array_) {
            out[member.name_] = element(field);
            // Unbounded bool sequences are std::vector<bool>, which has no per-element address.
        } else if (member.type_id_ == introspection::ROS_TYPE_BOOLEAN && member.array_size_ == 0) {
            const auto &bits = *static_cast<const std::vector<bool> *>(field);
            Json list = Json::array();
            for (const bool bit : bits)
                list.push_back(bit);
            out[member.name_] = list;
        } else {
            Json list = Json::array();
            for (std::size_t i = 0; i < member.size_function(field); ++i)
                list.push_back(element(member.get_const_function(field, i)));
            out[member.name_] = list;
        }
    }
    return out;
}
} // namespace nereus::ros_bridge
