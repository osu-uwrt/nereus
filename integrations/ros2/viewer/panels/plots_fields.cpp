// Field access by rosidl introspection: type support by name, field trees for browsing, compiled numeric paths.
#include "plots_fields.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <mutex>
#include <new>
#include <rclcpp/serialization.hpp>
#include <rclcpp/typesupport_helpers.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <stdexcept>

namespace nereus::ros_viewer::plots {
namespace {

// "pkg/Name" -> "pkg/msg/Name"; three-part names pass through.
std::string threePart(const std::string &name) {
    const auto first = name.find('/');
    if (first == std::string::npos)
        return name;
    if (name.find('/', first + 1) != std::string::npos)
        return name;
    return name.substr(0, first) + "/msg/" + name.substr(first + 1);
}

const introspection::MessageMembers *nested(const introspection::MessageMember &member) {
    return static_cast<const introspection::MessageMembers *>(member.members_->data);
}

bool isMessage(const introspection::MessageMember &m) {
    return m.type_id_ == introspection::ROS_TYPE_MESSAGE;
}

bool isNumericId(std::uint8_t id) {
    using namespace introspection;
    return id == ROS_TYPE_FLOAT || id == ROS_TYPE_DOUBLE || id == ROS_TYPE_LONG_DOUBLE || id == ROS_TYPE_CHAR ||
           id == ROS_TYPE_BOOLEAN || id == ROS_TYPE_OCTET || id == ROS_TYPE_UINT8 || id == ROS_TYPE_INT8 ||
           id == ROS_TYPE_UINT16 || id == ROS_TYPE_INT16 || id == ROS_TYPE_UINT32 || id == ROS_TYPE_INT32 ||
           id == ROS_TYPE_UINT64 || id == ROS_TYPE_INT64;
}

const char *primitiveName(std::uint8_t id) {
    using namespace introspection;
    switch (id) {
    case ROS_TYPE_FLOAT:
        return "float32";
    case ROS_TYPE_DOUBLE:
        return "float64";
    case ROS_TYPE_LONG_DOUBLE:
        return "float128";
    case ROS_TYPE_CHAR:
        return "char";
    case ROS_TYPE_BOOLEAN:
        return "bool";
    case ROS_TYPE_OCTET:
        return "byte";
    case ROS_TYPE_UINT8:
        return "uint8";
    case ROS_TYPE_INT8:
        return "int8";
    case ROS_TYPE_UINT16:
        return "uint16";
    case ROS_TYPE_INT16:
        return "int16";
    case ROS_TYPE_UINT32:
        return "uint32";
    case ROS_TYPE_INT32:
        return "int32";
    case ROS_TYPE_UINT64:
        return "uint64";
    case ROS_TYPE_INT64:
        return "int64";
    case ROS_TYPE_STRING:
        return "string";
    case ROS_TYPE_WSTRING:
        return "wstring";
    default:
        return "?";
    }
}

double readPrimitive(std::uint8_t id, const void *p) {
    using namespace introspection;
    switch (id) {
    case ROS_TYPE_FLOAT:
        return *static_cast<const float *>(p);
    case ROS_TYPE_DOUBLE:
        return *static_cast<const double *>(p);
    case ROS_TYPE_LONG_DOUBLE:
        return double(*static_cast<const long double *>(p));
    case ROS_TYPE_CHAR:
        return *static_cast<const signed char *>(p);
    case ROS_TYPE_BOOLEAN:
        return *static_cast<const bool *>(p) ? 1 : 0;
    case ROS_TYPE_OCTET:
    case ROS_TYPE_UINT8:
        return *static_cast<const std::uint8_t *>(p);
    case ROS_TYPE_INT8:
        return *static_cast<const std::int8_t *>(p);
    case ROS_TYPE_UINT16:
        return *static_cast<const std::uint16_t *>(p);
    case ROS_TYPE_INT16:
        return *static_cast<const std::int16_t *>(p);
    case ROS_TYPE_UINT32:
        return *static_cast<const std::uint32_t *>(p);
    case ROS_TYPE_INT32:
        return *static_cast<const std::int32_t *>(p);
    case ROS_TYPE_UINT64:
        return double(*static_cast<const std::uint64_t *>(p));
    case ROS_TYPE_INT64:
        return double(*static_cast<const std::int64_t *>(p));
    default:
        return std::numeric_limits<double>::quiet_NaN();
    }
}

// Element `index` of a primitive array, copied out (works for std::vector<bool> too).
double readElement(const introspection::MessageMember &m, const void *array, std::size_t index) {
    alignas(16) unsigned char value[16] = {};
    if (m.fetch_function)
        m.fetch_function(array, index, value);
    else
        std::memcpy(value, m.get_const_function(array, index), 8);
    return readPrimitive(m.type_id_, value);
}

bool fixedArray(const introspection::MessageMember &m) {
    return m.is_array_ && m.array_size_ > 0 && !m.is_upper_bound_;
}

std::size_t arraySize(const introspection::MessageMember &m, const void *array) {
    if (fixedArray(m))
        return m.array_size_;
    return array && m.size_function ? m.size_function(array) : 0;
}

std::string typeName(const introspection::MessageMembers *members) {
    return members->message_name_;
}

bool isQuaternion(const introspection::MessageMembers *members) {
    return std::strncmp(members->message_namespace_, "geometry_msgs", 13) == 0 &&
           std::strcmp(members->message_name_, "Quaternion") == 0;
}

const introspection::MessageMember *memberNamed(const introspection::MessageMembers *owner, const std::string &name) {
    for (std::uint32_t k = 0; k < owner->member_count_; ++k)
        if (name == owner->members_[k].name_)
            return &owner->members_[k];
    return nullptr;
}

// Roll / pitch / yaw (1, 2, 3) in degrees of the geometry_msgs/Quaternion at `q`.
double quaternionAngle(const introspection::MessageMembers *members, const void *q, int angle) {
    double c[4] = {0, 0, 0, 1};
    const char *names[] = {"x", "y", "z", "w"};
    for (int i = 0; i < 4; ++i)
        if (const auto *m = memberNamed(members, names[i]))
            c[i] = readPrimitive(m->type_id_, static_cast<const char *>(q) + m->offset_);
    const double x = c[0], y = c[1], z = c[2], w = c[3];
    const double norm = std::sqrt(x * x + y * y + z * z + w * w);
    if (!(norm > 0))
        return std::numeric_limits<double>::quiet_NaN();
    double r = 0;
    if (angle == 1)
        r = std::atan2(2 * (w * x + y * z), norm * norm - 2 * (x * x + y * y));
    else if (angle == 2)
        r = std::asin(std::clamp(2 * (w * y - z * x) / (norm * norm), -1.0, 1.0));
    else
        r = std::atan2(2 * (w * z + x * y), norm * norm - 2 * (y * y + z * z));
    return r * 180 / M_PI;
}

int angleOf(const std::string &name) {
    return name == "roll" ? 1 : name == "pitch" ? 2 : name == "yaw" ? 3 : 0;
}

// A path segment: member name and an optional index ("data[3]" -> data, 3).
struct Segment {
    std::string name;
    long index = -1;
};

std::vector<Segment> parse(const std::string &path) {
    std::vector<Segment> out;
    std::size_t start = 0;
    while (start <= path.size() && !path.empty()) {
        const auto dot = path.find('.', start);
        std::string token = path.substr(start, dot == std::string::npos ? std::string::npos : dot - start);
        Segment s;
        if (const auto open = token.find('['); open != std::string::npos) {
            const auto close = token.find(']', open);
            if (close == std::string::npos || close + 1 != token.size())
                throw std::invalid_argument("bad index in '" + path + "'");
            const std::string digits = token.substr(open + 1, close - open - 1);
            if (digits.empty() || digits.find_first_not_of("0123456789") != std::string::npos)
                throw std::invalid_argument("bad index in '" + path + "'");
            s.index = std::stol(digits);
            token = token.substr(0, open);
        }
        if (token.empty())
            throw std::invalid_argument("empty field name in '" + path + "'");
        s.name = token;
        out.push_back(s);
        if (dot == std::string::npos)
            break;
        start = dot + 1;
    }
    return out;
}

std::mutex cacheMutex;

// Where a browse path lands: a message's members, an array's elements, or (members only) a quaternion.
struct Level {
    const introspection::MessageMembers *members = nullptr; // listing a message's fields
    const introspection::MessageMember *array = nullptr;    // listing an array's elements
    const void *data = nullptr;                             // the message / array storage, when known
};

Level resolveLevel(const MessageType &type, const std::string &path, const void *message) {
    Level level{type.members(), nullptr, message};
    if (path.empty())
        return level;
    for (const auto &segment : parse(path)) {
        if (!level.members)
            throw std::invalid_argument("'" + path + "' goes below a leaf");
        const auto *m = memberNamed(level.members, segment.name);
        if (!m)
            throw std::invalid_argument(std::string(level.members->message_name_) + " has no field '" + segment.name +
                                        "'");
        const void *storage = level.data ? static_cast<const char *>(level.data) + m->offset_ : nullptr;
        if (m->is_array_ && segment.index < 0) {
            level = Level{nullptr, m, storage};
            continue;
        }
        if (m->is_array_) {
            if (std::size_t(segment.index) >= std::max(arraySize(*m, storage), fixedArray(*m) ? m->array_size_ : 0))
                storage = nullptr;
            else if (storage && isMessage(*m))
                storage = m->get_const_function(storage, std::size_t(segment.index));
        }
        if (!isMessage(*m))
            throw std::invalid_argument("'" + path + "' ends on a leaf");
        level = Level{nested(*m), nullptr, storage};
    }
    return level;
}

std::string memberType(const introspection::MessageMember &m) {
    std::string base = isMessage(m) ? typeName(nested(m)) : primitiveName(m.type_id_);
    if (m.is_array_)
        base += fixedArray(m) ? "[" + std::to_string(m.array_size_) + "]" : "[]";
    return base;
}

} // namespace

std::shared_ptr<const MessageType> MessageType::get(const std::string &name) {
    static std::map<std::string, std::shared_ptr<const MessageType>> cache;
    const std::string full = threePart(name);
    std::lock_guard<std::mutex> lock(cacheMutex);
    if (const auto found = cache.find(full); found != cache.end())
        return found->second;
    auto type = std::make_shared<MessageType>();
    type->name_ = full;
    try {
        type->cppLibrary_ = rclcpp::get_typesupport_library(full, "rosidl_typesupport_cpp");
        type->typeSupport_ = rclcpp::get_typesupport_handle(full, "rosidl_typesupport_cpp", *type->cppLibrary_);
        type->introspectionLibrary_ = rclcpp::get_typesupport_library(full, "rosidl_typesupport_introspection_cpp");
        const auto *handle =
            rclcpp::get_typesupport_handle(full, "rosidl_typesupport_introspection_cpp", *type->introspectionLibrary_);
        type->members_ = static_cast<const introspection::MessageMembers *>(handle->data);
    } catch (const std::exception &error) {
        throw std::runtime_error(full + " isn't installed on this computer (" + error.what() + ")");
    }
    cache.emplace(full, type);
    return type;
}

Message::Message(std::shared_ptr<const MessageType> type) : type_(std::move(type)) {
    const auto *members = type_->members();
    data_ = ::operator new(members->size_of_, std::align_val_t{16});
    members->init_function(data_, rosidl_runtime_cpp::MessageInitialization::ALL);
}

Message::~Message() {
    type_->members()->fini_function(data_);
    ::operator delete(data_, std::align_val_t{16});
}

bool Message::deserialize(const rclcpp::SerializedMessage &bytes) {
    try {
        rclcpp::SerializationBase serializer(type_->typeSupport());
        serializer.deserialize_message(&bytes, data_);
        return true;
    } catch (const std::exception &) {
        return false;
    }
}

std::vector<Field> fieldsAt(const MessageType &type, const std::string &path, const void *message,
                            std::size_t maxElements) {
    std::vector<Field> out;
    const Level level = resolveLevel(type, path, message);
    const std::string prefix = path.empty() ? "" : path + ".";
    if (level.array) {
        const auto &m = *level.array;
        const std::size_t n = std::min(arraySize(m, level.data), maxElements);
        const std::string element = isMessage(m) ? typeName(nested(m)) : primitiveName(m.type_id_);
        for (std::size_t i = 0; i < n; ++i) {
            Field f;
            f.name = "[" + std::to_string(i) + "]";
            f.path = path + f.name;
            f.type = element;
            f.numeric = !isMessage(m) && isNumericId(m.type_id_);
            f.expandable = isMessage(m);
            out.push_back(f);
        }
        return out;
    }
    if (isQuaternion(level.members))
        for (const char *angle : {"roll", "pitch", "yaw"}) {
            Field f;
            f.name = angle;
            f.path = prefix + angle;
            f.type = "°";
            f.numeric = f.angle = true;
            out.push_back(f);
        }
    for (std::uint32_t k = 0; k < level.members->member_count_; ++k) {
        const auto &m = level.members->members_[k];
        Field f;
        f.name = m.name_;
        f.path = prefix + m.name_;
        f.type = memberType(m);
        f.numeric = !m.is_array_ && !isMessage(m) && isNumericId(m.type_id_);
        f.expandable = isMessage(m) || m.is_array_;
        out.push_back(f);
    }
    return out;
}

std::vector<std::string> numericPaths(const MessageType &type, const void *message, std::size_t maxElements) {
    std::vector<std::string> out;
    std::vector<std::string> pending{""};
    while (!pending.empty() && out.size() < 4096) {
        const std::string path = pending.back();
        pending.pop_back();
        std::vector<Field> fields;
        try {
            fields = fieldsAt(type, path, message, maxElements);
        } catch (const std::exception &) {
            continue;
        }
        const bool quaternion = !fields.empty() && fields.front().angle;
        for (auto it = fields.rbegin(); it != fields.rend(); ++it) {
            if (quaternion && !it->angle)
                continue; // a quaternion's raw parts stay out of the index (the browser shows them)
            if (it->numeric)
                out.push_back(it->path);
            else if (it->expandable)
                pending.push_back(it->path);
        }
    }
    return out;
}

FieldReader::FieldReader(const MessageType &type, const std::string &path) {
    const auto *owner = type.members();
    const auto segments = parse(path);
    if (segments.empty())
        throw std::invalid_argument("empty field path");
    for (std::size_t i = 0; i < segments.size(); ++i) {
        const auto &segment = segments[i];
        const bool last = i + 1 == segments.size();
        if (!owner)
            throw std::invalid_argument("'" + path + "' goes below a number");
        if (const int angle = angleOf(segment.name); angle && isQuaternion(owner) && segment.index < 0) {
            if (!last)
                throw std::invalid_argument("'" + path + "' goes below an angle");
            steps_.push_back({nullptr, -1, angle, owner});
            break;
        }
        const auto *m = memberNamed(owner, segment.name);
        if (!m)
            throw std::invalid_argument(std::string(owner->message_name_) + " has no field '" + segment.name + "'");
        if (m->is_array_ && segment.index < 0)
            throw std::invalid_argument("'" + segment.name + "' is an array: pick an element, e.g. " + segment.name +
                                        "[0]");
        if (!m->is_array_ && segment.index >= 0)
            throw std::invalid_argument("'" + segment.name + "' is not an array");
        if (fixedArray(*m) && std::size_t(segment.index) >= m->array_size_)
            throw std::invalid_argument("'" + segment.name + "' has " + std::to_string(m->array_size_) + " elements");
        steps_.push_back({m, segment.index, 0, nullptr});
        if (isMessage(*m)) {
            if (last) {
                if (!isQuaternion(nested(*m)))
                    throw std::invalid_argument("'" + path + "' is a " + typeName(nested(*m)) + ", not a number");
                throw std::invalid_argument("'" + path + "' is a quaternion: plot its roll, pitch or yaw");
            }
            owner = nested(*m);
        } else {
            if (!last)
                throw std::invalid_argument("'" + path + "' goes below a number");
            if (!isNumericId(m->type_id_))
                throw std::invalid_argument("'" + path + "' is a " + primitiveName(m->type_id_) + ", not a number");
            owner = nullptr;
        }
    }
}

double FieldReader::read(const void *message) const {
    const void *at = message;
    for (const auto &step : steps_) {
        if (step.angle)
            return quaternionAngle(step.quaternion, at, step.angle);
        const auto &m = *step.member;
        const void *storage = static_cast<const char *>(at) + m.offset_;
        if (m.is_array_) {
            if (std::size_t(step.index) >= arraySize(m, storage))
                return std::numeric_limits<double>::quiet_NaN();
            if (!isMessage(m))
                return readElement(m, storage, std::size_t(step.index));
            at = m.get_const_function(storage, std::size_t(step.index));
        } else {
            if (!isMessage(m))
                return readPrimitive(m.type_id_, storage);
            at = storage;
        }
    }
    return std::numeric_limits<double>::quiet_NaN();
}

bool hasHeader(const MessageType &type) {
    const auto *members = type.members();
    if (members->member_count_ == 0)
        return false;
    const auto &first = members->members_[0];
    return std::strcmp(first.name_, "header") == 0 && isMessage(first) && !first.is_array_ &&
           std::strcmp(nested(first)->message_name_, "Header") == 0;
}

std::optional<double> headerStamp(const MessageType &type, const void *message) {
    if (!hasHeader(type))
        return std::nullopt;
    const auto &header = type.members()->members_[0];
    const auto *headerMembers = nested(header);
    const auto *stamp = memberNamed(headerMembers, "stamp");
    if (!stamp || !isMessage(*stamp))
        return std::nullopt;
    const char *at = static_cast<const char *>(message) + header.offset_ + stamp->offset_;
    const auto *time = nested(*stamp);
    const auto *sec = memberNamed(time, "sec"), *nanosec = memberNamed(time, "nanosec");
    if (!sec || !nanosec)
        return std::nullopt;
    return readPrimitive(sec->type_id_, at + sec->offset_) +
           readPrimitive(nanosec->type_id_, at + nanosec->offset_) * 1e-9;
}

std::string shortType(const std::string &type) {
    const auto slash = type.rfind('/');
    return slash == std::string::npos ? type : type.substr(slash + 1);
}

} // namespace nereus::ros_viewer::plots
