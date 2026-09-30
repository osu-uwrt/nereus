#include "mapping.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <set>
#include <sstream>

namespace robotics::ros_bridge {
namespace {
constexpr const char *kQuaternion = "geometry_msgs/Quaternion";
constexpr const char *kTime = "builtin_interfaces/Time";

bool isVectorMessage(const RosType &t) {
    return !t.isArray() && (t.base == "geometry_msgs/Vector3" || t.base == "geometry_msgs/Point");
}
bool isFloatBase(const std::string &b) {
    return b == "float" || b == "double";
}
bool intRange(const std::string &base, std::int64_t &low, std::uint64_t &high) {
    static const std::map<std::string, std::pair<std::int64_t, std::uint64_t>> table = {
        {"int8", {-128, 127}},       {"uint8", {0, 255}},
        {"byte", {0, 255}},          {"octet", {0, 255}},
        {"char", {0, 255}},          {"int16", {-32768, 32767}},
        {"uint16", {0, 65535}},      {"int32", {INT32_MIN, INT32_MAX}},
        {"uint32", {0, UINT32_MAX}}, {"int64", {INT64_MIN, INT64_MAX}},
        {"uint64", {0, UINT64_MAX}}};
    const auto found = table.find(base);
    if (found == table.end())
        return false;
    low = found->second.first;
    high = found->second.second;
    return true;
}
bool isIntBase(const std::string &b) {
    std::int64_t l;
    std::uint64_t h;
    return intRange(b, l, h);
}
void checkRange(const std::string &base, std::int64_t value, const std::string &where) {
    std::int64_t low;
    std::uint64_t high;
    intRange(base, low, high);
    const bool ok = value >= low && (value < 0 || static_cast<std::uint64_t>(value) <= high);
    if (!ok)
        throw MappingError(where + ": " + std::to_string(value) + " out of range for " + base);
}

// ------------------------------------------------------------------ native -> ROS

using Store = std::function<void(void *element, const Value &value)>;

Store scalarStore(const introspection::MessageMember &member, const RosType &type, const Spec &spec,
                  const std::string &where) {
    using namespace introspection;
    const auto id = member.type_id_;
    const auto &base = type.base;
    if (isFloatBase(base) && (spec.dtype == Dtype::Float || spec.dtype == Dtype::Int)) {
        if (id == ROS_TYPE_FLOAT)
            return [](void *p, const Value &v) { *static_cast<float *>(p) = static_cast<float>(v.asDouble()); };
        return [](void *p, const Value &v) { *static_cast<double *>(p) = v.asDouble(); };
    }
    if (isIntBase(base) && spec.dtype == Dtype::Int) {
        return [id, base, where](void *p, const Value &v) {
            const std::int64_t x = v.asInt();
            checkRange(base, x, where);
            if (id == ROS_TYPE_UINT8 || id == ROS_TYPE_OCTET || id == ROS_TYPE_CHAR)
                *static_cast<std::uint8_t *>(p) = static_cast<std::uint8_t>(x);
            else if (id == ROS_TYPE_INT8)
                *static_cast<std::int8_t *>(p) = static_cast<std::int8_t>(x);
            else if (id == ROS_TYPE_UINT16)
                *static_cast<std::uint16_t *>(p) = static_cast<std::uint16_t>(x);
            else if (id == ROS_TYPE_INT16)
                *static_cast<std::int16_t *>(p) = static_cast<std::int16_t>(x);
            else if (id == ROS_TYPE_UINT32)
                *static_cast<std::uint32_t *>(p) = static_cast<std::uint32_t>(x);
            else if (id == ROS_TYPE_INT32)
                *static_cast<std::int32_t *>(p) = static_cast<std::int32_t>(x);
            else if (id == ROS_TYPE_UINT64)
                *static_cast<std::uint64_t *>(p) = static_cast<std::uint64_t>(x);
            else
                *static_cast<std::int64_t *>(p) = x;
        };
    }
    if (base == "boolean" && spec.dtype == Dtype::Bool)
        return [](void *p, const Value &v) { *static_cast<bool *>(p) = v.b; };
    if (base == "string" && spec.dtype == Dtype::String)
        return [](void *p, const Value &v) { *static_cast<std::string *>(p) = v.s; };
    throw MappingError(where + ": cannot assign native " + spec.describe() + " to ROS " + base);
}

// Setter of a whole field value at the address of the field (or indexed element).
using Setter = std::function<void(void *target, const Value &value)>;

Setter makeSetter(const FieldPath &ros, const Spec &spec, const std::string &where) {
    const RosType &type = ros.type;
    if (type.base == kQuaternion && !type.isArray())
        throw MappingError(where + ": quaternions are never assigned whole; map w/x/y/z components explicitly");
    if (type.base == kTime && !type.isArray()) {
        if (spec != timeSpec())
            throw MappingError(where + ": " + std::string(kTime) + " needs a native time source");
        return [](void *target, const Value &v) {
            const std::int64_t ns = v.asInt();
            std::int64_t sec = ns / 1000000000, rest = ns % 1000000000;
            if (rest < 0) {
                rest += 1000000000;
                --sec;
            }
            auto *time = static_cast<builtin_interfaces::msg::Time *>(target);
            time->sec = static_cast<std::int32_t>(sec);
            time->nanosec = static_cast<std::uint32_t>(rest);
        };
    }
    if (isVectorMessage(type)) {
        if (spec != vector3Spec())
            throw MappingError(where + ": " + type.base + " needs a native float[3] vector");
        return [](void *target, const Value &v) {
            if (v.a.size() != 3)
                throw MappingError("vector value must have 3 components");
            auto *vec = static_cast<geometry_msgs::msg::Vector3 *>(target); // Point has the same layout
            vec->x = v.a[0];
            vec->y = v.a[1];
            vec->z = v.a[2];
        };
    }
    if (type.isMessage() && !type.isArray())
        throw MappingError(where + ": whole " + type.base + " assignment is not supported; map its fields");
    const auto &member = ros.member();
    if (type.isArray() && !ros.indexed()) {
        if (type.isMessage() || spec.shape.empty())
            throw MappingError(where + ": cannot assign native " + spec.describe() + " to ROS " + type.base + " array");
        long size = 1;
        for (const int dimension : spec.shape)
            size = (dimension < 0 || size < 0) ? -1 : size * dimension;
        if (type.length >= 0 && size != type.length)
            throw MappingError(where + ": native " + spec.describe() + " (row-major) does not fill " + type.base + "[" +
                               std::to_string(type.length) + "]");
        if (type.base == "boolean")
            throw MappingError(where + ": boolean arrays are not supported");
        Spec element_spec{spec.dtype, {}};
        const Store store = scalarStore(member, type.element(), element_spec, where);
        const bool fixed = type.length >= 0;
        const int length = type.length;
        const auto *member_ptr = &member;
        return [store, fixed, length, member_ptr, where](void *target, const Value &v) {
            const std::size_t n = v.a.size();
            if (fixed && static_cast<int>(n) != length)
                throw MappingError(where + ": runtime size " + std::to_string(n) + " != " + std::to_string(length));
            if (!fixed)
                member_ptr->resize_function(target, n);
            for (std::size_t k = 0; k < n; ++k) {
                Value element = Value::real(v.a[k]);
                store(member_ptr->get_function(target, k), element);
            }
        };
    }
    if (!spec.shape.empty())
        throw MappingError(where + ": cannot assign native " + spec.describe() + " to scalar " + type.base);
    return scalarStore(member, type, spec, where);
}

Spec constantSpec(const Json &value, const std::string &where) {
    if (value.is_boolean())
        return booleanSpec();
    if (value.is_number_integer())
        return integerSpec();
    if (value.is_number_float())
        return scalarSpec();
    if (value.is_string())
        return stringSpec();
    if (value.is_array() && !value.empty()) {
        bool floats = false, ints = false;
        for (const auto &item : value) {
            if (item.is_array())
                throw MappingError(where + ": nested constant arrays are not supported");
            if (item.is_number_float())
                floats = true;
            else if (item.is_number_integer())
                ints = true;
            else
                throw MappingError(where + ": mixed constant array");
        }
        (void)ints;
        return {floats ? Dtype::Float : Dtype::Int, {static_cast<int>(value.size())}};
    }
    throw MappingError(where + ": unsupported constant " + pyRepr(value));
}
Value constantValue(const Json &value) {
    if (value.is_boolean())
        return Value::boolean(value.get<bool>());
    if (value.is_number_integer())
        return Value::integer(value.get<std::int64_t>());
    if (value.is_number_float())
        return Value::real(value.get<double>());
    if (value.is_string())
        return Value::text(value.get<std::string>());
    std::vector<double> items;
    for (const auto &item : value)
        items.push_back(item.get<double>());
    return Value::array(std::move(items));
}
} // namespace

std::string pyRepr(const Json &value) {
    if (value.is_boolean())
        return value.get<bool>() ? "True" : "False";
    if (value.is_string())
        return repr(value.get<std::string>());
    if (value.is_null())
        return "None";
    return value.dump();
}

void Writer::apply(void *message, const Value &values) const {
    for (const auto &step : steps_)
        step.produce(locate(message, step.path), values);
}

std::shared_ptr<Message> Writer::make(const std::shared_ptr<const MessageType> &type, const Value &values) const {
    auto message = std::make_shared<Message>(type);
    apply(message->data(), values);
    return message;
}

Writer compileWriter(const introspection::MessageMembers *type, const Json &fields, const SpecTree &sources,
                     const std::optional<std::string> &frame_id, const std::string &where) {
    Writer writer;
    bool stamped = false;
    for (std::uint32_t k = 0; k < type->member_count_; ++k)
        stamped = stamped || std::string(type->members_[k].name_) == "header";
    if (frame_id) {
        if (!frame_id->empty() && !stamped)
            throw MappingError(where + ": frame_id " + repr(*frame_id) + " set on unstamped " + type->message_name_);
        if (stamped && frame_id->empty())
            throw MappingError(where + ": stamped " + type->message_name_ + " requires a frame_id");
        if (stamped) {
            if (!fields.contains("header.stamp"))
                throw MappingError(where + ": stamped " + type->message_name_ + " must map header.stamp");
            const auto path = resolveField(type, "header.frame_id", true);
            const std::string frame = *frame_id;
            writer.steps_.push_back(
                {path, [frame](void *target, const Value &) { *static_cast<std::string *>(target) = frame; }});
        }
    }
    for (const auto &[destination, source] : fields.items()) {
        const std::string at = where + "/" + destination;
        const FieldPath ros = resolveField(type, destination, true);
        if (source.contains("constant")) {
            const Json &constant = source["constant"];
            const Spec spec = constantSpec(constant, at);
            const Setter set = makeSetter(ros, spec, at);
            const Value fixed = constantValue(constant);
            // Range-check integer constants when the map is compiled.
            if (isIntBase(ros.type.base) && !ros.type.isArray())
                checkRange(ros.type.base, fixed.asInt(), at);
            else if (isIntBase(ros.type.base) && ros.type.isArray())
                for (const double item : fixed.a)
                    checkRange(ros.type.base, static_cast<std::int64_t>(item), at);
            writer.steps_.push_back({ros, [set, fixed](void *target, const Value &) { set(target, fixed); }});
            continue;
        }
        const std::string path = source.at("from").get<std::string>();
        Spec spec;
        const SourceRef ref = SourceRef::compile(sources, path, spec);
        if (source.contains("enum_map")) {
            std::map<std::string, std::int64_t> table;
            for (const auto &[state, number] : source["enum_map"].items())
                table[state] = number.get<std::int64_t>();
            if (spec != stringSpec())
                throw MappingError(at + ": enum_map needs a native string state, got " + spec.describe());
            const Setter set = makeSetter(ros, integerSpec(), at);
            if (isIntBase(ros.type.base) && !ros.type.isArray())
                for (const auto &entry : table)
                    checkRange(ros.type.base, entry.second, at);
            writer.steps_.push_back({ros, [ref, table, set, at](void *target, const Value &values) {
                                         const Value state = ref.read(values);
                                         const auto found = table.find(state.s);
                                         if (found == table.end())
                                             throw MappingError(at + ": native state " + repr(state.s) +
                                                                " has no enum_map entry");
                                         set(target, Value::integer(found->second));
                                     }});
            continue;
        }
        const Setter set = makeSetter(ros, spec, at);
        writer.steps_.push_back(
            {ros, [ref, set](void *target, const Value &values) { set(target, ref.read(values)); }});
    }
    return writer;
}

// ------------------------------------------------------------------ ROS -> native

namespace {
Spec rosSpec(const FieldPath &field, const std::string &where) {
    const RosType &t = field.type;
    if (t.base == kQuaternion && !t.isArray())
        throw MappingError(where + ": read quaternion w/x/y/z components explicitly");
    if (isVectorMessage(t))
        return vector3Spec();
    if (t.isMessage())
        throw MappingError(where + ": cannot read whole " + t.base);
    Dtype dtype;
    if (isFloatBase(t.base))
        dtype = Dtype::Float;
    else if (isIntBase(t.base))
        dtype = Dtype::Int;
    else if (t.base == "boolean")
        dtype = Dtype::Bool;
    else if (t.base == "string")
        dtype = Dtype::String;
    else
        throw MappingError(where + ": unsupported ROS type " + t.base);
    if (t.isArray()) {
        if (t.base == "boolean")
            throw MappingError(where + ": boolean arrays are not supported");
        return {dtype, {t.length >= 0 ? t.length : -1}};
    }
    return {dtype, {}};
}

double readDouble(std::uint8_t id, const void *p) {
    using namespace introspection;
    if (id == ROS_TYPE_FLOAT)
        return *static_cast<const float *>(p);
    if (id == ROS_TYPE_DOUBLE)
        return *static_cast<const double *>(p);
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
        return static_cast<double>(*static_cast<const std::uint64_t *>(p));
    if (id == ROS_TYPE_INT64)
        return static_cast<double>(*static_cast<const std::int64_t *>(p));
    throw MappingError("unsupported ROS numeric type");
}
std::int64_t readInteger(std::uint8_t id, const void *p) {
    using namespace introspection;
    if (id == ROS_TYPE_UINT64)
        return static_cast<std::int64_t>(*static_cast<const std::uint64_t *>(p));
    if (id == ROS_TYPE_INT64)
        return *static_cast<const std::int64_t *>(p);
    return static_cast<std::int64_t>(readDouble(id, p));
}

std::function<Value(const void *)> makeRead(const FieldPath &field, const Spec &spec) {
    const auto *member = &field.member();
    const std::uint8_t id = member->type_id_;
    if (spec == vector3Spec() && isVectorMessage(field.type))
        return [](const void *p) {
            const auto *v = static_cast<const geometry_msgs::msg::Vector3 *>(p);
            return Value::array({v->x, v->y, v->z});
        };
    if (!spec.shape.empty()) {
        const int expected = spec.shape[0];
        const std::string path = field.path;
        return [member, id, expected, path, spec](const void *p) {
            const std::size_t n = member->size_function(p);
            if (expected >= 0 && static_cast<std::size_t>(expected) != n)
                throw MappingError(repr(path) + ": received shape [" + std::to_string(n) + "] does not match native " +
                                   spec.describe());
            std::vector<double> out(n);
            for (std::size_t k = 0; k < n; ++k)
                out[k] = readDouble(id, member->get_const_function(p, k));
            return Value::array(std::move(out));
        };
    }
    switch (spec.dtype) {
    case Dtype::Float:
        return [id](const void *p) { return Value::real(readDouble(id, p)); };
    case Dtype::Int:
        return [id](const void *p) { return Value::integer(readInteger(id, p)); };
    case Dtype::Bool:
        return [](const void *p) { return Value::boolean(*static_cast<const bool *>(p)); };
    case Dtype::String:
        return [](const void *p) { return Value::text(*static_cast<const std::string *>(p)); };
    default:
        break;
    }
    throw MappingError("unsupported native type " + spec.describe());
}

std::string listRepr(const std::set<std::string> &items) {
    std::string out = "[";
    bool first = true;
    for (const auto &item : items) {
        out += (first ? "" : ", ") + repr(item);
        first = false;
    }
    return out + "]";
}
} // namespace

Reader compileReader(const introspection::MessageMembers *type, const Json &fields,
                     const std::map<std::string, Spec> &arguments, const Json &accept_if, const std::string &where) {
    std::set<std::string> given, wanted, unknown, missing;
    for (const auto &item : fields.items())
        given.insert(item.key());
    for (const auto &item : arguments)
        wanted.insert(item.first);
    std::set_difference(given.begin(), given.end(), wanted.begin(), wanted.end(),
                        std::inserter(unknown, unknown.end()));
    std::set_difference(wanted.begin(), wanted.end(), given.begin(), given.end(),
                        std::inserter(missing, missing.end()));
    if (!unknown.empty() || !missing.empty())
        throw MappingError(where + ": native arguments must be exactly " + listRepr(wanted) + " (unknown " +
                           listRepr(unknown) + ", missing " + listRepr(missing) + ")");
    Reader reader;
    for (const auto &[argument, source] : fields.items()) {
        const std::string at = where + "/" + argument;
        if (!source.contains("from") || source.size() != 1)
            throw MappingError(at + ": inbound maps accept only {from: path}");
        const FieldPath field = resolveField(type, source["from"].get<std::string>(), false);
        const Spec actual = rosSpec(field, at);
        const Spec &expected = arguments.at(argument);
        if (!compatible(expected, actual))
            throw MappingError(at + ": ROS " + actual.describe() + " does not provide native " + expected.describe());
        reader.steps_.push_back({argument, field, expected, makeRead(field, expected)});
    }
    if (accept_if.is_array()) {
        for (const auto &condition : accept_if) {
            const std::string name = condition.at("field").get<std::string>();
            const std::string at = where + "/accept_if/" + name;
            const FieldPath field = resolveField(type, name, false);
            const Spec actual = rosSpec(field, at);
            const Json &equals = condition.at("equals");
            if (!actual.shape.empty() || !compatible(actual, constantSpec(equals, at)))
                throw MappingError(at + ": cannot compare ROS " + actual.describe() + " with " + pyRepr(equals));
            reader.filters_.push_back({field, constantValue(equals), actual.dtype});
        }
    }
    return reader;
}

bool Reader::accepts(const void *message) const {
    for (const auto &filter : filters_) {
        const void *p;
        try {
            p = locateConst(message, filter.path);
        } catch (const MappingError &) {
            throw MappingError("accept_if field index beyond the received sequence");
        }
        const auto id = filter.path.member().type_id_;
        bool equal = false;
        switch (filter.dtype) {
        case Dtype::Float:
            equal = readDouble(id, p) == filter.equals.asDouble();
            break;
        case Dtype::Int:
            equal = readInteger(id, p) == filter.equals.asInt();
            break;
        case Dtype::Bool:
            equal = *static_cast<const bool *>(p) == (filter.equals.asInt() != 0);
            break;
        case Dtype::String:
            equal = *static_cast<const std::string *>(p) == filter.equals.s;
            break;
        default:
            break;
        }
        if (!equal)
            return false;
    }
    return true;
}

Value Reader::operator()(const void *message) const {
    std::map<std::string, Value> out;
    for (const auto &step : steps_)
        out[step.argument] = step.read(locateConst(message, step.path));
    return Value::map(std::move(out));
}

} // namespace robotics::ros_bridge
