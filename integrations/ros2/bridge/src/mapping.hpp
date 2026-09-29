#pragma once
// Declarative field maps between native values and ROS messages, checked before stepping
// (port of the compile_writer/compile_reader half of mapping.py). A map is
//   {destination path: {from: path} | {constant: value} | {from: path, enum_map: {...}}}
// Paths are dotted names with integer indexes; there is no expression language. Every
// destination is resolved against the ROS type description and every source against a typed
// native value specification, so unknown fields and incompatible shapes fail at startup.
#include "native.hpp"
#include "ros_types.hpp"

#include <functional>
#include <optional>

namespace robotics::ros_bridge {

class Writer {
  public:
    // Applies every step to an initialised message of the compiled type.
    void apply(void *message, const Value &values) const;
    std::shared_ptr<Message> make(const std::shared_ptr<const MessageType> &type,
                                  const Value &values) const;

  private:
    friend Writer compileWriter(const introspection::MessageMembers *, const Json &,
                                const SpecTree &, const std::optional<std::string> &,
                                const std::string &);
    struct Step {
        FieldPath path;
        std::function<void(void *target, const Value &values)> produce;
    };
    std::vector<Step> steps_;
};

// `frame_id` nullopt: not a header-stamped stream (service responses).
Writer compileWriter(const introspection::MessageMembers *type, const Json &fields,
                     const SpecTree &sources, const std::optional<std::string> &frame_id,
                     const std::string &where);

class Reader {
  public:
    bool accepts(const void *message) const;       // may throw MappingError
    Value operator()(const void *message) const;   // Map of native arguments; may throw

  private:
    friend Reader compileReader(const introspection::MessageMembers *, const Json &,
                                const std::map<std::string, Spec> &, const Json &,
                                const std::string &);
    struct Step {
        std::string argument;
        FieldPath path;
        Spec spec;
        std::function<Value(const void *field)> read;
    };
    struct Filter {
        FieldPath path;
        Value equals;
        Dtype dtype;
    };
    std::vector<Step> steps_;
    std::vector<Filter> filters_;
};

Reader compileReader(const introspection::MessageMembers *type, const Json &fields,
                     const std::map<std::string, Spec> &arguments, const Json &accept_if,
                     const std::string &where);

// Python repr of a JSON scalar, used in messages.
std::string pyRepr(const Json &value);

} // namespace robotics::ros_bridge
