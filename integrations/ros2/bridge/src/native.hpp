#pragma once
// Typed native values and value specifications shared by the field-map compiler and the core
// (port of the Spec/SpecTree machinery in mapping.py). No ROS dependency.
#include <cstdint>
#include <initializer_list>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace robotics::ros_bridge {

// A declared field map cannot be applied to the declared ROS or native types.
class MappingError : public std::runtime_error {
  public:
    using std::runtime_error::runtime_error;
};
// Bridge configuration cannot run against this runtime and ROS installation.
class BridgeError : public std::runtime_error {
  public:
    using std::runtime_error::runtime_error;
};

std::string repr(const std::string &text); // Python-style quoted string, for identical messages

enum class Dtype { Float, Int, Bool, String, Time };

// dtype plus shape: {} scalar, otherwise dimensions with -1 meaning any size.
struct Spec {
    Dtype dtype{Dtype::Float};
    std::vector<int> shape;
    bool operator==(const Spec &other) const {
        return dtype == other.dtype && shape == other.shape;
    }
    bool operator!=(const Spec &other) const { return !(*this == other); }
    std::string describe() const;
};
Spec scalarSpec();  // float
Spec integerSpec(); // int
Spec booleanSpec();
Spec stringSpec();
Spec timeSpec();
Spec floatArray(std::vector<int> shape);
Spec vector3Spec();
Spec quaternionSpec();
Spec matrix3Spec();

// Nested mapping whose leaves are Spec.
struct SpecTree {
    std::optional<Spec> leaf;
    std::map<std::string, SpecTree> children;
    SpecTree() = default;
    SpecTree(Spec spec) : leaf(std::move(spec)) {}
    SpecTree(std::map<std::string, SpecTree> nodes) : children(std::move(nodes)) {}
};

// One native value; Map nodes mirror SpecTree. Arrays are row-major doubles.
struct Value {
    enum class Kind { None, Float, Int, Bool, String, Time, Array, Map };
    Kind kind{Kind::None};
    double f{0};
    std::int64_t i{0};
    bool b{false};
    std::string s;
    std::vector<double> a;
    std::map<std::string, Value> m;

    static Value real(double v);
    static Value integer(std::int64_t v);
    static Value boolean(bool v);
    static Value text(std::string v);
    static Value time(std::int64_t ns);
    static Value array(std::vector<double> v);
    static Value map(std::map<std::string, Value> v);
    const Value &at(const std::string &key) const; // throws MappingError when absent
    double asDouble() const;                       // Float or Int
    std::int64_t asInt() const;
};

struct PathToken {
    std::string name;
    std::vector<int> indexes;
};
// Dotted names with integer indexes, e.g. "a.b[0].c[1][2]"; throws MappingError otherwise.
std::vector<PathToken> parsePath(const std::string &path);

// Compiled read access into a native value tree (validated against a SpecTree).
class SourceRef {
  public:
    SourceRef() = default;
    // Throws MappingError with the messages of mapping.source_spec.
    static SourceRef compile(const SpecTree &tree, const std::string &path, Spec &out_spec);
    Value read(const Value &root) const;

  private:
    std::vector<std::string> keys_;
    std::size_t offset_{0}, count_{1};
    std::vector<int> shape_; // remaining shape after indexes
    Dtype dtype_{Dtype::Float};
    bool indexed_{false};
};

bool compatible(const Spec &expected, const Spec &actual);

} // namespace robotics::ros_bridge
