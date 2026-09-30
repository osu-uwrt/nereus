#include "native.hpp"

#include <cctype>

namespace nereus::ros_bridge {

std::string repr(const std::string &text) {
    return "'" + text + "'";
}

std::string Spec::describe() const {
    static const char *names[] = {"float", "int", "bool", "string", "time"};
    std::string result = names[static_cast<int>(dtype)];
    if (!shape.empty()) {
        result += "[";
        for (std::size_t k = 0; k < shape.size(); ++k) {
            if (k)
                result += ", ";
            result += shape[k] < 0 ? "None" : std::to_string(shape[k]);
        }
        result += "]";
    }
    return result;
}
Spec scalarSpec() {
    return {Dtype::Float, {}};
}
Spec integerSpec() {
    return {Dtype::Int, {}};
}
Spec booleanSpec() {
    return {Dtype::Bool, {}};
}
Spec stringSpec() {
    return {Dtype::String, {}};
}
Spec timeSpec() {
    return {Dtype::Time, {}};
}
Spec floatArray(std::vector<int> shape) {
    return {Dtype::Float, std::move(shape)};
}
Spec vector3Spec() {
    return floatArray({3});
}
Spec quaternionSpec() {
    return floatArray({4});
}
Spec matrix3Spec() {
    return floatArray({3, 3});
}

Value Value::real(double v) {
    Value r;
    r.kind = Kind::Float;
    r.f = v;
    return r;
}
Value Value::integer(std::int64_t v) {
    Value r;
    r.kind = Kind::Int;
    r.i = v;
    return r;
}
Value Value::boolean(bool v) {
    Value r;
    r.kind = Kind::Bool;
    r.b = v;
    return r;
}
Value Value::text(std::string v) {
    Value r;
    r.kind = Kind::String;
    r.s = std::move(v);
    return r;
}
Value Value::time(std::int64_t ns) {
    Value r;
    r.kind = Kind::Time;
    r.i = ns;
    return r;
}
Value Value::array(std::vector<double> v) {
    Value r;
    r.kind = Kind::Array;
    r.a = std::move(v);
    return r;
}
Value Value::map(std::map<std::string, Value> v) {
    Value r;
    r.kind = Kind::Map;
    r.m = std::move(v);
    return r;
}
const Value &Value::at(const std::string &key) const {
    const auto found = m.find(key);
    if (kind != Kind::Map || found == m.end())
        throw MappingError("native value has no field " + repr(key));
    return found->second;
}
double Value::asDouble() const {
    if (kind == Kind::Float)
        return f;
    if (kind == Kind::Int || kind == Kind::Time)
        return static_cast<double>(i);
    if (kind == Kind::Bool)
        return b ? 1.0 : 0.0;
    throw MappingError("native value is not a number");
}
std::int64_t Value::asInt() const {
    if (kind == Kind::Int || kind == Kind::Time)
        return i;
    if (kind == Kind::Bool)
        return b ? 1 : 0;
    if (kind == Kind::Float)
        return static_cast<std::int64_t>(f);
    throw MappingError("native value is not an integer");
}

namespace {
bool identStart(char c) {
    return std::isalpha(static_cast<unsigned char>(c)) || c == '_';
}
bool identChar(char c) {
    return std::isalnum(static_cast<unsigned char>(c)) || c == '_';
}
} // namespace

std::vector<PathToken> parsePath(const std::string &path) {
    const auto invalid = [&] { return MappingError("invalid field path " + repr(path)); };
    std::vector<PathToken> tokens;
    std::size_t pos = 0;
    while (true) {
        if (pos >= path.size() || !identStart(path[pos]))
            throw invalid();
        PathToken token;
        const auto start = pos;
        while (pos < path.size() && identChar(path[pos]))
            ++pos;
        token.name = path.substr(start, pos - start);
        while (pos < path.size() && path[pos] == '[') {
            ++pos;
            const auto digits = pos;
            while (pos < path.size() && std::isdigit(static_cast<unsigned char>(path[pos])))
                ++pos;
            if (pos == digits || pos >= path.size() || path[pos] != ']' || pos - digits > 9)
                throw invalid();
            token.indexes.push_back(std::stoi(path.substr(digits, pos - digits)));
            ++pos;
        }
        tokens.push_back(std::move(token));
        if (pos == path.size())
            return tokens;
        if (path[pos] != '.')
            throw invalid();
        ++pos;
    }
}

SourceRef SourceRef::compile(const SpecTree &tree, const std::string &path, Spec &out_spec) {
    SourceRef ref;
    const SpecTree *node = &tree;
    for (const auto &token : parsePath(path)) {
        if (node == nullptr || node->leaf)
            throw MappingError("unknown native field " + repr(path));
        const auto found = node->children.find(token.name);
        if (found == node->children.end())
            throw MappingError("unknown native field " + repr(path));
        node = &found->second;
        ref.keys_.push_back(token.name);
        if (token.indexes.empty())
            continue;
        if (!node->leaf || node->leaf->shape.empty())
            throw MappingError("native field " + repr(path) + " is not indexable");
        const auto &shape = node->leaf->shape;
        Spec current = *node->leaf;
        std::size_t offset = 0, rest = 1;
        for (std::size_t d = 0; d < token.indexes.size(); ++d) {
            if (current.shape.empty())
                throw MappingError("native field " + repr(path) + " is not indexable");
            const int index = token.indexes[d], size = current.shape[0];
            if (size >= 0 && index >= size)
                throw MappingError("index " + std::to_string(index) + " out of range in native field " + repr(path));
            offset = offset * static_cast<std::size_t>(size < 0 ? 0 : size) + static_cast<std::size_t>(index);
            current.shape.erase(current.shape.begin());
        }
        for (std::size_t d = token.indexes.size(); d < shape.size(); ++d)
            rest *= static_cast<std::size_t>(shape[d] < 0 ? 0 : shape[d]);
        ref.indexed_ = true;
        ref.offset_ = offset * rest;
        ref.count_ = rest;
        ref.shape_ = current.shape;
        ref.dtype_ = current.dtype;
        out_spec = current;
        node = nullptr; // an index ends the path
    }
    if (!ref.indexed_) {
        if (!node->leaf)
            throw MappingError("native field " + repr(path) + " is a structure, not a value");
        out_spec = *node->leaf;
        ref.shape_ = out_spec.shape;
        ref.dtype_ = out_spec.dtype;
    }
    return ref;
}

Value SourceRef::read(const Value &root) const {
    const Value *node = &root;
    for (const auto &key : keys_)
        node = &node->at(key);
    if (!indexed_)
        return *node;
    if (node->kind != Value::Kind::Array || offset_ + count_ > node->a.size())
        throw MappingError("native array is shorter than the mapped index");
    if (shape_.empty()) {
        const double v = node->a[offset_];
        return dtype_ == Dtype::Int ? Value::integer(static_cast<std::int64_t>(v)) : Value::real(v);
    }
    return Value::array(std::vector<double>(node->a.begin() + static_cast<std::ptrdiff_t>(offset_),
                                            node->a.begin() + static_cast<std::ptrdiff_t>(offset_ + count_)));
}

bool compatible(const Spec &expected, const Spec &actual) {
    if (expected.dtype == Dtype::Float && actual.dtype != Dtype::Float && actual.dtype != Dtype::Int)
        return false;
    if (expected.dtype != Dtype::Float && expected.dtype != actual.dtype)
        return false;
    if (expected.shape.size() != actual.shape.size())
        return false;
    for (std::size_t k = 0; k < expected.shape.size(); ++k)
        if (expected.shape[k] >= 0 && actual.shape[k] >= 0 && expected.shape[k] != actual.shape[k])
            return false;
    return true;
}

} // namespace nereus::ros_bridge
