#include "prior_map.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace nereus::ros_viewer::host::prior_map {
namespace {
constexpr double kPi = 3.14159265358979323846;
const std::set<std::string> kDeprecated{"prequal_gate", "prequal_pole"};

double radians(double degrees) {
    return degrees * kPi / 180;
}
double round6(double value) {
    const double r = std::round(value * 1e6) / 1e6;
    return r == 0 ? 0 : r; // never "-0.0"
}
// Always a YAML float ("2.0", never "2"): ROS 2 then types the parameter as a double.
std::string formatFloat(double value) {
    char text[64];
    std::snprintf(text, sizeof(text), "%.6f", round6(value));
    std::string s = text;
    while (s.size() > 1 && s.back() == '0' && s[s.size() - 2] != '.')
        s.pop_back();
    return s;
}
std::string parentText(const std::string &parent) {
    return parent == kMap ? kMap : parent + "_frame";
}
bool plain(const std::string &text) {
    return !text.empty() && std::all_of(text.begin(), text.end(), [](unsigned char c) {
        return std::isalnum(c) || c == '_' || c == '-' || c == '.';
    });
}

// ---- source positions: yaml-cpp gives a node's start (line, column); its text is found and checked there.
struct Source {
    const std::string &text;
    std::vector<std::size_t> lines; // byte offset of each line start
    explicit Source(const std::string &t) : text(t) {
        lines.push_back(0);
        for (std::size_t i = 0; i < text.size(); ++i)
            if (text[i] == '\n')
                lines.push_back(i + 1);
    }
    std::size_t offset(const YAML::Mark &mark) const {
        if (mark.line < 0 || std::size_t(mark.line) >= lines.size())
            throw std::runtime_error("cannot locate a value in the file");
        return lines[std::size_t(mark.line)] + std::size_t(std::max(0, mark.column));
    }
    std::size_t lineStart(std::size_t at) const {
        const auto found = text.rfind('\n', at == 0 ? 0 : at - 1);
        return at == 0 || found == std::string::npos ? 0 : found + 1;
    }
    std::size_t lineEnd(std::size_t at) const { // just past the newline
        const auto found = text.find('\n', at);
        return found == std::string::npos ? text.size() : found + 1;
    }
    std::size_t indent(std::size_t lineAt) const {
        std::size_t i = lineAt;
        while (i < text.size() && text[i] == ' ')
            ++i;
        return i - lineAt;
    }
    // The [start, end) of a plain or quoted scalar beginning at `start`.
    std::size_t scalarEnd(std::size_t start) const {
        if (start < text.size() && (text[start] == '"' || text[start] == '\'')) {
            const char quote = text[start];
            for (std::size_t i = start + 1; i < text.size(); ++i)
                if (text[i] == quote && !(quote == '\'' && i + 1 < text.size() && text[i + 1] == '\'') &&
                    !(quote == '"' && text[i - 1] == '\\'))
                    return i + 1;
            throw std::runtime_error("unterminated quoted value");
        }
        std::size_t end = start;
        while (end < text.size() && text[end] != '\n' && text[end] != ',' && text[end] != '}' &&
               !(text[end] == '#' && end > start && text[end - 1] == ' '))
            ++end;
        while (end > start && (text[end - 1] == ' ' || text[end - 1] == '\r'))
            --end;
        return end;
    }
    // The range of a scalar node, checked against its parsed value so a misplaced edit can never happen.
    std::pair<std::size_t, std::size_t> scalar(const YAML::Node &node) const {
        const auto start = offset(node.Mark());
        const auto end = scalarEnd(start);
        std::string raw = text.substr(start, end - start);
        if (raw.size() >= 2 && (raw.front() == '"' || raw.front() == '\''))
            raw = raw.substr(1, raw.size() - 2);
        if (raw != node.Scalar())
            throw std::runtime_error("value '" + node.Scalar() + "' not found where the parser put it");
        return {start, end};
    }
    // A key's whole entry: its line through every following line indented deeper (blank lines inside kept,
    // trailing ones left for whatever follows).
    std::pair<std::size_t, std::size_t> block(const YAML::Node &key) const {
        const auto first = lineStart(offset(key.Mark()));
        const auto keyIndent = indent(first);
        std::size_t end = lineEnd(first), last = end;
        while (end < text.size()) {
            const auto next = lineEnd(end);
            const auto content = text.find_first_not_of(" \t\r", end);
            const bool blank = content == std::string::npos || content >= next - 1 || text[content] == '\n';
            if (!blank && indent(end) <= keyIndent)
                break;
            end = next;
            if (!blank)
                last = end;
        }
        return {first, last};
    }
};
struct Splice {
    std::size_t start, end;
    std::string text;
};

// A missing key is an undefined node (YAML::Node() would be a defined null, and test true).
YAML::Node child(const YAML::Node &map, const std::string &key) {
    if (map.IsDefined() && map.IsMap())
        for (auto it = map.begin(); it != map.end(); ++it)
            if (it->first.Scalar() == key)
                return it->second;
    return YAML::Node(YAML::NodeType::Undefined);
}
YAML::Node keyNode(const YAML::Node &map, const std::string &key) {
    if (map.IsDefined() && map.IsMap())
        for (auto it = map.begin(); it != map.end(); ++it)
            if (it->first.Scalar() == key)
                return it->first;
    return YAML::Node(YAML::NodeType::Undefined);
}
YAML::Node initData(const YAML::Node &root, const std::string &ns) {
    return child(child(child(root, ns), "ros__parameters"), "init_data");
}
double number(const YAML::Node &map, const char *key, double fallback) {
    const auto value = child(map, key);
    if (!value || !value.IsScalar())
        return fallback;
    try {
        return value.as<double>();
    } catch (const YAML::Exception &) {
        return fallback;
    }
}
bool flag(const YAML::Node &map, const char *key) {
    const auto value = child(map, key);
    try {
        return value && value.IsScalar() && value.as<bool>();
    } catch (const YAML::Exception &) {
        return false;
    }
}

// Structural equality of two YAML trees; `skip` (a path) is not compared.
bool same(const YAML::Node &a, const YAML::Node &b, std::vector<std::string> &path,
          const std::vector<std::string> &skip) {
    if (path == skip)
        return true;
    if (a.Type() != b.Type())
        return false;
    if (a.IsScalar())
        return a.Scalar() == b.Scalar();
    if (a.IsSequence()) {
        if (a.size() != b.size())
            return false;
        for (std::size_t i = 0; i < a.size(); ++i) {
            path.push_back("#" + std::to_string(i));
            const bool ok = same(a[i], b[i], path, skip);
            path.pop_back();
            if (!ok)
                return false;
        }
        return true;
    }
    if (a.IsMap()) {
        if (a.size() != b.size())
            return false;
        for (auto it = a.begin(); it != a.end(); ++it) {
            const auto key = it->first.Scalar();
            const auto other = child(b, key);
            if (!other)
                return false;
            path.push_back(key);
            const bool ok = same(it->second, other, path, skip);
            path.pop_back();
            if (!ok)
                return false;
        }
        return true;
    }
    return true;
}
bool sameObject(const Object &a, const Object &b) {
    const auto near = [](double x, double y) { return std::abs(round6(x) - round6(y)) < 5e-7; };
    return a.name == b.name && a.parent == b.parent && a.cls == b.cls && a.lockOrientation == b.lockOrientation &&
           a.pointYawAtParent == b.pointYawAtParent && near(a.pose.x, b.pose.x) && near(a.pose.y, b.pose.y) &&
           near(a.pose.z, b.pose.z) && near(a.pose.yaw, b.pose.yaw) && near(a.covar.x, b.covar.x) &&
           near(a.covar.y, b.covar.y) && near(a.covar.z, b.covar.z) && near(a.covar.yaw, b.covar.yaw);
}
} // namespace

double wrapDegrees(double degrees) {
    double a = std::fmod(std::fmod(degrees + 180.0, 360.0) + 360.0, 360.0) - 180.0;
    return a == -180.0 ? 180.0 : a;
}
Pose compose(const Pose &parent, const Pose &child) {
    const double c = std::cos(radians(parent.yaw)), s = std::sin(radians(parent.yaw));
    return {parent.x + child.x * c - child.y * s, parent.y + child.x * s + child.y * c, parent.z + child.z,
            wrapDegrees(parent.yaw + child.yaw)};
}
Pose decompose(const Pose &parent, const Pose &mapPose) {
    const double c = std::cos(radians(parent.yaw)), s = std::sin(radians(parent.yaw));
    const double dx = mapPose.x - parent.x, dy = mapPose.y - parent.y;
    return {dx * c + dy * s, -dx * s + dy * c, mapPose.z - parent.z, wrapDegrees(mapPose.yaw - parent.yaw)};
}

bool deprecated(const std::string &name) {
    return kDeprecated.count(name) > 0;
}

Document load(const std::string &text, const std::string &preferredNs) {
    const YAML::Node root = YAML::Load(text);
    if (!root.IsMap())
        throw std::runtime_error("not a riptide_mapping config: the file is not a mapping");
    Document doc;
    doc.text = text;
    for (auto it = root.begin(); it != root.end(); ++it) {
        const auto key = it->first.Scalar();
        if (initData(root, key).IsMap() && key.find("liltank") == std::string::npos)
            doc.namespaces.push_back(key);
    }
    if (doc.namespaces.empty())
        throw std::runtime_error("no '<ns>/riptide_mapping2: ros__parameters: init_data' in the file");
    const auto talos = std::find_if(doc.namespaces.begin(), doc.namespaces.end(),
                                    [](const std::string &n) { return n.find("talos") != std::string::npos; });
    doc.ns = std::find(doc.namespaces.begin(), doc.namespaces.end(), preferredNs) != doc.namespaces.end() ? preferredNs
             : talos != doc.namespaces.end() ? *talos
                                             : doc.namespaces[0];
    const auto init = initData(root, doc.ns);
    for (auto it = init.begin(); it != init.end(); ++it) {
        const auto name = it->first.Scalar();
        const auto entry = it->second;
        if (name.empty() || deprecated(name) || !entry.IsMap())
            continue;
        Object o;
        o.name = name;
        const auto parent = child(entry, "parent");
        o.parent = parent && parent.IsScalar() ? parent.Scalar() : kMap;
        constexpr std::string_view suffix = "_frame";
        if (o.parent != kMap && o.parent.size() > suffix.size() &&
            o.parent.compare(o.parent.size() - suffix.size(), suffix.size(), suffix) == 0)
            o.parent.resize(o.parent.size() - suffix.size());
        const auto pose = child(entry, "pose"), covar = child(entry, "covar");
        o.pose = {number(pose, "x", 0), number(pose, "y", 0), number(pose, "z", 0), number(pose, "yaw", 0)};
        o.covar = {number(covar, "x", 1), number(covar, "y", 1), number(covar, "z", 1), number(covar, "yaw", 1)};
        o.lockOrientation = flag(entry, "lock_orientation_to_config");
        o.pointYawAtParent = flag(entry, "point_yaw_at_parent");
        const auto cls = child(entry, "class");
        o.cls = cls && cls.IsScalar() ? cls.Scalar() : "";
        o.locked = o.parent != kMap; // assemblies move as one; children are unlocked to adjust them
        doc.objects.push_back(o);
        doc.loadedNames.push_back(name);
    }
    return doc;
}

std::string save(const Document &doc) {
    const YAML::Node root = YAML::Load(doc.text);
    const auto init = initData(root, doc.ns);
    if (!init.IsMap() || init.size() == 0 || init.Style() == YAML::EmitterStyle::Flow)
        throw std::runtime_error(doc.ns + " init_data is empty or written inline: add a first entry by hand");
    const Source src(doc.text);
    const auto firstKey = init.begin()->first;
    const auto entryLine = src.lineStart(src.offset(firstKey.Mark()));
    const std::string entryIndent(src.indent(entryLine), ' ');
    std::size_t step = 2;
    if (const auto parentKey = keyNode(init.begin()->second, "parent"))
        step = std::max<std::size_t>(1, src.indent(src.lineStart(src.offset(parentKey.Mark()))) - entryIndent.size());
    std::vector<Splice> splices;
    std::vector<std::string> fresh;
    const auto entryText = [&](const Object &o) {
        if (!plain(o.name) || (!o.cls.empty() && !plain(o.cls)))
            throw std::runtime_error("'" + o.name + "': names and classes are letters, digits, '_', '-' or '.'");
        const std::string f = entryIndent + std::string(step, ' '), n = f + std::string(step, ' ');
        std::string t = entryIndent + o.name + ":\n" + f + "parent: " + parentText(o.parent) + "\n";
        if (!o.cls.empty())
            t += f + "class: " + o.cls + "\n";
        if (o.lockOrientation)
            t += f + "lock_orientation_to_config: true\n";
        if (o.pointYawAtParent)
            t += f + "point_yaw_at_parent: true\n";
        t += f + "covar:\n" + n + "x: " + formatFloat(o.covar.x) + "\n" + n + "y: " + formatFloat(o.covar.y) + "\n" +
             n + "z: " + formatFloat(o.covar.z) + "\n" + n + "yaw: " + formatFloat(o.covar.yaw) + "\n";
        t += f + "pose:\n" + n + "x: " + formatFloat(o.pose.x) + "\n" + n + "y: " + formatFloat(o.pose.y) + "\n" + n +
             "z: " + formatFloat(o.pose.z) + "\n" + n + "yaw: " + formatFloat(o.pose.yaw) + "\n";
        return t;
    };
    for (const auto &o : doc.objects) {
        const auto entry = child(init, o.name);
        if (!entry) {
            fresh.push_back(entryText(o));
            continue;
        }
        if (!entry.IsMap())
            throw std::runtime_error(o.name + " is not a mapping in the file");
        const auto parentKey = keyNode(entry, "parent");
        const auto parent = child(entry, "parent");
        if (!parentKey || !parent.IsScalar())
            throw std::runtime_error(o.name + " has no parent line to edit");
        const std::string fieldIndent(src.indent(src.lineStart(src.offset(parentKey.Mark()))), ' ');
        const auto [parentStart, parentEnd] = src.scalar(parent);
        if (parent.Scalar() != parentText(o.parent)) {
            if (!plain(parentText(o.parent)))
                throw std::runtime_error("parent '" + o.parent + "' cannot be written plainly");
            splices.push_back({parentStart, parentEnd, parentText(o.parent)});
        }
        // class and the flags: replaced in place, or inserted after the parent line, or their line removed.
        const auto insertAt = src.lineEnd(parentEnd);
        std::string inserts;
        const auto setLine = [&](const char *key, bool wanted, const std::string &value) {
            const auto node = child(entry, key);
            if (wanted) {
                if (!node)
                    inserts += fieldIndent + key + ": " + value + "\n";
                else if (!node.IsScalar())
                    throw std::runtime_error(o.name + "." + key + " is not a plain value");
                else if (node.Scalar() != value) {
                    const auto [s, e] = src.scalar(node);
                    splices.push_back({s, e, value});
                }
            } else if (node) {
                const auto [s, e] = src.block(keyNode(entry, key));
                splices.push_back({s, e, ""});
            }
        };
        if (!o.cls.empty() && !plain(o.cls))
            throw std::runtime_error("class '" + o.cls + "' cannot be written plainly");
        setLine("class", !o.cls.empty(), o.cls);
        setLine("lock_orientation_to_config", o.lockOrientation, "true");
        setLine("point_yaw_at_parent", o.pointYawAtParent, "true");
        if (!inserts.empty())
            splices.push_back({insertAt, insertAt, inserts});
        for (const auto &[group, values] :
             {std::pair<const char *, std::array<double, 4>>{"covar", {o.covar.x, o.covar.y, o.covar.z, o.covar.yaw}},
              std::pair<const char *, std::array<double, 4>>{"pose", {o.pose.x, o.pose.y, o.pose.z, o.pose.yaw}}}) {
            const auto map = child(entry, group);
            if (!map.IsMap())
                throw std::runtime_error(o.name + " has no " + group + " to edit");
            const char *keys[] = {"x", "y", "z", "yaw"};
            for (int k = 0; k < 4; ++k) {
                const auto node = child(map, keys[k]);
                if (!node || !node.IsScalar())
                    throw std::runtime_error(o.name + "." + group + "." + keys[k] + " is missing");
                if (std::abs(number(map, keys[k], 1e300) - round6(values[std::size_t(k)])) < 5e-7)
                    continue;
                const auto [s, e] = src.scalar(node);
                splices.push_back({s, e, formatFloat(values[std::size_t(k)])});
            }
        }
    }
    // Deletions: only names the tool loaded and the operator removed.
    for (const auto &name : doc.loadedNames)
        if (!find(doc.objects, name) && !deprecated(name))
            if (const auto key = keyNode(init, name)) {
                const auto [s, e] = src.block(key);
                splices.push_back({s, e, ""});
            }
    if (!fresh.empty()) {
        // The last entry (an iterator: assigning a bound YAML::Node would overwrite the node it is bound to).
        auto last = init.begin();
        for (auto it = init.begin(); it != init.end(); ++it)
            last = it;
        const auto at = src.block(last->first).second;
        std::string text;
        for (const auto &entry : fresh)
            text += entry;
        splices.push_back({at, at, text});
    }
    std::sort(splices.begin(), splices.end(), [](const Splice &a, const Splice &b) {
        return a.start < b.start || (a.start == b.start && a.end < b.end);
    });
    for (std::size_t i = 1; i < splices.size(); ++i)
        if (splices[i].start < splices[i - 1].end)
            throw std::runtime_error("overlapping edits; nothing written");
    std::string out = doc.text;
    for (auto it = splices.rbegin(); it != splices.rend(); ++it)
        out.replace(it->start, it->end - it->start, it->text);

    // Check: it loads back as the objects, and nothing else in the file changed.
    const auto back = load(out, doc.ns);
    if (back.objects.size() != doc.objects.size())
        throw std::runtime_error("the saved file would not load back with the same objects; nothing written");
    for (const auto &o : doc.objects) {
        const auto *b = find(back.objects, o.name);
        if (!b || !sameObject(o, *b))
            throw std::runtime_error("the saved file would not load '" + o.name + "' back as edited; nothing written");
    }
    std::vector<std::string> path;
    if (!same(root, YAML::Load(out), path, {doc.ns, "ros__parameters", "init_data"}))
        throw std::runtime_error("the save would change other parts of the file; nothing written");
    const YAML::Node reloaded = YAML::Load(out); // kept alive: nodes found in it point into its memory
    const auto newInit = initData(reloaded, doc.ns);
    for (auto it = init.begin(); it != init.end(); ++it)
        if (deprecated(it->first.Scalar()) &&
            YAML::Dump(it->second) != YAML::Dump(child(newInit, it->first.Scalar()))) // untouched, as loaded
            throw std::runtime_error("the save would change " + it->first.Scalar() + "; nothing written");
    return out;
}

const Object *find(const std::vector<Object> &objects, const std::string &name) {
    for (const auto &o : objects)
        if (o.name == name)
            return &o;
    return nullptr;
}
Object *find(std::vector<Object> &objects, const std::string &name) {
    for (auto &o : objects)
        if (o.name == name)
            return &o;
    return nullptr;
}

std::map<std::string, Pose> mapPoses(const std::vector<Object> &objects) {
    std::map<std::string, Pose> memo;
    std::function<Pose(const std::string &, std::set<std::string> &)> pose = [&](const std::string &name,
                                                                                 std::set<std::string> &visiting) {
        if (const auto found = memo.find(name); found != memo.end())
            return found->second;
        const auto *o = find(objects, name);
        Pose result;
        if (o) {
            if (o->parent == kMap || !find(objects, o->parent) || visiting.count(o->parent))
                result = o->pose;
            else {
                visiting.insert(name);
                result = compose(pose(o->parent, visiting), o->pose);
                visiting.erase(name);
            }
        }
        memo[name] = result;
        return result;
    };
    for (const auto &o : objects) {
        std::set<std::string> visiting;
        pose(o.name, visiting);
    }
    return memo;
}

std::set<std::string> descendants(const std::vector<Object> &objects, const std::string &name) {
    std::set<std::string> out;
    std::vector<std::string> stack{name};
    while (!stack.empty()) {
        const auto current = stack.back();
        stack.pop_back();
        for (const auto &o : objects)
            if (o.parent == current && out.insert(o.name).second)
                stack.push_back(o.name);
    }
    return out;
}

Pose relativeUnder(const std::string &parent, const std::map<std::string, Pose> &poses, const Pose &mapPose) {
    const auto found = poses.find(parent);
    return parent == kMap || found == poses.end() ? mapPose : decompose(found->second, mapPose);
}

std::string uniqueName(const std::vector<Object> &objects, const std::string &base) {
    if (!find(objects, base))
        return base;
    for (int i = 2;; ++i)
        if (!find(objects, base + "_" + std::to_string(i)))
            return base + "_" + std::to_string(i);
}

bool reparent(std::vector<Object> &objects, const std::string &name, const std::string &parent, std::string *error) {
    auto *o = find(objects, name);
    if (!o || o->parent == parent)
        return false;
    if (parent != kMap && (parent == name || descendants(objects, name).count(parent) || !find(objects, parent))) {
        if (error)
            *error = "cannot parent an object to itself or its own descendant";
        return false;
    }
    const auto poses = mapPoses(objects);
    o->pose = relativeUnder(parent, poses, poses.at(name)); // stays where it is
    o->parent = parent;
    return true;
}

bool rename(std::vector<Object> &objects, const std::string &name, const std::string &next, std::string *error) {
    if (next.empty() || next == name || !find(objects, name))
        return false;
    if (find(objects, next) || deprecated(next) || !plain(next)) {
        if (error)
            *error = find(objects, next) ? "'" + next + "' already exists"
                     : deprecated(next)  ? "'" + next + "' is a reserved (deprecated) name"
                                         : "use letters, digits, '_', '-' or '.'";
        return false;
    }
    for (auto &o : objects) {
        if (o.name == name)
            o.name = next;
        else if (o.parent == name)
            o.parent = next;
    }
    return true;
}

void remove(std::vector<Object> &objects, const std::string &name) {
    const auto poses = mapPoses(objects);
    for (auto &o : objects)
        if (o.parent == name) {
            o.pose = poses.at(o.name);
            o.parent = kMap;
        }
    objects.erase(std::remove_if(objects.begin(), objects.end(), [&](const Object &o) { return o.name == name; }),
                  objects.end());
}

bool swapPoses(std::vector<Object> &objects, const std::string &a, const std::string &b, std::string *error) {
    auto *pa = find(objects, a), *pb = find(objects, b);
    if (!pa || !pb || a == b)
        return false;
    if (descendants(objects, a).count(b) || descendants(objects, b).count(a)) {
        if (error)
            *error = "cannot swap an object with its own ancestor or descendant";
        return false;
    }
    const auto poses = mapPoses(objects);
    const auto ra = relativeUnder(pa->parent, poses, poses.at(b)), rb = relativeUnder(pb->parent, poses, poses.at(a));
    pa->pose = ra;
    pb->pose = rb;
    return true;
}

void swapClasses(std::vector<Object> &objects, const std::string &a, const std::string &b) {
    auto *pa = find(objects, a), *pb = find(objects, b);
    if (pa && pb && a != b)
        std::swap(pa->cls, pb->cls);
}

void setMapPose(std::vector<Object> &objects, const std::string &name, const Pose &mapPose) {
    auto *o = find(objects, name);
    if (o)
        o->pose = relativeUnder(o->parent, mapPoses(objects), mapPose);
}

std::string add(std::vector<Object> &objects, const std::string &parent, const Pose &mapPose) {
    Object o;
    o.name = uniqueName(objects, "prop");
    o.parent = parent == kMap || find(objects, parent) ? parent : kMap;
    o.pose = relativeUnder(o.parent, mapPoses(objects), mapPose);
    objects.push_back(o);
    return o.name;
}

std::string duplicate(std::vector<Object> &objects, const std::string &name) {
    const auto *source = find(objects, name);
    if (!source)
        return {};
    Object copy = *source;
    copy.name = uniqueName(objects, name + "_copy");
    copy.pose.x += .5;
    copy.pose.y += .5;
    objects.push_back(copy);
    return copy.name;
}

Pose mapToPool(const Pose &p, const Origin &origin) {
    const double phi = radians(origin.yaw()), c = std::cos(phi), s = std::sin(phi);
    return {origin.x + p.x * c - p.y * s, origin.y + p.x * s + p.y * c, origin.z + p.z,
            wrapDegrees(p.yaw + origin.yaw())};
}
Pose poolToMap(const Pose &p, const Origin &origin) {
    const double phi = radians(origin.yaw()), c = std::cos(phi), s = std::sin(phi);
    const double dx = p.x - origin.x, dy = p.y - origin.y;
    return {dx * c + dy * s, -dx * s + dy * c, p.z - origin.z, wrapDegrees(p.yaw - origin.yaw())};
}
void keepInPool(std::vector<Object> &objects, const Origin &from, const Origin &to) {
    for (auto &o : objects)
        if (o.parent == kMap || !find(objects, o.parent))
            o.pose = poolToMap(mapToPool(o.pose, from), to);
}

std::vector<TagSpot> tagSpots(double length, double width, const std::vector<Line> &lines, double minimumLength) {
    std::vector<TagSpot> spots;
    const auto add = [&](double x, double y, double phi, char wall) {
        for (const auto &s : spots)
            if (s.wall == wall && std::abs(s.x - x) < .05 && std::abs(s.y - y) < .05)
                return;
        spots.push_back({x, y, phi, wall});
    };
    for (const auto &l : lines) {
        const double dx = l.x1 - l.x0, dy = l.y1 - l.y0;
        if (std::hypot(dx, dy) < minimumLength)
            continue;
        if (std::abs(dx) >= std::abs(dy) * 4) { // along the length: meets the x = 0 and x = length walls
            const double y = (l.y0 + l.y1) / 2;
            if (y > -.01 && y < width + .01) {
                add(0, y, 0, 'W');
                add(length, y, 180, 'E');
            }
        } else if (std::abs(dy) >= std::abs(dx) * 4) { // across: meets the y = 0 and y = width walls
            const double x = (l.x0 + l.x1) / 2;
            if (x > -.01 && x < length + .01) {
                add(x, 0, 90, 'S');
                add(x, width, 270, 'N');
            }
        }
    }
    for (const auto &[x, y] : {std::pair{0.0, 0.0}, {length, 0.0}, {0.0, width}, {length, width}}) {
        add(x, y, x == 0 ? 0 : 180, x == 0 ? 'W' : 'E');
        add(x, y, y == 0 ? 90 : 270, y == 0 ? 'S' : 'N');
    }
    return spots;
}

std::optional<TagSpot> nearestSpot(const std::vector<TagSpot> &spots, double x, double y, double reach) {
    std::optional<TagSpot> best;
    double bestDistance = 1e300;
    const auto wallDistance = [&](const TagSpot &s) {
        return s.wall == 'W' || s.wall == 'E' ? std::abs(x - s.x) : std::abs(y - s.y);
    };
    for (const auto &s : spots) {
        const double d = std::hypot(s.x - x, s.y - y);
        if (d > reach)
            continue;
        const bool tie = best && std::abs(d - bestDistance) < 1e-9;
        if (!best || d < bestDistance || (tie && wallDistance(s) < wallDistance(*best))) {
            best = s;
            bestDistance = d;
        }
    }
    return best;
}
} // namespace nereus::ros_viewer::host::prior_map
