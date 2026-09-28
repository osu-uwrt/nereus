#pragma once
#include "robotics/config/scenario.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

namespace robotics::config::detail {
inline void keys(const YAML::Node &node, std::initializer_list<const char *> allowed,
                 const std::string &path) {
    if (!node.IsMap()) {
        throw std::invalid_argument(path + " must be a mapping");
    }
    const std::set<std::string> permitted(allowed.begin(), allowed.end());
    std::set<std::string> seen;
    for (const auto &item : node) {
        const auto key = item.first.as<std::string>();
        if (!permitted.count(key) || !seen.insert(key).second) {
            throw std::invalid_argument(path + "." + key + ": unknown or duplicate field");
        }
    }
}

inline double number(const YAML::Node &n, const char *key, const std::string &path) {
    try {
        const double value = n[key].as<double>();
        if (!std::isfinite(value)) {
            throw std::invalid_argument("must be finite");
        }
        return value;
    } catch (const std::exception &e) {
        throw std::invalid_argument(path + "." + key + ": " + e.what());
    }
}

inline std::uint64_t integer(const YAML::Node &n, const char *key, const std::string &path) {
    try {
        const auto text = n[key].as<std::string>();
        if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos) {
            throw std::invalid_argument("expected a nonnegative decimal integer");
        }
        return std::stoull(text);
    } catch (const std::exception &e) {
        throw std::invalid_argument(path + "." + key + ": " + e.what());
    }
}

inline Eigen::VectorXd vector(const YAML::Node &n, const char *key, Eigen::Index size,
                              const std::string &path) {
    const auto list = n[key];
    if (!list.IsSequence() || list.size() != static_cast<std::size_t>(size)) {
        throw std::invalid_argument(path + "." + key + ": expected " + std::to_string(size) +
                                    " values");
    }
    Eigen::VectorXd values(size);
    for (Eigen::Index i = 0; i < size; ++i) {
        try {
            values[i] = list[static_cast<std::size_t>(i)].as<double>();
        } catch (const YAML::Exception &e) {
            throw std::invalid_argument(path + "." + key + ": " + e.what());
        }
    }
    if (!values.allFinite()) {
        throw std::invalid_argument(path + "." + key + ": values must be finite");
    }
    return values;
}

inline Eigen::MatrixXd matrixOrDiagonal(const YAML::Node &node, const char *diagonal,
                                        const char *full, Eigen::Index size,
                                        const std::string &path) {
    if (node[diagonal].IsDefined() == node[full].IsDefined()) {
        throw std::invalid_argument(path + ": provide exactly one of " + diagonal + " or " + full);
    }
    if (node[diagonal]) {
        return vector(node, diagonal, size, path).asDiagonal();
    }
    const auto rows = node[full];
    if (!rows.IsSequence() || rows.size() != static_cast<std::size_t>(size)) {
        throw std::invalid_argument(path + "." + full + ": wrong matrix row count");
    }
    Eigen::MatrixXd result(size, size);
    for (Eigen::Index row = 0; row < size; ++row) {
        YAML::Node wrapper;
        wrapper["row"] = rows[static_cast<std::size_t>(row)];
        result.row(row) =
            vector(wrapper, "row", size, path + "." + full + "[" + std::to_string(row) + "]")
                .transpose();
    }
    return result;
}

inline std::string text(const YAML::Node &n, const char *key, const std::string &path) {
    try {
        if (!n[key].IsScalar()) {
            throw std::invalid_argument("expected a text scalar");
        }
        const auto value = n[key].as<std::string>();
        if (value.empty() || value.find('\0') != std::string::npos) {
            throw std::invalid_argument("expected a nonempty text value without NUL");
        }
        return value;
    } catch (const std::exception &e) {
        throw std::invalid_argument(path + "." + key + ": " + e.what());
    }
}
struct Document {
    YAML::Node root;
    std::filesystem::path path;
};
inline Document read(const std::filesystem::path &path,
                     std::vector<std::filesystem::path> &sources) {
    const auto resolved = std::filesystem::absolute(path).lexically_normal();
    try {
        auto root = YAML::LoadFile(resolved.string());
        if (std::find(sources.begin(), sources.end(), resolved) == sources.end()) {
            sources.push_back(resolved);
        }
        return {root, resolved};
    } catch (const std::exception &e) {
        throw std::invalid_argument(resolved.string() + ": " + e.what());
    }
}
inline Document reference(const YAML::Node &node, const char *key,
                          const std::filesystem::path &declaring, const char *kind,
                          std::vector<std::filesystem::path> &sources) {
    const auto name = text(node, key, declaring.string());
    if (name.find("://") != std::string::npos) {
        throw std::invalid_argument(declaring.string() +
                                    ": profile references must be filesystem paths");
    }
    auto doc = read(declaring.parent_path() / name, sources);
    if (integer(doc.root, "schema_version", doc.path.string()) != 1 ||
        text(doc.root, "kind", doc.path.string()) != kind) {
        throw std::invalid_argument(doc.path.string() + ": expected schema_version 1 and kind " +
                                    kind);
    }
    return doc;
}
SensorPlan parseSensor(const YAML::Node &node, const std::filesystem::path &declaring,
                       const std::string &field, std::vector<std::filesystem::path> &sources);
} // namespace robotics::config::detail
