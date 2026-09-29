// Equivalence of the C++ robosub_2026 rules with the Python hook. The fixture
// (extensions/rules/robosub_2026/tests/rules_calls.json, written by capture_fixture.py) holds every
// distinct hook call made by the Python reference tests (evaluate/describe/feed inputs and
// outputs, or the raised error). Every output must be identical JSON: integers, strings, booleans
// and structure exactly; floating point numbers to 1e-9 relative tolerance.
#include <gtest/gtest.h>

#include <rules/registry.hpp>

#include <cmath>
#include <fstream>
#include <map>

namespace {
using robotics::session::Json;

std::string firstDifference(const Json &a, const Json &b, const std::string &path) {
    if (a.is_number() && b.is_number()) {
        if (a.is_number_integer() != b.is_number_integer()) return path + ": integer vs float";
        if (a.is_number_integer()) return a == b ? "" : path + ": " + a.dump() + " != " + b.dump();
        double x = a.get<double>(), y = b.get<double>();
        return std::fabs(x - y) <= 1e-9 * std::max(std::fabs(x), std::fabs(y)) ? ""
                                                                                : path + ": " + a.dump() + " != " + b.dump();
    }
    if (a.type() != b.type()) return path + ": type " + a.dump() + " vs " + b.dump();
    if (a.is_object()) {
        if (a.size() != b.size()) return path + ": keys " + a.dump() + " vs " + b.dump();
        for (auto it = a.begin(); it != a.end(); ++it) {
            if (!b.contains(it.key())) return path + "/" + it.key() + " missing";
            if (auto d = firstDifference(it.value(), b.at(it.key()), path + "/" + it.key()); !d.empty()) return d;
        }
        return "";
    }
    if (a.is_array()) {
        if (a.size() != b.size()) return path + ": length " + std::to_string(a.size()) + " vs " + std::to_string(b.size());
        for (std::size_t i = 0; i < a.size(); ++i)
            if (auto d = firstDifference(a[i], b[i], path + "/" + std::to_string(i)); !d.empty()) return d;
        return "";
    }
    return a == b ? "" : path + ": " + a.dump() + " != " + b.dump();
}

Json loadCalls() {
    std::ifstream file(std::string(RP_SOURCE_DIR) + "/extensions/rules/robosub_2026/tests/rules_calls.json");
    EXPECT_TRUE(file.good());
    return Json::parse(file).at("calls");
}

std::unique_ptr<robotics::session::Rules> rules() {
    auto registry = robotics::rules::standardRules();
    return registry.at("robosub_2026")();
}

TEST(Robosub2026Rules, MatchesPythonHookOnEveryRecordedCall) {
    Json calls = loadCalls();
    ASSERT_GT(calls.size(), 100u);
    auto impl = rules();
    std::map<std::string, int> checked, errors;
    for (const Json &call : calls) {
        const std::string name = call.at("function");
        const Json &in = call.at("inputs");
        Json out;
        bool threw = false;
        try {
            if (name == "evaluate") out = impl->evaluate(in[0], in[1].get<robotics::session::Events>(), in[2]);
            else if (name == "describe") out = impl->describe(in[0], in[1]);
            else out = impl->feed(in[0], in[1].get<robotics::session::Events>(), in[2], in[3]);
        } catch (const std::exception &) {
            threw = true;
        }
        if (call.contains("error")) {
            EXPECT_TRUE(threw) << name << " should throw " << call.at("error") << " for " << in.dump();
            ++errors[name];
            continue;
        }
        ASSERT_FALSE(threw) << name << " threw for " << in.dump();
        std::string diff = firstDifference(out, call.at("output"), "");
        ASSERT_TRUE(diff.empty()) << name << " differs at " << diff << "\nexpected " << call.at("output").dump()
                                  << "\nactual   " << out.dump() << "\ninputs " << in.dump();
        ++checked[name];
    }
    EXPECT_GT(checked["evaluate"], 100);
    EXPECT_GT(checked["describe"], 100);
    EXPECT_GT(checked["feed"], 100);
    EXPECT_GE(errors["evaluate"], 1);
}

TEST(Robosub2026Rules, StatelessFeedRequiresState) {
    EXPECT_THROW(rules()->feed({}, Json::object(), Json::object()), std::logic_error);
}
} // namespace
