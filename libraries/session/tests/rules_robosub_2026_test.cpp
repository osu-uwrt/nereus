// The robosub_2026 rules against recorded calls. The fixture
// (extensions/rules/robosub_2026/tests/rules_calls.json) holds every distinct rules call of the
// reference scoring scripts (evaluate/describe/feed inputs and
// outputs, or the raised error). Every output must be identical JSON: integers, strings, booleans
// and structure exactly; floating point numbers to 1e-9 relative tolerance.
#include <gtest/gtest.h>

#include <rules/registry.hpp>

#include <cmath>
#include <fstream>
#include <map>

namespace {
using nereus::session::Json;

// Path of the first difference between two JSON values ("" if equal); floats compare to 1e-9 relative.
std::string firstDifference(const Json &a, const Json &b, const std::string &path) {
    if (a.is_number() && b.is_number()) {
        if (a.is_number_integer() != b.is_number_integer())
            return path + ": integer vs float";
        if (a.is_number_integer())
            return a == b ? "" : path + ": " + a.dump() + " != " + b.dump();
        double x = a.get<double>(), y = b.get<double>();
        return std::fabs(x - y) <= 1e-9 * std::max(std::fabs(x), std::fabs(y))
                   ? ""
                   : path + ": " + a.dump() + " != " + b.dump();
    }
    if (a.type() != b.type())
        return path + ": type " + a.dump() + " vs " + b.dump();
    if (a.is_object()) {
        if (a.size() != b.size())
            return path + ": keys " + a.dump() + " vs " + b.dump();
        for (auto it = a.begin(); it != a.end(); ++it) {
            if (!b.contains(it.key()))
                return path + "/" + it.key() + " missing";
            if (auto d = firstDifference(it.value(), b.at(it.key()), path + "/" + it.key()); !d.empty())
                return d;
        }
        return "";
    }
    if (a.is_array()) {
        if (a.size() != b.size())
            return path + ": length " + std::to_string(a.size()) + " vs " + std::to_string(b.size());
        for (std::size_t i = 0; i < a.size(); ++i)
            if (auto d = firstDifference(a[i], b[i], path + "/" + std::to_string(i)); !d.empty())
                return d;
        return "";
    }
    return a == b ? "" : path + ": " + a.dump() + " != " + b.dump();
}

// All recorded calls: {function, inputs, output | error}.
Json loadCalls() {
    std::ifstream file(std::string(NEREUS_SOURCE_DIR) + "/extensions/rules/robosub_2026/tests/rules_calls.json");
    EXPECT_TRUE(file.good());
    return Json::parse(file).at("calls");
}

// A fresh robosub_2026 rules instance from the standard registry.
std::unique_ptr<nereus::session::Rules> rules() {
    auto registry = nereus::rules::standardRules();
    return registry.at("robosub_2026")();
}

TEST(Robosub2026Rules, MatchesEveryRecordedCall) {
    Json calls = loadCalls();
    ASSERT_GT(calls.size(), 100u);
    auto impl = rules();
    std::map<std::string, int> checked, errors;
    for (const Json &call : calls) {

        // Dispatch on the recorded function name; inputs are positional.
        const std::string name = call.at("function");
        const Json &in = call.at("inputs");
        Json out;
        bool threw = false;
        try {
            if (name == "evaluate")
                out = impl->evaluate(in[0], in[1].get<nereus::session::Events>(), in[2]);
            else if (name == "describe")
                out = impl->describe(in[0], in[1]);
            else
                out = impl->feed(in[0], in[1].get<nereus::session::Events>(), in[2], in[3]);
        } catch (const std::exception &) {
            threw = true;
        }

        // Recorded errors only require that the call throws, not the same message.
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

    // Every function must have been exercised many times, plus at least one error case.
    EXPECT_GT(checked["evaluate"], 100);
    EXPECT_GT(checked["describe"], 100);
    EXPECT_GT(checked["feed"], 100);
    EXPECT_GE(errors["evaluate"], 1);
}

// The stateless feed overload throws: robosub_2026 needs the run snapshot passed by TaskRuntime.
TEST(Robosub2026Rules, StatelessFeedRequiresState) {
    EXPECT_THROW(rules()->feed({}, Json::object(), Json::object()), std::logic_error);
}
} // namespace
