#include "pose_command.hpp"
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <vector>

namespace nereus::ros_viewer::host {
namespace {
// Wrap to (-180, 180].
float wrapDegrees(float degrees) {
    degrees = std::fmod(degrees, 360.f);
    if (degrees <= -180)
        degrees += 360;
    else if (degrees > 180)
        degrees -= 360;
    return degrees;
}

// A number, with any unit after it ("0.5m", "30deg", "30°"); false if the word does not start with one.
bool number(const std::string &word, float &value) {
    const char *start = word.c_str();
    char *end = nullptr;
    value = std::strtof(start, &end);
    if (end == start || !std::isfinite(value))
        return false;
    const std::string unit(end);
    return unit.empty() || unit == "m" || unit == "deg" || unit == "°";
}

// printf one float into a short string.
std::string format(const char *pattern, float value) {
    char text[48];
    std::snprintf(text, sizeof(text), pattern, double(value));
    return text;
}
} // namespace

PoseTarget stepped(const PoseTarget &from, float forward, float left, float up, float turn) {
    PoseTarget out = from;
    const float yaw = glm::radians(from.degrees.z);
    out.position.x += forward * std::cos(yaw) - left * std::sin(yaw);
    out.position.y += forward * std::sin(yaw) + left * std::cos(yaw);
    out.position.z += up;
    out.degrees.z = wrapDegrees(from.degrees.z + turn);
    return out;
}

std::string describe(const PoseTarget &target) {
    std::string text = format("x %.2f", target.position.x) + format("  y %.2f", target.position.y) +
                       format("  z %.2f", target.position.z);
    if (std::abs(target.degrees.x) >= .05f || std::abs(target.degrees.y) >= .05f)
        text += format("  roll %.0f°", target.degrees.x) + format("  pitch %.0f°", target.degrees.y);
    return text + format("  yaw %.0f°", wrapDegrees(target.degrees.z));
}

ParsedMove parseMove(const std::string &text, const PoseTarget &from) {
    // Lower-case and split on spaces and commas.
    std::vector<std::string> words;
    {
        std::string spaced = text;
        for (char &c : spaced)
            c = c == ',' ? ' ' : char(std::tolower(static_cast<unsigned char>(c)));
        std::istringstream in(spaced);
        for (std::string word; in >> word;)
            words.push_back(word);
    }

    // Word classes: relative moves along the heading, absolute axes, and every move keyword.
    const auto along = [](const std::string &w) {
        return w == "forward" || w == "fwd" || w == "ahead" || w == "back" || w == "backward" || w == "backwards" ||
               w == "left" || w == "right" || w == "up" || w == "down";
    };
    const auto absolute = [](const std::string &w) {
        return w == "x" || w == "y" || w == "z" || w == "roll" || w == "pitch" || w == "yaw" || w == "heading";
    };
    const auto moveWord = [&](const std::string &w) {
        return along(w) || absolute(w) || w == "turn" || w == "go" || w == "goto" || w == "level";
    };

    // Not a move at all unless the first word is one; the palette treats it as something else.
    ParsedMove parsed;
    if (words.empty() || !moveWord(words[0]))
        return parsed;
    parsed.isMove = true;

    // Apply each move word in order to a running target; `said` collects the read-back.
    PoseTarget target = from;
    std::vector<std::string> said;
    std::size_t i = 0;
    // Consume the next word if it is a number.
    const auto next = [&](float &value) { return i < words.size() && number(words[i], value) ? (++i, true) : false; };
    while (i < words.size()) {
        const std::string word = words[i++];
        float value = 0;
        if (!moveWord(word)) {
            parsed.error = "\"" + word +
                           "\" is not a move: forward / back / left / right / up / down, turn, x / y / z, "
                           "roll / pitch / yaw, go, level";
            return parsed;
        }

        if (along(word)) {
            if (!next(value)) {
                parsed.error = word + " needs a distance in metres, e.g. " + word + " 0.5";
                return parsed;
            }
            const bool backward = word == "back" || word == "backward" || word == "backwards";
            const float forward = word == "forward" || word == "fwd" || word == "ahead" ? value : backward ? -value : 0;
            const float left = word == "left" ? value : word == "right" ? -value : 0;
            const float up = word == "up" ? value : word == "down" ? -value : 0;
            target = stepped(target, forward, left, up, 0);
            said.push_back((backward                           ? std::string("back")
                            : word == "fwd" || word == "ahead" ? "forward"
                                                               : word) +
                           format(" %g m", value));
        } else if (word == "turn") {
            float sign = 1;
            if (i < words.size() && (words[i] == "left" || words[i] == "right"))
                sign = words[i++] == "right" ? -1 : 1;
            if (!next(value)) {
                parsed.error = "turn needs degrees (left positive), e.g. turn 30 or turn right 45";
                return parsed;
            }
            target = stepped(target, 0, 0, 0, sign * value);
            said.push_back(format("turn %g°", sign * value));
        } else if (absolute(word)) {
            const bool angle = word == "roll" || word == "pitch" || word == "yaw" || word == "heading";
            if (!next(value)) {
                parsed.error = word + (angle ? " needs degrees, e.g. " : " needs metres, e.g. ") + word +
                               (angle         ? " 90"
                                : word == "z" ? " -1.5"
                                              : " 2");
                return parsed;
            }

            // x / y / z index position components 0 / 1 / 2.
            if (word == "x" || word == "y" || word == "z")
                target.position[word[0] - 'x'] = value;
            else if (word == "roll")
                target.degrees.x = value;
            else if (word == "pitch")
                target.degrees.y = value;
            else
                target.degrees.z = wrapDegrees(value);
            said.push_back((word == "heading" ? "yaw" : word) + format(angle ? " %g°" : " %g m", value));
        } else if (word == "go" || word == "goto") {
            // go x y z, go x y z yaw, or go x y z roll pitch yaw.
            float v[6];
            int count = 0;
            while (count < 6 && next(v[count]))
                ++count;
            if (count != 3 && count != 4 && count != 6) {
                parsed.error = "go needs x y z, then yaw or roll pitch yaw: e.g. go 2 1 -1.5 90";
                return parsed;
            }
            target.position = {v[0], v[1], v[2]};
            if (count == 4)
                target.degrees.z = wrapDegrees(v[3]);
            if (count == 6)
                target.degrees = {v[3], v[4], wrapDegrees(v[5])};
            said.push_back("go to " + describe(target));
        } else { // level
            target.degrees.x = target.degrees.y = 0;
            said.push_back("level");
        }
    }

    // Join the read-back parts into the summary.
    PoseCommand command{target, {}};
    for (const auto &part : said)
        command.summary += (command.summary.empty() ? "" : ", ") + part;
    parsed.command = command;
    return parsed;
}
} // namespace nereus::ros_viewer::host
