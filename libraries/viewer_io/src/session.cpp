#include <robotics/viewer/session.hpp>

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace robotics::viewer {
namespace v = visualization;
Sources localSources() {
    return {{"local", [](const std::filesystem::path &path, const std::string &id) {
                 auto source = std::make_shared<v::LocalSource>(id, loadRecording(path));
                 return Connection{source, source, source->duration(),
                                   [source] { source->reconnect(); }};
             }}};
}
Session::Session(Sources sources, v::Displays displays)
    : sources_(std::move(sources)), displays_(std::move(displays)), workspace_(emptyWorkspace()) {}
void Session::open(Workspace workspace) {
    std::map<std::string, Connection> connections;
    std::map<std::string, std::string> errors;
    for (const auto &binding : workspace.sources) {
        try {
            auto connection = sources_.at(binding.type)(binding.file, binding.id);
            if (!connection.source || connection.source->snapshot().id != binding.id ||
                connection.duration_ns < 0)
                throw std::invalid_argument("source factory returned an invalid connection");
            connections.emplace(binding.id, std::move(connection));
        } catch (const std::exception &error) {
            errors[binding.id] = sources_.count(binding.type)
                                     ? error.what()
                                     : "Unknown source type: " + binding.type;
        }
    }
    for (auto &[id, connection] : connections_) {
        (void)id;
        const auto source = connection.source;
        const bool retained =
            std::any_of(connections.begin(), connections.end(),
                        [source](const auto &entry) { return entry.second.source == source; });
        if (!retained)
            connection.source->disconnect();
    }
    connections_ = std::move(connections);
    errors_ = std::move(errors);
    workspace_ = std::move(workspace);
}
Workspace &Session::workspace() {
    return workspace_;
}
const std::map<std::string, std::string> &Session::errors() const {
    return errors_;
}
const Connection *Session::selected() const {
    const auto found = connections_.find(workspace_.selected_source);
    return found == connections_.end() ? nullptr : &found->second;
}
std::optional<v::SourceSnapshot> Session::snapshot() const {
    const auto *connection = selected();
    return connection ? std::optional<v::SourceSnapshot>(connection->source->snapshot())
                      : std::nullopt;
}
bool Session::canSeek() const {
    const auto *connection = selected();
    return connection && connection->playback && connection->source->snapshot().data;
}
v::Time Session::duration() const {
    const auto *connection = selected();
    return connection ? connection->duration_ns : 0;
}
void Session::seek(v::Time time_ns) {
    if (!canSeek())
        throw std::logic_error("selected source has no active playback capability");
    selected()->playback->seek(time_ns);
}
void Session::advance(v::Time elapsed_ns) {
    if (elapsed_ns < 0)
        throw std::invalid_argument("negative playback interval");
    const auto current = snapshot();
    if (!canSeek() || !current)
        return;
    seek(current->time_ns + std::min(elapsed_ns, duration() - current->time_ns));
}
void Session::disconnect() {
    if (const auto *connection = selected())
        connection->source->disconnect();
}
void Session::reconnect() {
    if (const auto *connection = selected(); connection && connection->reconnect)
        connection->reconnect();
}
Scene Session::scene() const {
    Scene result;
    const auto source = snapshot();
    for (const auto &settings : workspace_.displays) {
        if (!settings.source.empty() && settings.source != workspace_.selected_source)
            continue;
        try {
            auto display =
                displays_.draw({source ? &*source : nullptr, workspace_.fixed_frame, settings});
            if (display.lines.size() > 20000 || result.lines.size() + display.lines.size() > 200000)
                throw std::runtime_error("display exceeded geometry budget");
            for (const auto &line : display.lines)
                if (!line.from.allFinite() || !line.to.allFinite() ||
                    line.from.cwiseAbs().maxCoeff() > 1e14 ||
                    line.to.cwiseAbs().maxCoeff() > 1e14 ||
                    std::any_of(line.color.begin(), line.color.end(),
                                [](int value) { return value < 0 || value > 255; }))
                    throw std::runtime_error("display returned invalid geometry");
            result.lines.insert(result.lines.end(), display.lines.begin(), display.lines.end());
            result.status[settings.id] = {display.level, display.status};
        } catch (const std::exception &error) {
            result.status[settings.id] = {v::Level::error, error.what()};
        }
    }
    return result;
}
} // namespace robotics::viewer
