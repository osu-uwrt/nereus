#pragma once
#include <robotics/viewer/workspace.hpp>

namespace robotics::viewer {
struct Connection {
    std::shared_ptr<visualization::Source> source;
    std::shared_ptr<visualization::Playback> playback; // Optional, shares the source lifetime.
    visualization::Time duration_ns{0};
    std::function<void()> reconnect;
};
using SourceFactory = std::function<Connection(const std::filesystem::path &, const std::string &)>;
using Sources = std::map<std::string, SourceFactory>;
Sources localSources();
struct Scene {
    std::vector<visualization::Line> lines;
    std::map<std::string, std::pair<visualization::Level, std::string>> status;
};
class Session {
  public:
    Session(Sources sources, visualization::Displays displays);
    void open(Workspace workspace);
    Workspace &workspace();
    const std::map<std::string, std::string> &errors() const;
    std::optional<visualization::SourceSnapshot> snapshot() const;
    bool canSeek() const;
    visualization::Time duration() const;
    void seek(visualization::Time time_ns);
    void advance(visualization::Time elapsed_ns);
    void disconnect();
    void reconnect();
    Scene scene() const;

  private:
    const Connection *selected() const;
    Sources sources_;
    visualization::Displays displays_;
    Workspace workspace_;
    std::map<std::string, Connection> connections_;
    std::map<std::string, std::string> errors_;
};
// Atomic replacement on the supported POSIX platform. Failed writes preserve the old workspace.
void saveWorkspace(const Workspace &workspace, const std::filesystem::path &destination);
} // namespace robotics::viewer
