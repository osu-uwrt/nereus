#include <robotics/viewer/session.hpp>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <system_error>
#include <unistd.h>

namespace robotics::viewer {
void saveWorkspace(const Workspace &workspace, const std::filesystem::path &destination) {
    const auto text = serializeWorkspace(workspace, destination);
    std::string temporary = destination.string() + ".XXXXXX";
    int descriptor = mkstemp(temporary.data());
    if (descriptor < 0)
        throw std::system_error(errno, std::generic_category(), "create workspace");
    try {
        std::size_t written = 0;
        while (written < text.size()) {
            const auto count = write(descriptor, text.data() + written, text.size() - written);
            if (count < 0 && errno == EINTR)
                continue;
            if (count <= 0)
                throw std::system_error(errno, std::generic_category(), "write workspace");
            written += static_cast<std::size_t>(count);
        }
        if (fsync(descriptor) != 0)
            throw std::system_error(errno, std::generic_category(), "sync workspace");
        const int closed = close(descriptor);
        descriptor = -1;
        if (closed != 0)
            throw std::system_error(errno, std::generic_category(), "close workspace");
        if (std::rename(temporary.c_str(), destination.c_str()) != 0)
            throw std::system_error(errno, std::generic_category(), "replace workspace");
    } catch (...) {
        if (descriptor >= 0)
            close(descriptor);
        std::remove(temporary.c_str());
        throw;
    }
}
} // namespace robotics::viewer
