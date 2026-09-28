#include <robotics/rendering/lines.hpp>

#include <GL/glew.h>
#include <array>
#include <stdexcept>

namespace robotics::rendering {
namespace {
GLuint shader(GLenum type, const char *source) {
    const GLuint result = glCreateShader(type);
    glShaderSource(result, 1, &source, nullptr);
    glCompileShader(result);
    GLint ok = 0;
    glGetShaderiv(result, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        std::array<char, 1024> log{};
        glGetShaderInfoLog(result, static_cast<GLsizei>(log.size()), nullptr, log.data());
        glDeleteShader(result);
        throw std::runtime_error(std::string("line shader: ") + log.data());
    }
    return result;
}
} // namespace
struct Lines::Resources {
    GLuint vertex{0}, fragment{0}, program{0}, array{0}, buffer{0};
    ~Resources() {
        if (buffer)
            glDeleteBuffers(1, &buffer);
        if (array)
            glDeleteVertexArrays(1, &array);
        if (program)
            glDeleteProgram(program);
        if (vertex)
            glDeleteShader(vertex);
        if (fragment)
            glDeleteShader(fragment);
    }
};
Lines::Lines() : resources_(std::make_unique<Resources>()) {
    if (!GLEW_VERSION_3_3)
        throw std::runtime_error("line rendering requires initialized GLEW and OpenGL 3.3");
    auto &r = *resources_;
    r.vertex = shader(GL_VERTEX_SHADER, R"(#version 330 core
layout(location=0) in vec3 position;
layout(location=1) in vec3 color;
uniform mat4 view_projection;
out vec3 vertex_color;
void main() { gl_Position = view_projection * vec4(position, 1); vertex_color = color; }
)");
    r.fragment = shader(GL_FRAGMENT_SHADER, R"(#version 330 core
in vec3 vertex_color;
out vec4 output_color;
void main() { output_color = vec4(vertex_color, 1); }
)");
    r.program = glCreateProgram();
    glAttachShader(r.program, r.vertex);
    glAttachShader(r.program, r.fragment);
    glLinkProgram(r.program);
    GLint ok = 0;
    glGetProgramiv(r.program, GL_LINK_STATUS, &ok);
    if (!ok)
        throw std::runtime_error("line shader link failed");
    glGenVertexArrays(1, &r.array);
    glGenBuffers(1, &r.buffer);
}
Lines::~Lines() = default;
void Lines::draw(const std::vector<visualization::Line> &lines, const Eigen::Matrix4f &matrix) {
    if (lines.size() > 200000 || !matrix.allFinite())
        throw std::invalid_argument("invalid line scene");
    std::vector<float> vertices;
    vertices.reserve(lines.size() * 12);
    for (const auto &line : lines) {
        for (const auto &point : {line.from, line.to}) {
            if (!point.allFinite() || point.cwiseAbs().maxCoeff() > 1e14)
                throw std::invalid_argument("invalid line coordinates");
            for (int i = 0; i < 3; ++i)
                vertices.push_back(static_cast<float>(point[i]));
            for (int color : line.color) {
                if (color < 0 || color > 255)
                    throw std::invalid_argument("invalid line color");
                vertices.push_back(static_cast<float>(color) / 255.0F);
            }
        }
    }
    const auto &r = *resources_;
    glBindVertexArray(r.array);
    glBindBuffer(GL_ARRAY_BUFFER, r.buffer);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(float)),
                 vertices.data(), GL_STREAM_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          reinterpret_cast<void *>(3 * sizeof(float)));
    glUseProgram(r.program);
    glUniformMatrix4fv(glGetUniformLocation(r.program, "view_projection"), 1, GL_FALSE,
                       matrix.data());
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDisable(GL_BLEND);
    glLineWidth(1.0F);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(lines.size() * 2));
    glBindVertexArray(0);
    glUseProgram(0);
}
} // namespace robotics::rendering
