#include "render/gl_shader.h"
#include <glad/glad.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

namespace cosmos {
namespace render {

namespace {

bool readFile(const std::string& path, std::string& out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    std::stringstream ss;
    ss << f.rdbuf();
    out = ss.str();
    return true;
}

// Recursively expands #include "name". `#line n fileIndex` keeps compiler messages pointing at the right file/line.
bool expand(const std::string& dir, const std::string& file, int depth, std::vector<std::string>& files, std::string& out)
{
    if (depth > 8) {
        std::cerr << "shader: #include nesting too deep at " << file << "\n";
        return false;
    }
    std::string text;
    if (!readFile(dir + "/" + file, text)) {
        std::cerr << "shader: cannot open " << dir << "/" << file << "\n";
        return false;
    }
    const int fileIndex = static_cast<int>(files.size());
    files.push_back(file);
    if (depth > 0) out += "#line 1 " + std::to_string(fileIndex) + "\n";   // #version must stay the first line of the top file

    std::istringstream in(text);
    std::string line;
    int lineNo = 0;
    while (std::getline(in, line)) {
        ++lineNo;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        const std::size_t p = line.find("#include");
        if (p != std::string::npos && line.find_first_not_of(" \t") == p) {
            const std::size_t q1 = line.find('"', p);
            const std::size_t q2 = q1 == std::string::npos ? q1 : line.find('"', q1 + 1);
            if (q2 == std::string::npos) {
                std::cerr << "shader: malformed #include in " << file << ":" << lineNo << "\n";
                return false;
            }
            if (!expand(dir, line.substr(q1 + 1, q2 - q1 - 1), depth + 1, files, out)) return false;
            out += "#line " + std::to_string(lineNo + 1) + " " + std::to_string(fileIndex) + "\n";
        } else {
            out += line;
            out += '\n';
        }
    }
    return true;
}

unsigned int compile(unsigned int type, const std::string& source, const std::string& name, const std::vector<std::string>& files)
{
    const unsigned int id = glCreateShader(type);
    const char* src = source.c_str();
    glShaderSource(id, 1, &src, nullptr);
    glCompileShader(id);

    int ok = 0;
    glGetShaderiv(id, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        int length = 0;
        glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length);
        std::string log(static_cast<std::size_t>(length > 0 ? length : 1), '\0');
        glGetShaderInfoLog(id, length, &length, &log[0]);
        std::cerr << "Failed to compile " << name << " (source file indices:";
        for (std::size_t i = 0; i < files.size(); ++i) std::cerr << " " << i << "=" << files[i];
        std::cerr << ")\n" << log << "\n";
        glDeleteShader(id);
        return 0;
    }
    return id;
}

unsigned int load(unsigned int type, const std::string& dir, const std::string& file)
{
    std::vector<std::string> files;
    std::string source;
    if (!expand(dir, file, 0, files, source)) return 0;
    return compile(type, source, file, files);
}

} // namespace

unsigned int buildProgram(const std::string& dir, const std::string& vertFile, const std::string& fragFile)
{
    const unsigned int vs = load(GL_VERTEX_SHADER, dir, vertFile);
    const unsigned int fs = load(GL_FRAGMENT_SHADER, dir, fragFile);
    if (!vs || !fs) {
        if (vs) glDeleteShader(vs);
        if (fs) glDeleteShader(fs);
        return 0;
    }

    unsigned int program = glCreateProgram();
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);
    int ok = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[2048];
        glGetProgramInfoLog(program, sizeof(log), nullptr, log);
        std::cerr << "Shader link failed (" << vertFile << " + " << fragFile << "):\n" << log << "\n";
        glDeleteProgram(program);
        program = 0;
    }
    glDeleteShader(vs);
    glDeleteShader(fs);
    return program;
}

unsigned int buildComputeProgram(const std::string& dir, const std::string& file)
{
    const unsigned int cs = load(GL_COMPUTE_SHADER, dir, file);
    if (!cs) return 0;
    unsigned int program = glCreateProgram();
    glAttachShader(program, cs);
    glLinkProgram(program);
    int ok = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[2048];
        glGetProgramInfoLog(program, sizeof(log), nullptr, log);
        std::cerr << "Shader link failed (" << file << "):\n" << log << "\n";
        glDeleteProgram(program);
        program = 0;
    }
    glDeleteShader(cs);
    return program;
}

} // namespace render
} // namespace cosmos
