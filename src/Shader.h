#pragma once
#include <string>

unsigned int CompileShader(unsigned int type, const std::string& source);
unsigned int CreateShader(const std::string& vertexShader, const std::string& fragmentShader);
std::string ReadFile(const std::string& path);

// Inserts "#define NAME VALUE" right after the "#version" line, then "#line 2" so GLSL error line numbers
// still match the file on disk. Returns the source unchanged if there is no #version line.
std::string InjectDefine(const std::string& source, const std::string& name, const std::string& value);
