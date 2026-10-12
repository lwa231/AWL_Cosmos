#pragma once
#include <string>

unsigned int CompileShader(unsigned int type, const std::string& source);
int CreateShader(const std::string& vertexShader, const std::string& fragmentShader);
std::string ReadFile(const std::string& path);
