#include <glad/glad.h>   // must come BEFORE glfw3.h
#include <GLFW/glfw3.h>
#include <iostream>
#include <string>
#include <vector>
#include <cstdlib> // for alloca
#include <cmath> // for sin and cos
#include "Shader.h"
#include "GraphicsObData.h"

const float WORLD_HALF_HEIGHT = 1.0f;

struct WindowState {
    int width;
    int height; 
    bool dirty = true; // Flag to indicate if the window state has changed
};

const int SCREEN_WIDTH = 800;
const int SCREEN_HEIGHT = 600;




// Callback function to adjust the viewport when the window is resized
void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
    auto* state = static_cast<WindowState*>(glfwGetWindowUserPointer(window));
    state -> width = width;
    state -> height = height;
    state -> dirty = true; // Mark the window state as dirty
}



struct Planet 
{
    float radius;
    //float mass;
};



int main() {

    if (!glfwInit()) return -1;

    // Ask for the same OpenGL version you picked on the GLAD site
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);


    // Create a windowed mode window and its OpenGL context
    GLFWwindow* window = glfwCreateWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "AWL Cosmos", nullptr, nullptr);
    WindowState windowState{SCREEN_WIDTH, SCREEN_HEIGHT};
    glfwSetWindowUserPointer(window, &windowState);
    glfwGetFramebufferSize(window, &windowState.width, &windowState.height);

    if (!window) 
    { 
        glfwTerminate(); return -1; 
    }

    // Make the window's context current
    glfwMakeContextCurrent(window);

    // Load OpenGL functions. This has to happen AFTER making the context current.
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD\n";
        return -1;
    }


    // Set the viewport to the size of the window
    glViewport(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    //check versions
    //std::cout << "OpenGL " << glGetString(GL_VERSION) << "\n";
    //std::cout << "GLFW " << glfwGetVersionString() << "\n";


    {   // scope: gfx must be destroyed BEFORE glfwTerminate (its destructor calls glDelete*)
    //Graphics Objects Data
    GOD gfx;
    gfx.addCircle(36);
    gfx.addTriangle();
    gfx.upload();


    std::string vertexShader = ReadFile("shaders/basic.vert");
    std::string fragmentShader = ReadFile("shaders/basic.frag");

    unsigned int shader = CreateShader(vertexShader, fragmentShader);
    glUseProgram(shader);
    GLint halfExtentLoc = glGetUniformLocation(shader, "uHalfExtent");
    GLint offsetLoc = glGetUniformLocation(shader, "uOffset");
    GLint scaleLoc = glGetUniformLocation(shader, "uScale");
    GLint colorLoc = glGetUniformLocation(shader, "uColor");

    //gameloop
    while (!glfwWindowShouldClose(window)){
        if (windowState.dirty && windowState.height > 0) {
            float aspect = (float)windowState.width / (float)windowState.height;
            glUniform2f(halfExtentLoc, WORLD_HALF_HEIGHT * aspect, WORLD_HALF_HEIGHT);
            windowState.dirty = false;
        }
        
        glClearColor(0.12f, 0.13f, 0.13f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        //glDrawArrays(GL_TRIANGLES, 0, 3);
        // Planet: big, left of center
        glUniform2f(offsetLoc, -0.4f, 0.0f);
        glUniform1f(scaleLoc, 0.35f);
        glUniform3f(colorLoc, 0.25f, 0.55f, 0.85f);
        gfx.draw(Shape::Circle);

        // Moon: small, right
        glUniform2f(offsetLoc, 0.5f, 0.2f);
        glUniform1f(scaleLoc, 0.1f);
        glUniform3f(colorLoc, 0.75f, 0.75f, 0.78f);
        gfx.draw(Shape::Circle);
        
        // Shuttle: small, bottom
        glUniform2f(offsetLoc, 0.0f, -0.5f);
        glUniform1f(scaleLoc, 0.1f);
        glUniform3f(colorLoc, 0.44f, 1.0f, 0.16f);
        gfx.draw(Shape::Triangle);

        glfwSwapBuffers(window);

        glfwPollEvents();

    }
    }   // end of gfx scope
    glfwTerminate();
    return 0;
}