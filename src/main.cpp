#include <glad/glad.h>   // must come BEFORE glfw3.h
#include <GLFW/glfw3.h>
#include <iostream>
#include <string>
#include <vector>
#include <cstdlib> // for alloca
#include "Shader.h"
#include "GraphicsObData.h"
#include "SimulationLogic.h"

//Screen Consts
const float WORLD_HALF_HEIGHT = 30.0f;
const int SCREEN_WIDTH = 800;
const int SCREEN_HEIGHT = 600;


struct WindowState {
    int width;
    int height; 
    bool dirty = true; // Flag to indicate if the window state has changed
};




// Callback function to adjust the viewport when the window is resized
void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
    auto* state = static_cast<WindowState*>(glfwGetWindowUserPointer(window));
    state -> width = width;
    state -> height = height;
    state -> dirty = true; // Mark the window state as dirty
}





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
    
    if (!window) 
    { 
        glfwTerminate(); return -1; 
    }
    
    glfwSetWindowUserPointer(window, &windowState);
    glfwGetFramebufferSize(window, &windowState.width, &windowState.height);

    

    // Make the window's context current
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // Load OpenGL functions. This has to happen AFTER making the context current.
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD\n";
        return -1;
    }


    // Set the viewport to the size of the window
    glViewport(0, 0, windowState.width, windowState.height);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    //check versions
    //std::cout << "OpenGL " << glGetString(GL_VERSION) << "\n";
    //std::cout << "GLFW " << glfwGetVersionString() << "\n";


    {   // scope: gfx must be destroyed BEFORE glfwTerminate (its destructor calls glDelete*)
    SimLogic sim;
    //Graphics Objects Data
    GOD gfx;
    gfx.addCircle(36);
    //gfx.addTriangle();
    gfx.upload();


    std::string vertexShader = ReadFile("shaders/basic.vert");
    std::string fragmentShader = ReadFile("shaders/basic.frag");
    
    unsigned int shader = CreateShader(vertexShader, fragmentShader);
    
    if (vertexShader.empty() || fragmentShader.empty()) {
        std::cerr << "Shader files are empty, ABORTING." << std::endl;
        glfwTerminate();
        return -1;
    }
    
    glUseProgram(shader);
    GLint halfExtentLoc = glGetUniformLocation(shader, "uHalfExtent");
    if (halfExtentLoc == -1) {
        std::cerr << "Failed to get uniform location for uHalfExtent" << std::endl;
    }
    GLint offsetLoc = glGetUniformLocation(shader, "uOffset");
    if (offsetLoc == -1) {
        std::cerr << "Failed to get uniform location for uOffset" << std::endl;
    }
    GLint scaleLoc = glGetUniformLocation(shader, "uScale");
    if (scaleLoc == -1) {
        std::cerr << "Failed to get uniform location for uScale" << std::endl;
    }
    GLint colorLoc = glGetUniformLocation(shader, "uColor");
    if (colorLoc == -1) {
        std::cerr << "Failed to get uniform location for uColor" << std::endl;
    }
    GLint starPosLoc = glGetUniformLocation(shader, "uStarPos");
    if (starPosLoc == -1) {
        std::cerr << "Failed to get uniform location for starPosLoc" << std::endl;
    }
    GLint litLoc     = glGetUniformLocation(shader, "uLit");
    if (litLoc == -1){
        std::cerr << "Failed to get uniform location for litLoc" << std::endl;
    }

    
    sim.ResetChrono();

    //Setup Objects on the screen
    //           x,    y,  mass,radius,r,   g,     b,    vx, vy
    sim.InitBody(0,    0,  1000, 2.0, 1.0f, 0.85f, 0.35f, 0, 0);   // star
    sim.InitBody(20,   0,  50,   0.8, 0.25f,0.55f, 0.85f, 0, 7.25);// planet
    sim.InitBody(21.5, 0,  1,  0.25,0.75f,0.75f, 0.78f, 0, 7.25 + 5.80); // moon
    sim.ZeroMomentum();



    //gameloop
    while (!glfwWindowShouldClose(window)){
        if (windowState.dirty && windowState.height > 0) {
            float aspect = (float)windowState.width / (float)windowState.height;
            glUniform2f(halfExtentLoc, WORLD_HALF_HEIGHT * aspect, WORLD_HALF_HEIGHT);
            windowState.dirty = false;
        }
        
        //Keep Time
        sim.Chrono();
        sim.Update();

        if((sim.getTime()) != 0.0) {std::cout << static_cast<int>(sim.getTime()) << "  |  " << sim.getFrame() << "  |  " << sim.getFrameRate() << std::endl;}

        glClearColor(0.12f, 0.13f, 0.13f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        
        
        const auto& bodies = sim.getBodies();
        glUniform2f(starPosLoc, (float)bodies[0].x, (float)bodies[0].y);
        
        for (size_t i = 0; i < bodies.size(); ++i) {
            const auto& b = bodies[i];
            glUniform1f(litLoc, i == 0 ? 0.0f : 1.0f);
            glUniform1f(scaleLoc, (float)b.radius);
            glUniform2f(offsetLoc, (float)b.x, (float)b.y);
            glUniform3f(colorLoc,  b.color[0], b.color[1], b.color[2]);
            gfx.draw(b.shape);
        }

        glfwSwapBuffers(window);

        glfwPollEvents();

    }
    }   // end of gfx scope
    glfwTerminate();
    return 0;
}