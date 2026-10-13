#include <glad/glad.h>   // must come BEFORE glfw3.h
#include <GLFW/glfw3.h>
#include <iostream>
#include <string>
#include <vector>
#include <cstdlib> // for alloca
#include <cmath> // for sin and cos
#include "Shader.h"
#include "GraphicsObData.h"

//Screen Consts
const float WORLD_HALF_HEIGHT = 30.0f;
const int SCREEN_WIDTH = 800;
const int SCREEN_HEIGHT = 600;


//Physics Consts
static constexpr double G = 1.0;
static constexpr double softening = 0.01;
static constexpr double FIXED_DT = (1.0/120.0);
static constexpr double MAX_FRAME_TIME = 0.25;





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


class SimLogic 
{
    public:
        SimLogic() : CurrentTime(glfwGetTime()), LastTime(CurrentTime) {TimeCounter = 0.0; deltaTime = 0.0; LastFullSecond = false; totalframe = 0; tempframe = 0; fps = 0;}
        
        
        //Time Keeping
        void Chrono() 
        {
            CurrentTime = glfwGetTime();
            deltaTime = CurrentTime - LastTime;
            TimeCounter += deltaTime; 
            totalframe++;
            LastTime = CurrentTime;

            double interval = TimeCounter - TempTime;
            if (interval >= 1.0) {
                fps = (totalframe - tempframe) / interval;   // frames per real second
                tempframe = totalframe;
                TempTime = TimeCounter;
                LastFullSecond = true;
            } else {
                LastFullSecond = false;
            }
        }
        void ResetChrono() 
        {
            CurrentTime = glfwGetTime();
            LastTime = CurrentTime;
            TimeCounter = 0.0;
        }
        double getTime()
        {
            if(LastFullSecond){return TimeCounter;}
            else{return 0.0;}
        }
        //Frame Counting/Info
        int getFrame(){return totalframe;}

        int getFrameRate(){return fps;}
        
        struct Body 
        {
            float x, y, vx, vy, mass, radius;
            float color[3];
            Shape shape = Shape::Circle;
        };
        void pushBody()
        {

        }
        void InitBody(float in_x, float in_y, float in_mass, float in_radius, float in_r, float in_g, float in_b, float in_vx = 0.0, float in_vy = 0.0)
        {
            Body body{};

            body.x = in_x;
            body.y = in_y;
            body.vx = in_vx;
            body.vy = in_vy;
            body.mass = in_mass;
            body.radius = in_radius;
            body.color[0] = in_r;
            body.color[1] = in_g;
            body.color[2] = in_b;

            bodies.push_back(body);
        }

        const std::vector<Body>& getBodies() const {return bodies;}

        struct Shuttle 
        {
            float x, y, vx, vy, mass;
        };
        
        void Update()
        {
            double frametime = deltaTime;
            if (frametime > MAX_FRAME_TIME) frametime = MAX_FRAME_TIME;   // spiral-of-death guard
            accumulator += frametime * timeScale;                      // 0 = paused

            while (accumulator >= FIXED_DT) {
            Step(FIXED_DT);
            accumulator -= FIXED_DT;
            }
        }

        void ZeroMomentum()                // public: call once after adding bodies
        {
            double px = 0, py = 0, m = 0;
            for (const auto& b : bodies) { px += b.mass * b.vx; py += b.mass * b.vy; m += b.mass; }
            for (auto& b : bodies)       { b.vx -= px / m;      b.vy -= py / m; }
        }
        

    private:
        
        
        double CurrentTime, LastTime, deltaTime, TimeCounter;
        bool LastFullSecond;
        double TempTime = 0.0;
        //frames iterator
        int totalframe, tempframe, fps;
        
        void Step(double dt)
        {
            const size_t n = bodies.size();
            std::vector<double> ax(n, 0.0), ay(n, 0.0);       // zero-filled

            // Pass 1: accelerations from ONE frozen snapshot of positions
            for (size_t i = 0; i < n; ++i) {
                for (size_t j = 0; j < n; ++j) {
                    if (i == j) continue;
                    double dx  = bodies[j].x - bodies[i].x;   // points toward j
                    double dy  = bodies[j].y - bodies[i].y;
                    double d2  = dx*dx + dy*dy + softening;
                    double inv = 1.0 / (d2 * std::sqrt(d2));  // 1 / r^3 (softened)
                    ax[i] += G * bodies[j].mass * dx * inv;
                    ay[i] += G * bodies[j].mass * dy * inv;
                }
            }

            // Pass 2: semi-implicit Euler (velocity first, then position)
            for (size_t i = 0; i < n; ++i) {
                bodies[i].vx += ax[i] * dt;
                bodies[i].vy += ay[i] * dt;
                bodies[i].x  += bodies[i].vx * dt;
                bodies[i].y  += bodies[i].vy * dt;
            }
        }
        std::vector<Body> bodies;
        double accumulator = 0.0;
        double timeScale = 1.0;
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

        for (const auto& b : sim.getBodies()) {
            glUniform1f(scaleLoc,  (float)b.radius);
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