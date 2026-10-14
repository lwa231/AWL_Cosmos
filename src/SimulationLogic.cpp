#include "SimulationLogic.h"
#include <GLFW/glfw3.h>
#include <vector>

SimLogic::SimLogic()  : CurrentTime(glfwGetTime()), LastTime(CurrentTime) {TimeCounter = 0.0; deltaTime = 0.0; LastFullSecond = false; totalframe = 0; tempframe = 0; fps = 0;}

void SimLogic::Chrono()
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

void SimLogic::ResetChrono()
{
    CurrentTime = glfwGetTime();
    LastTime = CurrentTime;
    TimeCounter = 0.0;
}

void SimLogic::InitBody(double in_x, double in_y, float in_mass, float in_radius, BodyKind in_kind, float in_r, float in_g, float in_b, double in_vx, double in_vy)
{
    Body body{};

    body.x = in_x;
    body.y = in_y;
    body.vx = in_vx;
    body.vy = in_vy;
    body.mass = in_mass;
    body.radius = in_radius;
    body.kind = in_kind;
    body.color[0] = in_r;
    body.color[1] = in_g;
    body.color[2] = in_b;

    bodies.push_back(body);
}

void SimLogic::Update()
{
    double frametime = deltaTime;
    if (frametime > MAX_FRAME_TIME) frametime = MAX_FRAME_TIME;   // spiral-of-death guard
    accumulator += frametime * timeScale;                      // 0 = paused

    while (accumulator >= FIXED_DT) {
    Step(FIXED_DT);
    accumulator -= FIXED_DT;
    }
}

void SimLogic::ZeroMomentum()
{
    double px = 0, py = 0, m = 0;
    for (const auto& b : bodies) 
    { 
        px += b.mass * b.vx; 
        py += b.mass * b.vy; 
        m += b.mass; 
    }
    for (auto& b : bodies)       
    { 
        if(m == 0.0){break;}
        b.vx -= px / m;
        b.vy -= py / m; 
    }
}