#pragma once
#include <vector>
#include <cmath>
#include "GraphicsObData.h"

//Physics Consts
static constexpr double G = 1.0;
static constexpr double softening = 0.01;
static constexpr double FIXED_DT = (1.0/120.0);
static constexpr double MAX_FRAME_TIME = 0.25;

// What a body is. Rendering behavior is looked up per kind below.
enum class BodyKind { Star, Planet, Moon, Shuttle };

struct KindProps {
    bool  emitsLight;      // acts as a light source for other bodies
    bool  receivesLight;   // gets shaded by the lights (false = drawn flat/bright)
    float ambient;         // faint light on the night side
};

// Hardcoded behavior per kind: edit here when adding new kinds
inline KindProps kindProps(BodyKind k)
{
    switch (k) {
        case BodyKind::Star:    return { true,  false, 0.00f };
        case BodyKind::Planet:  return { false, true,  0.04f };
        case BodyKind::Moon:    return { false, true,  0.02f };
        case BodyKind::Shuttle: return { false, true,  0.10f };
    }
    return { false, true, 0.04f };
}

class SimLogic 
{
    public:
        SimLogic();
        
        
        //Time Keeping
        void Chrono(); 
        void ResetChrono(); 
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
            double x, y, vx, vy, mass, radius;
            float color[3];
            Shape shape = Shape::Circle;
            BodyKind kind = BodyKind::Planet;
        };
        void pushBody()
        {

        }
        void InitBody(double in_x, double in_y, float in_mass, float in_radius, BodyKind in_kind, float in_r, float in_g, float in_b, double in_vx = 0.0, double in_vy = 0.0);
        const std::vector<Body>& getBodies() const {return bodies;}


        struct Shuttle 
        {
            float x, y, vx, vy, mass;
        };
        

        void Update();
        void ZeroMomentum();                // public: call once after adding bodies
        
        

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