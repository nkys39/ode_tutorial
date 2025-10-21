#ifndef DRAWSTUFF_VIEWER_H
#define DRAWSTUFF_VIEWER_H

#include <drawstuff/drawstuff.h>
#include <ode/ode.h>
#include <functional>

namespace ode_tutorial {

// Simple wrapper for DrawStuff visualization
class DrawStuffViewer {
public:
    DrawStuffViewer() = default;
    ~DrawStuffViewer() = default;

    // Set callbacks
    void setSimulationCallback(std::function<void(double)> callback) {
        simCallback = callback;
    }

    void setDrawCallback(std::function<void()> callback) {
        drawCallback = callback;
    }

    void setCommandCallback(std::function<void(int)> callback) {
        commandCallback = callback;
    }

    // Start simulation
    void start(int argc, char** argv);

    // Static helper functions
    static void setColor(float r, float g, float b, float alpha = 1.0f);
    static void drawBox(const dReal* pos, const dReal* R, const dReal sides[3]);
    static void drawSphere(const dReal* pos, const dReal* R, dReal radius);
    static void drawCapsule(const dReal* pos, const dReal* R, dReal length, dReal radius);
    static void drawCylinder(const dReal* pos, const dReal* R, dReal length, dReal radius);

private:
    // Static callbacks for DrawStuff
    static void startCallback();
    static void stepCallback(int pause);
    static void commandCallback_static(int cmd);

    // User callbacks
    static std::function<void(double)> simCallback;
    static std::function<void()> drawCallback;
    static std::function<void(int)> commandCallback;

    static double lastTime;
};

} // namespace ode_tutorial

#endif // DRAWSTUFF_VIEWER_H
