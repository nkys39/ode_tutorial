#ifndef VIEWER_H
#define VIEWER_H

#include <ode/ode.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <functional>

namespace ode_tutorial {

// Camera structure
struct Camera {
    float distance;
    float yaw;
    float pitch;
    float target_x;
    float target_y;
    float target_z;

    Camera() : distance(5.0f), yaw(45.0f), pitch(30.0f),
               target_x(0.0f), target_y(0.0f), target_z(0.5f) {}
};

// Viewer class for OpenGL visualization
class Viewer {
public:
    Viewer(int argc, char** argv, const char* title = "ODE Tutorial");
    ~Viewer();

    // Set simulation callback
    void setSimulationCallback(std::function<void(double)> callback);

    // Set drawing callback
    void setDrawCallback(std::function<void()> callback);

    // Set keyboard callback
    void setKeyboardCallback(std::function<void(unsigned char, int, int)> callback);

    // Start main loop
    void start();

    // Camera controls
    static Camera& getCamera();

    // Utility drawing functions
    static void drawBox(const dReal* pos, const dReal* R, const dReal sides[3]);
    static void drawSphere(const dReal* pos, const dReal* R, dReal radius);
    static void drawCapsule(const dReal* pos, const dReal* R, dReal length, dReal radius);
    static void drawCylinder(const dReal* pos, const dReal* R, dReal length, dReal radius);
    static void drawPlane(const dReal* normal, dReal distance, dReal size);
    static void drawLine(const dReal* start, const dReal* end, float r = 1.0f, float g = 1.0f, float b = 1.0f);
    static void drawGrid(float size, int divisions);
    static void drawAxes(float size);

    // Color utilities
    static void setColor(float r, float g, float b, float alpha = 1.0f);

    // Draw ODE geometry
    static void drawGeometry(dGeomID geom);
    static void drawBody(dBodyID body);

    // Simulation control
    static void pause();
    static void resume();
    static void togglePause();
    static bool isPaused();
    static void reset();
    static void setShouldReset(bool value);
    static bool shouldReset();

private:
    static void displayCallback();
    static void reshapeCallback(int width, int height);
    static void idleCallback();
    static void mouseCallback(int button, int state, int x, int y);
    static void motionCallback(int x, int y);
    static void keyboardCallbackInternal(unsigned char key, int x, int y);
    static void specialKeyCallback(int key, int x, int y);

    static std::function<void(double)> simCallback;
    static std::function<void()> drawCallback;
    static std::function<void(unsigned char, int, int)> keyCallback;

    static Camera camera;
    static int mouseButton;
    static int mouseX, mouseY;
    static bool paused;
    static bool resetFlag;
    static double lastTime;
    static int windowWidth;
    static int windowHeight;
};

} // namespace ode_tutorial

#endif // VIEWER_H
