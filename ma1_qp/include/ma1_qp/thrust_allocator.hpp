#ifndef THRUST_ALLOCATOR_HPP
#define THRUST_ALLOCATOR_HPP

#include <vector>

struct ThrusterCommand {
    double fx;       // Force component in x (surge)
    double fy;       // Force component in y (sway)
    double magnitude;
    double azimuth;  // in radians
};

class ThrustAllocator {
public:
    // vesselLength: overall vessel length,
    // vesselWidth: overall vessel width,
    // maxThrust: maximum thrust per component
    ThrustAllocator(double maxThrust);

    // Allocate thrust commands given desired surge (Fx), sway (Fy) and yaw moment (Mz)
    std::vector<ThrusterCommand> allocate(double Fx, double Fy, double Mz);

private:
    double L;
    double W;
    double maxThrust;

    // Thruster position relative to vessel center.
    // Order: Thruster 1: top left, Thruster 2: top right,
    //        Thruster 3: bottom left, Thruster 4: bottom right.
    struct Thruster {
        double x;
        double y;
    };
    std::vector<Thruster> thrusters;
};

#endif // THRUST_ALLOCATOR_HPP
