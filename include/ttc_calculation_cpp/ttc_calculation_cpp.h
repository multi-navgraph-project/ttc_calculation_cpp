#include <iostream>
#include <cmath>
#include <limits>
#include <stdexcept>

using namespace std;

class TTCComputer
{
public:
    struct Point
    {
        double x, y;
    };

    struct Heading
    {
        double hx, hy;
    };

    struct Vehicle
    {
        Point position;
        Point velocity;
        Heading heading;
        double length;
    };

    struct Sample
    {
        Vehicle vehicle_i;
        Vehicle vehicle_j;
    };

    // Compute the heading based on velocity
    // static Heading computeHeading(const Point& velocity) {
    //    double magnitude = sqrt(pow(velocity.x, 2) + pow(velocity.y, 2));
    //    return Heading{velocity.x / magnitude, velocity.y / magnitude};
    //}

    // Compute distance to overlap considering heading
    static double computeDist2Overlap(const Sample &sample)
    {
        // Extract vehicle data
        Vehicle vehicle_i = sample.vehicle_i;
        Vehicle vehicle_j = sample.vehicle_j;

        // Compute relative velocity
        double rel_vx = vehicle_j.velocity.x - vehicle_i.velocity.x;
        double rel_vy = vehicle_j.velocity.y - vehicle_i.velocity.y;

        // Compute dot product of relative velocity and heading of each vehicle
        double dot_i = rel_vx * vehicle_i.heading.hx + rel_vy * vehicle_i.heading.hy;
        double dot_j = rel_vx * vehicle_j.heading.hx + rel_vy * vehicle_j.heading.hy;

        // Compute front point based on dot product
        Point front_i, front_j;

        front_i.x = vehicle_i.position.x + (dot_i >= 0 ? 1 : -1) * vehicle_i.heading.hx * vehicle_i.length / 2;
        front_i.y = vehicle_i.position.y + (dot_i >= 0 ? 1 : -1) * vehicle_i.heading.hy * vehicle_i.length / 2;

        front_j.x = vehicle_j.position.x + (dot_j >= 0 ? 1 : -1) * vehicle_j.heading.hx * vehicle_j.length / 2;
        front_j.y = vehicle_j.position.y + (dot_j >= 0 ? 1 : -1) * vehicle_j.heading.hy * vehicle_j.length / 2;

        // Now compute distance and overlap as before
        double dx = front_i.x - front_j.x;
        double dy = front_i.y - front_j.y;
        double distance = sqrt(dx * dx + dy * dy);
        double overlap = distance - (vehicle_i.length / 2) - (vehicle_j.length / 2);

        cout << "overlap : " << overlap << "\n";
        cout << "distance : " << distance << "\n";

        return (overlap > 0) ? overlap : numeric_limits<double>::infinity();
    }

    // Compute TTC
    static double computeTTC(const Sample &sample)
    {
        // Compute distance to overlap
        double dist2overlap = computeDist2Overlap(sample);

        // Compute relative velocity
        Point direct_v = {sample.vehicle_i.velocity.x - sample.vehicle_j.velocity.x, sample.vehicle_i.velocity.y - sample.vehicle_j.velocity.y};
        // cout << "VITESSE : " << direct_v.x;
        //  Compute TTC
        double TTC = dist2overlap / sqrt(pow(direct_v.x, 2) + pow(direct_v.y, 2));

        return TTC;
    }
};