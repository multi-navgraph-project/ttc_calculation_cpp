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

        // Compute distance to overlap considering heading
        Point front_i = {vehicle_i.position.x + vehicle_i.heading.hx * vehicle_i.length / 2, vehicle_i.position.y + vehicle_i.heading.hy * vehicle_i.length / 2};
        Point front_j = {vehicle_j.position.x + vehicle_j.heading.hx * vehicle_j.length / 2, vehicle_j.position.y + vehicle_j.heading.hy * vehicle_j.length / 2};

        double dx = front_i.x - front_j.x;
        double dy = front_i.y - front_j.y;
        double distance = sqrt(dx * dx + dy * dy);

        double overlap = distance;
        cout << "front_i.x : " << front_i.x << "front_i.y : " << front_i.y << "\n";
        cout << "front_j.x : " << front_j.x << "front_j.y : " << front_j.y << "\n";
        cout << "overlap : " << distance << "\n";

        return overlap;
    }

    // Compute TTC
    static double computeTTC(const Sample &sample)
    {
        // Compute distance to overlap
        double dist2overlap = computeDist2Overlap(sample);

        // Compute relative velocity
        Point direct_v = {sample.vehicle_i.velocity.x - sample.vehicle_j.velocity.x, sample.vehicle_i.velocity.y - sample.vehicle_j.velocity.y};
        cout << "VITESSE : " << direct_v.x;
        // Compute TTC
        double TTC;
        if (fabs(dist2overlap) < 0.2)
        {
            TTC = 0;
        }
        else
        {
            TTC = dist2overlap / sqrt(pow(direct_v.x, 2) + pow(direct_v.y, 2));
        }

        return fabs(TTC);
    }
};