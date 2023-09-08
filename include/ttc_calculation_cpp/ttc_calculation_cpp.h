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
        double width;
    };

    struct Sample
    {
        Vehicle vehicle_i;
        Vehicle vehicle_j;
    };

    // Helper function to compute dot product
    static double dotProduct(Point a, Point b)
    {
        return a.x * b.x + a.y * b.y;
    }

    // Helper function to compute the projection of a point onto an axis
    static double project(Point axis, Point point)
    {
        double dot = dotProduct(axis, point);
        double lenSq = dotProduct(axis, axis);
        return dot / lenSq;
    }

    // Helper function to compute distance between two points
    static double distance(Point a, Point b)
    {
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    }

    // Helper function to compute the minimum distance between two line segments defined by points a, b and c, d
    static double segmentToSegmentDistance(Point a, Point b, Point c, Point d)
    {
        Point ab = {b.x - a.x, b.y - a.y};
        Point cd = {d.x - c.x, d.y - c.y};
        Point ac = {c.x - a.x, c.y - a.y};

        double e = dotProduct(ab, ab);
        double f = dotProduct(ab, cd);
        double g = dotProduct(cd, cd);
        double h = dotProduct(ab, ac);
        double i = dotProduct(cd, ac);

        double det = e * g - f * f;

        double s, t;
        if (det != 0)
        {
            s = (g * h - f * i) / det;
            t = (e * i - f * h) / det;
        }
        else
        {
            s = t = 0.0;
        }

        s = max(min(s, 1.0), 0.0);
        t = max(min(t, 1.0), 0.0);

        Point closestPointOnAB = {a.x + s * ab.x, a.y + s * ab.y};
        Point closestPointOnCD = {c.x + t * cd.x, c.y + t * cd.y};

        return distance(closestPointOnAB, closestPointOnCD);
    }

    // Compute the corners of the oriented rectangle for a vehicle
    static vector<Point> computeCorners(const Vehicle &vehicle)
    {
        vector<Point> corners;
        double halfLength = vehicle.length / 2.0;
        double halfWidth = vehicle.width / 2.0;

        // Compute the four corners based on position, heading, length, and width
        for (int i = -1; i <= 1; i += 2)
        {
            for (int j = -1; j <= 1; j += 2)
            {
                Point corner;
                corner.x = vehicle.position.x + i * halfLength * vehicle.heading.hx + j * halfWidth * -vehicle.heading.hy;
                corner.y = vehicle.position.y + i * halfLength * vehicle.heading.hy + j * halfWidth * vehicle.heading.hx;
                corners.push_back(corner);
            }
        }

        return corners;
    }

    // Check for overlap using the Separating Axis Theorem (SAT)
    static bool checkOverlap(vector<Point> corners_i, vector<Point> corners_j)
    {
        // Define the axes based on the corners of the first rectangle
        vector<Point> axes;
        axes.push_back({corners_i[1].x - corners_i[0].x, corners_i[1].y - corners_i[0].y});
        axes.push_back({corners_i[1].x - corners_i[3].x, corners_i[1].y - corners_i[3].y});

        for (auto axis : axes)
        {
            double min_i = numeric_limits<double>::infinity();
            double max_i = -numeric_limits<double>::infinity();
            double min_j = numeric_limits<double>::infinity();
            double max_j = -numeric_limits<double>::infinity();

            // Project corners of both rectangles onto the axis and find the min and max projection
            for (auto corner : corners_i)
            {
                double projection = project(axis, corner);
                min_i = min(min_i, projection);
                max_i = max(max_i, projection);
            }

            for (auto corner : corners_j)
            {
                double projection = project(axis, corner);
                min_j = min(min_j, projection);
                max_j = max(max_j, projection);
            }

            // Check for overlap
            if (!(max_i >= min_j && max_j >= min_i))
            {
                return false;
            }
        }

        return true;
    }

    // Compute distance to overlap considering heading, length, and width
    static double computeDist2Overlap(const Sample &sample)
    {
        // Extract vehicle data
        Vehicle vehicle_i = sample.vehicle_i;
        Vehicle vehicle_j = sample.vehicle_j;

        // Compute the corners of the oriented rectangles for both vehicles
        vector<Point> corners_i = computeCorners(vehicle_i);
        vector<Point> corners_j = computeCorners(vehicle_j);

        // Check for overlap using SAT
        bool isOverlap = checkOverlap(corners_i, corners_j);

        // Compute distance to overlap based on overlap status
        if (isOverlap)
        {
            return 0;
        }
        else
        {
            // Compute the minimum distance between the edges of the two rectangles
            double minDist = numeric_limits<double>::infinity();
            for (int i = 0; i < 4; ++i)
            {
                for (int j = 0; j < 4; ++j)
                {
                    double dist = segmentToSegmentDistance(corners_i[i], corners_i[(i + 1) % 4], corners_j[j], corners_j[(j + 1) % 4]);
                    minDist = min(minDist, dist);
                }
            }

            return minDist;
        }
    }

    // Compute TTC
    static double computeTTC(const Sample &sample)
    {
        // Compute distance to overlap
        double dist2overlap = computeDist2Overlap(sample);

        // Compute relative velocity
        Point direct_v = {sample.vehicle_i.velocity.x - sample.vehicle_j.velocity.x, sample.vehicle_i.velocity.y - sample.vehicle_j.velocity.y};

        // Check if the vehicles are moving away from each other
        Point center_diff = {sample.vehicle_j.position.x - sample.vehicle_i.position.x, sample.vehicle_j.position.y - sample.vehicle_i.position.y};
        if (dotProduct(direct_v, center_diff) < 0)
        {
            return std::numeric_limits<double>::infinity();
        }

        // Compute TTC
        double TTC = dist2overlap / sqrt(pow(direct_v.x, 2) + pow(direct_v.y, 2));

        return TTC;
    }
};