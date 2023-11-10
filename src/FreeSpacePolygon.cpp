//
// Created by Luis Bracamontes on 10/24/23.
//

#include "FreeSpacePolygon.h"

std::vector<Point> FreeSpacePolygon::line(double angle, Point origin, double radius){
    std::vector<Point> points;
    for (double r = 1.0; r <= radius; r += 1.0) {
        int x = static_cast<int>(origin.x + r * cos((angle)));
        int y = static_cast<int>(origin.y + r * sin((angle)));
        points.push_back(Point(x, y));
    }
    return points;
}

std::vector<Point> FreeSpacePolygon::halfCircle(double angle, Point origin, double radius){
    std::vector<Point> points;
    double start_angle = angle - M_PI/2;
    double end_angle = angle + M_PI/2;
    for (int i = 0; i < HCnpts_; ++i) {
        double a = start_angle + (end_angle - start_angle) * i/HCnpts_;
        int x = static_cast<int>(origin.x + radius * cos((a)));
        int y = static_cast<int>(origin.y + radius * sin((a)));
        points.push_back(Point(x, y));
    }
    return points;
}

std::vector<Point> FreeSpacePolygon::lineWithBresenham(double angle, Point origin, double radius) {
    std::vector<Point> points;

    int x0 = origin.x;
    int y0 = origin.y;

    int x1 = static_cast<int>(origin.x + radius * cos(angle));
    int y1 = static_cast<int>(origin.y + radius * sin(angle));

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;

    int error = dx - dy;

    while (true) {
        points.push_back(Point(x0, y0));

        if (x0 == x1 && y0 == y1) {
            break;
        }

        int e2 = 2 * error;
        if (e2 > -dy) {
            error -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            error += dx;
            y0 += sy;
        }
    }

    return points;
}

std::vector<Point> FreeSpacePolygon::halfCircleWithBresenham(double angle, Point origin, double radius) {
    std::vector<Point> points;

    double start_angle = angle - M_PI / 2;
    double end_angle = angle + M_PI / 2;

    int x = static_cast<int>(origin.x + radius * cos(start_angle));
    int y = static_cast<int>(origin.y + radius * sin(start_angle));
    int dx = static_cast<int>(radius * cos(end_angle) - radius * cos(start_angle));
    int dy = static_cast<int>(radius * sin(end_angle) - radius * sin(start_angle));

    int d = 2 * dy - dx;
    int ystep = 2 * dy;
    int xstep = 2 * dx;

    points.push_back(Point(x, y));

    if (dx != 0) {
        for (int i = 0; i < dx; ++i) {
            x++;
            if (d >= 0) {
                d -= xstep;
                y += 1;
            }
            d += ystep;
            points.push_back(Point(x, y));
        }
    }

    return points;
}

std::vector<Point> FreeSpacePolygon::halfCircleShift(const std::vector<Point>& HCPts, int dx, int dy){
    std::vector<Point> points;
    for (const Point& p : HCPts) {
        int x = static_cast<int>(p.x+dx);
        int y = static_cast<int>(p.y+dy);
        points.push_back(Point(x, y));
    }
    return points;
}

Point FreeSpacePolygon::findCollisionPoint(const std::vector<Point>& Lk, std::vector<Point>& HCk, const cv::Mat& Pm){
    Point collisionPoint;
    Point pprev = Lk[0];

    bool found = false;

    for (const Point &p: Lk) {
        if (found) {
            break;
        }
        // deltas for shifting circle
        int dx = p.x - pprev.x;
        int dy = p.y - pprev.y;
        for (const Point &q: HCk) {
            int qx = static_cast<int>(q.x);
            int qy = static_cast<int>(q.y);
            // @TODO: OpenCV gets cols first ie y then x
            //Probability of occupancy if grid == 0 not occupied if grid == 255 occupied
            double mean_pix_value = (Pm.at<cv::Vec3b>(qy, qx)[0] + Pm.at<cv::Vec3b>(qy, qx)[1] +
                                       Pm.at<cv::Vec3b>(qy, qx)[2])/3.0;
            // opencv are reversed
            if (qx >= 0 && qx < Pm.cols && qy >= 0 && qy < Pm.rows &&
                (mean_pix_value / 255.0) < tau_obs_) {
                collisionPoint = q;  // collision cell
                found = true;
                break;
            }
        }

        HCk = halfCircleShift(HCk, dx, dy);  // shift HCk
        pprev = p;
    }
    if (!found){
        // last point in Lk and check boundaries
        collisionPoint = Lk.back();
        if (collisionPoint.x > Pm.rows)
            collisionPoint.x = Pm.rows;
        if (collisionPoint.y > Pm.cols)
            collisionPoint.y = Pm.cols;
    }
    return collisionPoint;
}

std::vector<Point> FreeSpacePolygon::calculateFreeSpacePolygon(Point po, double theta_o, const cv::Mat& Pm){
    std::vector<Point> P;
    //@TODO convert grid element to distance in m
    for (int k = 0; k < npts_; ++k) {
        double theta_k = theta_o + M_PI - (2.0 * M_PI * k)/npts_;
        std::vector<Point> Lk = lineWithBresenham(theta_k, po, rC_);
        std::vector<Point> HCk = halfCircle(theta_k, po, rHC_); //@TODO: half circle with bresenham needs checking

        // Find the collision point
        Point collisionPoint = findCollisionPoint(Lk, HCk, Pm);
        P.push_back(collisionPoint); // collision point
    }
    return P;
}