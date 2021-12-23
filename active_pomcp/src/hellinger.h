#pragma once
#include <algorithm>
#include <vector>
#include <array>
#include <math.h>
#include <iostream>

/*
 * Discrete Hellinger distance between two distributions
 */
template<size_t N>
static double hellinger(const std::array<double, N> p, const std::array<double, N> q) {
    double sum = 0.0;
    for (int i = 0; i < p.size(); i++) {
        sum += std::pow(std::sqrt(p[i]) - std::sqrt(q[i]), 2);
    }
    return std::sqrt(0.5 * sum);
}

#if 0
/*
 * This class stores a collection of points and a threshold in [0, 1].
 * It is used to compute if the minimum hellinger distance between a point and
 * the collection is below the threshold.
 */
template<int N>
class hellinger_shield {
public:
     hellinger_shield() : threshold(1 - 0.1 * 0.1), pts() {
     }

     hellinger_shield(double c, const std::vector<std::array<double, N>> &p)
         : threshold(1 - c * c), pts() {
         set_points(p);
     }

     void set_points(const std::vector<std::array<double, N>> &points) {
         pts.clear();
         for (const auto  &p : points) {
             pts.push_back(p);
             for (double &d : pts.back())
                 d = std::sqrt(d);
         }
     }

     void set_threshold(double t) {
         threshold = 1 - t*t;
     }

     bool is_in_threshold(const std::array<double, N> q) const {
         for (int i = 0; i < q.size(); i++)
             sqrt_buff[i] = std::sqrt(q[i]);

         for (int i = 0; i < pts.size(); ++i) {
             double sum = 0.0;
             for (int j = 0; j < N; j++)
                 sum += sqrt_buff[j] * pts[i][j];

             if (sum >= threshold) {
                 return true;
             }
         }
         return false;
     }

private:
    double threshold;
    mutable std::array<double, N> sqrt_buff;

    // directly stored as sqrt(points)
    std::vector<std::array<double, N>> pts;
};
#endif

/*
 * DIRECT IMPLEMENTATION
 */
template<size_t N>
class hellinger_shield {
public:
     hellinger_shield() : threshold(0.1), pts() {
     }

     hellinger_shield(double c, const std::vector<std::array<double, N>> &p)
         : threshold(c), pts(p) {
     }

     void set_points(const std::vector<std::array<double, N>> &points) {
         pts = points;
     }

     void set_threshold(double t) {
         threshold = t;
     }

     bool is_in_threshold(const std::array<double, N> q) const {
         for (const auto &p : pts) {
             if (hellinger(p, q) <= threshold)
                 return true;
         }
         return false;
     }

private:
    double threshold;
    // directly stored as sqrt(points)
    std::vector<std::array<double, N>> pts;
};
