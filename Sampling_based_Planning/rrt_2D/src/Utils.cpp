#include <Eigen/Dense>
#include <cmath>
#include <utility>
#include "Env.h"
#include "Node.h"
#include "Utils.h"

Utils::Utils(Env &env) : env(env) {}

bool Utils::check_collision(Node &node) {
    auto in_cir = [&](const std::vector<int> &r) {
        return std::hypot(node.x - r[0], node.y - r[1]) <= r[2] + this->delta;
    };

    auto in_rect = [&](const std::vector<int> &r) {
        return node.x >= r[0] - this->delta && node.x <= r[0] + r[2] + this->delta &&
               node.y >= r[1] - this->delta && node.y <= r[1] + r[3] + this->delta;
    };

    for (const auto &r : this->env.obs_circle) {
        if (in_cir(r)) {
            return true;
        }
    }

    for (const auto &r : this->env.obs_rectangle) {
        if (in_rect(r)) {
            return true;
        }
    }

    for (const auto &r : this->env.obs_boundary) {
        if (in_rect(r)) {
            return true;
        }
    }

    return false;
}

std::pair<Eigen::Vector2d, Eigen::Vector2d> Utils::get_ray(Node &start, Node &end) {
    // o is the origin of the ray, d is the direction of the ray
    Eigen::Vector2d o(start.x, start.y);
    Eigen::Vector2d d(end.x - start.x, end.y - start.y);
    return {o, d};
}

double Utils::get_dist(Node &start, Node &end) {
    return std::hypot(end.x - start.x, end.y - start.y);
}
