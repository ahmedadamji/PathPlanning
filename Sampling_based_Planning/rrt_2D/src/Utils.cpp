#include <Eigen/Dense>
#include <cmath>
#include <utility>
#include "Env.h"
#include "Node.h"
#include "Utils.h"

// Note that the default constructor should not be defined using Class() {}, but Class() = default;
// Utils::Utils() = default;
Utils::Utils(Env &env) : env(env) {}

bool Utils::check_collision_with_obs(Node &node) {
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

std::vector<std::vector<Eigen::Vector2d>> Utils::get_obs_vertex()
{
    std::vector<std::vector<Eigen::Vector2d>> obs_vertex;
    for (const auto &r : this->env.obs_rectangle)
    {
        std::vector<Eigen::Vector2d> vertex_list;
        vertex_list.push_back(Eigen::Vector2d(r[0] - this->delta, r[1] - this->delta));
        vertex_list.push_back(Eigen::Vector2d(r[0] + r[2] + this->delta, r[1] - this->delta));
        vertex_list.push_back(Eigen::Vector2d(r[0] - this->delta, r[1] + r[3] + this->delta));
        vertex_list.push_back(Eigen::Vector2d(r[0] + r[2] + this->delta, r[1] + r[3] + this->delta));
        obs_vertex.push_back(vertex_list);

    }
    return obs_vertex;
}

bool Utils::is_intersect_rect(Node &start, Node &end, std::pair<Eigen::Vector2d, Eigen::Vector2d> ray, Eigen::Vector2d a, Eigen::Vector2d b)
{
    Eigen::Vector2d v1((ray.first[0] - a[0]), (ray.first[1] - a[1]));
    Eigen::Vector2d v2((b[0] - a[0]), (b[1] - a[1]));
    Eigen::Vector2d v3(-ray.second[1], ray.second[0]);
    double div = v2.dot(v3);
    if (div == 0) {
        return false;
    }
    // Compute the determinant (or "cross product" for 2D vectors)
    double determinant = v2[0]*v1[1] - v2[1]*v1[0];
    // The "norm" of the cross product for 2D vectors is actually just the absolute value of the determinant
    double norm = std::abs(determinant);
    // Compute the signed distance from a to b
    double t1 = norm / div;
    double t2 = v1.dot(v3) / div;

    if (t1 >= 0 && (t2 >= 0 && t2 <= 1)) {
        Node shot(std::make_pair((ray.first[0] + t1 * ray.second[0]), (ray.first[1] + t1 * ray.second[1])));
        double dist_obs = this->get_dist(start, shot);
        double dist_seg = this->get_dist(start, end);
        if (dist_obs < dist_seg) {
            return true;
        }
    }
    return false;
}

bool Utils::is_intersect_circle(std::pair<Eigen::Vector2d, Eigen::Vector2d> ray, Eigen::Vector2d a, double r)
{
    double d2 = ray.second.dot(ray.second);
    
    if (d2 == 0) {
        return false;
    }

    Eigen::Vector2d a_minus_o = a - ray.first;
    double t = a_minus_o.dot(ray.second) / d2;

    if (0 <= t && t <= 1) {
        Node shot(std::make_pair((ray.first[0] + t * ray.second[0]), (ray.first[1] + t * ray.second[1])));
        Node a_node(std::make_pair(a[0], a[1]));
        if (this->get_dist(shot, a_node) <= r + this->delta) {

            return true;
        }
    }
    return false;
}

bool Utils::check_collision(Node &start, Node &end) {
    
    if (this->check_collision_with_obs(end) || this->check_collision_with_obs(start)) {
        return true;
    }
    std::pair<Eigen::Vector2d, Eigen::Vector2d> ray = this->get_ray(start, end);
    std::vector<std::vector<Eigen::Vector2d>> obs_vertex = this->get_obs_vertex();

    for (const auto &vertex_list : obs_vertex) {
        for (int i = 0; i < vertex_list.size(); i++) {
            if (this->is_intersect_rect(start, end, ray, vertex_list[0], vertex_list[1])) {
                return true;
            }
            if (this->is_intersect_rect(start, end, ray, vertex_list[1], vertex_list[2])) {
                return true;
            }
            if (this->is_intersect_rect(start, end, ray, vertex_list[2], vertex_list[3])) {
                return true;
            }
            if (this->is_intersect_rect(start, end, ray, vertex_list[3], vertex_list[0])) {
                return true;
            }
        }
    }

    for (const auto &r : this->env.obs_circle) {
        if (this->is_intersect_circle(ray, Eigen::Vector2d(r[0], r[1]), r[2])) {
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

double Utils::get_angle(Node &start, Node &end) {
    return std::atan2(end.y - start.y, end.x - start.x);
}
