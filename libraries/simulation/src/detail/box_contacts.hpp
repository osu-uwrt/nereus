#pragma once
#include "marine_dynamics.hpp"
#include <memory>
#include <string>
#include <vector>

namespace robotics::simulation::detail {
// Value geometry. Body proxies are COM-local; static proxies use world coordinates.
// Sequence order is preserved because the original sequential impulse solve depends on it.
struct BoxProxy {
    std::string id;
    Eigen::Vector3d size{Eigen::Vector3d::Ones()};
    Eigen::Vector3d center{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};
class BoxContacts {
  public:
    BoxContacts(std::vector<BoxProxy> body, std::vector<BoxProxy> world, double restitution = .1,
                double friction = .4);
    ~BoxContacts();
    BoxContacts(const BoxContacts &) = delete;
    BoxContacts &operator=(const BoxContacts &) = delete;
    State13d resolve(State13d state, const Matrix6d &inverse_mass) const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace robotics::simulation::detail
