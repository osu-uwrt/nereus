#pragma once
#include "marine_dynamics.hpp"
#include <memory>
#include <robotics/simulation/contacts.hpp>
#include <string>
#include <vector>

namespace robotics::simulation::detail {
// Value geometry. Body proxies are COM-local; static proxies use world coordinates.
// Sequence order is preserved because the original sequential impulse solve depends on it.
using simulation::BoxProxy;
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
