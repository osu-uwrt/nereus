#pragma once
// Sequential-impulse box contact solver for the BoxScene model: the robot's box proxies against static world
// boxes.
#include "marine_dynamics.hpp"
#include <memory>
#include <nereus/simulation/contacts.hpp>
#include <string>
#include <vector>

namespace nereus::simulation::detail {
// Value geometry. Body proxies are COM-local; static proxies use world coordinates.
// Sequence order is preserved because the original sequential impulse solve depends on it.
using simulation::BoxProxy;
class BoxContacts {
  public:
    BoxContacts(std::vector<BoxProxy> body, std::vector<BoxProxy> world, double restitution = .1, double friction = .4);
    ~BoxContacts();
    BoxContacts(const BoxContacts &) = delete;
    BoxContacts &operator=(const BoxContacts &) = delete;

    // Returns `state` (13-element body state, see State13d) with contact impulses applied; inverse_mass is
    // the 6x6 inverse mass in body axes.
    State13d resolve(State13d state, const Matrix6d &inverse_mass) const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace nereus::simulation::detail
