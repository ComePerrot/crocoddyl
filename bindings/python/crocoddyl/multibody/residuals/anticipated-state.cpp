///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "crocoddyl/multibody/residuals/anticipated-state.hpp"
#include "python/crocoddyl/multibody/multibody.hpp"

namespace crocoddyl {
namespace python {

void exposeResidualAnticipatedState() {
  bp::register_ptr_to_python<boost::shared_ptr<ResidualModelAnticipatedState> >();

  bp::class_<ResidualModelAnticipatedState, bp::bases<ResidualModelAbstract> >(
      "ResidualModelAnticipatedState",
      "This cost function defines a residual vector as r = q + time * v, with q and v as\n"
      "the current position and velocity joints and time a diagonal matrix.",
      bp::init<boost::shared_ptr<StateAbstract>, std::size_t, double>(
          bp::args("self", "state", "nu","anticipated_time"),
          "Initialize the state cost model.\n\n"
          ":param state: state description\n"
          ":param nu: dimension of control vector\n"
          ":param anticipated_time: time of anticipation"))
      .def(bp::init<boost::shared_ptr<StateAbstract>, double>(
          bp::args("self", "state", "anticipated_time"),
          "Initialize the state cost model.\n\n"
          "The default nu value is obtained from state.nv.\n"
          ":param state: state description\n"
          ":param anticipated_time: time of anticipation"))
      .def<void (ResidualModelAnticipatedState::*)(const boost::shared_ptr<ResidualDataAbstract>&,
                                        const Eigen::Ref<const Eigen::VectorXd>&,
                                        const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &ResidualModelAnticipatedState::calc, bp::args("self", "data", "x", "u"),
          "Compute the state cost.\n\n"
          ":param data: cost data\n"
          ":param x: time-discrete state vector\n"
          ":param u: time-discrete control input")
      .def<void (ResidualModelAnticipatedState::*)(const boost::shared_ptr<ResidualDataAbstract>&,
                                        const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &ResidualModelAbstract::calc, bp::args("self", "data", "x"))
      .def<void (ResidualModelAnticipatedState::*)(const boost::shared_ptr<ResidualDataAbstract>&,
                                        const Eigen::Ref<const Eigen::VectorXd>&,
                                        const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &ResidualModelAnticipatedState::calcDiff, bp::args("self", "data", "x", "u"),
          "Compute the derivatives of the state cost.\n\n"
          "It assumes that calc has been run first.\n"
          ":param data: action data\n"
          ":param x: time-discrete state vector\n"
          ":param u: time-discrete control input\n")
      .def<void (ResidualModelAnticipatedState::*)(const boost::shared_ptr<ResidualDataAbstract>&,
                                        const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &ResidualModelAbstract::calcDiff, bp::args("self", "data", "x"));
}

}  // namespace python
}  // namespace crocoddyl
