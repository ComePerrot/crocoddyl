///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020, University of Duisburg-Essen, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "crocoddyl/core/utils/exception.hpp"
#include "crocoddyl/multibody/costs/contact-cop-position.hpp"

namespace crocoddyl {

template<typename _Scalar>
CostModelContactCoPPositionTpl<_Scalar>::CostModelContactCoPPositionTpl(boost::shared_ptr<StateMultibody> state,
                                          boost::shared_ptr<ActivationModelAbstract> activation,
                                          const FootGeometry& foot_geom)
    : Base(state, activation), foot_geom_(foot_geom) {}

template <typename Scalar>
CostModelContactCoPPositionTpl<Scalar>::~CostModelContactCoPPositionTpl() {}

template <typename Scalar>
void CostModelContactCoPPositionTpl<Scalar>::calc(const boost::shared_ptr<CostDataAbstract>& data,
                                           const Eigen::Ref<const VectorXs>&, const Eigen::Ref<const VectorXs>&) {
  Data* d = static_cast<Data*>(data.get());

  // Transform the spatial force to a cartesian force expressed in world coordinates  
  data->fiMo = d->pinnochio.SE3(d->pinocchio->oMi[d->contact->joint].rotation().T, d->contact->jMf.translation());
  data->f = data->fiMo.actInv(d->contact->f);
  
  // Compute the CoP
  // OC = (tau_0^p x n) / (n * f^p) compare eq.(13) in https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.138.8014&rep=rep1&type=pdf
  data->cop(-data->f.angular().at(1) / data->f.linear().at(2), data->f.angular().at(0) / data->f.linear().at(2), 0.0);

  // Get foot position
  foot_pos_ = d->pinocchio->oMf[d->contact->frame].translation();

  // Compute the cost residual 
  // Preliminarily: Simply take the difference between the computed CoP and the foot frame center position
  // TODO: Impose inequality constraint A*r <= 0 (cmp. contact-friction-cone.hxx ) to penalize if CoP is not inside the contact area defined by 
  data->r = foot_pos_ - data->cop;                             
                            //      dx = max(abs(px - x) - width / 2, 0);
                            //      dy = max(abs(py - y) - height / 2, 0);
                            //      d =  dx * dx + dy * dy;
  // Compute the cost
  activation_->calc(data->activation, data->r);
  data->cost = data->activation->a_value;
}

template <typename Scalar> //TODO: Develop calcDiff() following contact-friction-cone.hxx + CoP derivation
void CostModelContactCoPPositionTpl<Scalar>::calcDiff(const boost::shared_ptr<CostDataAbstract>& data,
                                                      const Eigen::Ref<const VectorXs>&, 
                                                      const Eigen::Ref<const VectorXs>&) {
  // Update all data
  Data* d = static_cast<Data*>(data.get());

  // TODO: Compute derivatives of CoP (dCoP_dx, dCoP_du)

  // TODO: Compute derivatives of the cost residual (data->Rx = A * dCoP_dx, 
                                                   //data->Ru = A * dCoP_du)

  // TODO: Compute the derivatives of the cost function
}

template <typename Scalar>
boost::shared_ptr<CostDataAbstractTpl<Scalar> > CostModelContactCoPPositionTpl<Scalar>::createData(
    DataCollectorAbstract* const data) {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this, data);
}

template<typename Scalar>
const FrameFootGeometryTpl<Scalar>& CostModelContactCoPPositionTpl<Scalar>::get_footGeom() const {
  return foot_geom_;
}

}  // namespace crocoddyl
