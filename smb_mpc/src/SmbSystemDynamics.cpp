//
// Created by johannes on 01.05.19.
//
#include <smb_mpc/SmbSystemDynamics.h>

using namespace ocs2;

namespace smb_mpc {

SmbSystemDynamics::SmbSystemDynamics(const std::string &modelName,
                                     const std::string &modelFolder,
                                     bool recompileLibraries, bool verbose) {
  this->initialize(SmbDefinitions::STATE_DIM, SmbDefinitions::INPUT_DIM,
                   modelName, modelFolder, recompileLibraries, verbose);
}

ad_vector_t SmbSystemDynamics::systemFlowMap(
    ocs2::ad_scalar_t time, const ocs2::ad_vector_t &state,
    const ocs2::ad_vector_t &input, const ocs2::ad_vector_t &parameters) const {

  using ad_quat_t = Eigen::Quaternion<ad_scalar_t>;
  using ad_vec3_t = Eigen::Matrix<ad_scalar_t, 3, 1>;

  ad_vec3_t positionDerivative = ad_vec3_t::Zero();
  ad_quat_t orientationDerivative;
  orientationDerivative.coeffs().setZero();

  ad_scalar_t v_x = SmbConversions::readLinVel(input);
  ad_scalar_t omega_z = SmbConversions::readAngVel(input);

  ad_vec3_t currentPosition = SmbConversions::readPosition(state);
  ad_quat_t currentRotation = SmbConversions::readRotation(state);

  // Compute positionDerivative and orientationDerivative
  ad_vec3_t linear_vel = ad_vec3_t::Zero();
  linear_vel[0] = v_x;

  ad_quat_t delta_quat;
  // q'(t) = q0 * d(exp(w*t/2 * n)) = q0 * exp(w /2 * n) * (w / 2 * n) = q(t) * (w / 2 * n)
  delta_quat.x() = 0;
  delta_quat.y() = 0;
  delta_quat.z() = omega_z / 2;
  delta_quat.w() = 0;

  positionDerivative = currentRotation * linear_vel;
  orientationDerivative = (currentRotation * delta_quat);

  ad_vector_t stateDerivative = ad_vector_t::Zero(SmbDefinitions::STATE_DIM);
  stateDerivative << positionDerivative, orientationDerivative.coeffs();
  return stateDerivative;
}

} // namespace smb_mpc
