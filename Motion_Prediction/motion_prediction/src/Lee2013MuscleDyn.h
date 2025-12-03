#pragma once
#ifdef Success
#undef Success
#endif
#include <Eigen/Dense>
#include "PhysicsLib/TRL/eigenSupport.h"

namespace OpenHRP
{
class DynamicsSimulator ;
}
Eigen::VectorXd sigmoid (const Eigen::VectorXd& m1);
double computeCosPennation_scalar(double l_m, double l_m_opt, double pa_opt);
Eigen::VectorXd computeCosPennation(Eigen::VectorXd const& l_m, Eigen::VectorXd const& l_m_opt, Eigen::VectorXd const& pa_opt);
double  computeActivationDeriv_scalar(double u, double  a, double tau_act, double tau_deact);
Eigen::VectorXd computeActivationDeriv(Eigen::VectorXd const& u, Eigen::VectorXd const& a, Eigen::VectorXd const& tau_act, Eigen::VectorXd const& tau_deact);
double computeNormTendonForce_scalar(double eps_t, double eps_t_o);
Eigen::VectorXd computeNormTendonForce(Eigen::VectorXd const& eps_t, Eigen::VectorXd const& eps_t_o);
double computeNormPassiveFiberForceByLength_scalar(double l_m_norm, double eps_m_o, double k_pe);
double computeNormActiveFiberForceByLength_scalar(double l_m_norm, double gamma);
double computeNormActiveFiberForceByVelocity_scalar(double dl_mdt_norm, double a_f, double f_m_len, double v_m_max);
double computeNormFiberLengthDeriv_scalar(double f_m_norm, double _a, double f_l, double a_f, double f_m_len, double damping, double v_m_max, double option);
double getFiberLengthDeriv_scalar(double a, double l_m, double l_mt, double l_m_opt, double pa_opt, double l_t_sl, double eps_t_o, double eps_m_o, double k_pe, double gamma, double a_f, double f_m_len, double damping, double v_m_max, double option);
Eigen::VectorXd getFiberLengthDeriv(Eigen::VectorXd const& a, Eigen::VectorXd const& l_m, Eigen::VectorXd const& l_mt, Eigen::VectorXd const& l_m_opt, Eigen::VectorXd const& pa_opt, Eigen::VectorXd const& l_t_sl, Eigen::VectorXd const& eps_t_o, Eigen::VectorXd const& eps_m_o, Eigen::VectorXd const& k_pe, Eigen::VectorXd const& gamma, Eigen::VectorXd const& a_f, Eigen::VectorXd const& f_m_len, Eigen::VectorXd const& damping, Eigen::VectorXd const& v_m_max, double option);
Eigen::VectorXd getIsometricFiberLength(Eigen::VectorXd const& a, Eigen::VectorXd const& l_mt, Eigen::VectorXd const& l_m_opt, Eigen::VectorXd const& pa_opt, Eigen::VectorXd const& l_t_sl, Eigen::VectorXd const& eps_t_o, Eigen::VectorXd const& eps_m_o, Eigen::VectorXd const& k_pe,Eigen::VectorXd const&  gamma,Eigen::VectorXd const&  a_f,Eigen::VectorXd const&  f_m_len,Eigen::VectorXd const&  damping,Eigen::VectorXd const&  v_m_max);
void clamp_dl_m(Eigen::VectorXd & dl_m, Eigen::VectorXd const& l_m_opt, Eigen::VectorXd const& v_m_max);

struct path_points
{
	struct path_point {
		int treeIndex;
		int muscleIndex;
		int conditionalDOFindex;
		double range[2];
		vector3 localPos;
	};
	std::vector<path_point> _data;
	path_points(int num_muscles, int numPathpoints);
	void setPathPoint(int i, int ti, int mi, int cdof, double r1, double r2, vector3 const& lp);

	// actually update pathPoint positions and tendon muscle lengths too.
	void updatePPindices(BoneForwardKinematics const& fk, vectorn const& posedof);

	void _calcPathPointPositions(BoneForwardKinematics const& fk, vector3N& pathPointPositions);
	void _calcTendonMuscleLengths(vector3N const& pathPointPositions, Eigen::VectorXd& lm_t);

	vector3N pathPointPositions;
	Eigen::VectorXd lm_t;

	std::vector<intvectorn> eppis;
	std::vector<intvectorn> vppis;
	inline intvectorn const& get_eppis(int imuscle) const { return eppis[imuscle];}
	inline intvectorn const& get_vppis(int imuscle) const { return vppis[imuscle];}
	inline vector3 getPathPointPos(int i_pp) const { return pathPointPositions(i_pp);}
	const Eigen::VectorXd& get_lm_t() const { return lm_t;}

	void applyMuscleForces(int ichara, Eigen::VectorXd const& forces, OpenHRP::DynamicsSimulator& sim);
};


struct muscle_data
{
	Eigen::VectorXd const& _l_mt; 
	Eigen::VectorXd const& _l_m_opt; 
	Eigen::VectorXd const& _pa_opt; 
	Eigen::VectorXd const& _l_t_sl; 
	Eigen::VectorXd const& _eps_t_o; 
	Eigen::VectorXd const& _eps_m_o; 
	Eigen::VectorXd const& _k_pe; 
	Eigen::VectorXd const& _gamma; 
	Eigen::VectorXd const& _a_f; 
	Eigen::VectorXd const& _f_m_len; 
	Eigen::VectorXd const& _damping; 
	Eigen::VectorXd const& _v_m_max; 
	Eigen::VectorXd const& _tau_act; 
	Eigen::VectorXd const& _tau_deact;
	double _option;

		muscle_data(
	Eigen::VectorXd const& l_mt,
	Eigen::VectorXd const& l_m_opt, Eigen::VectorXd const& pa_opt, Eigen::VectorXd const& l_t_sl, Eigen::VectorXd const& eps_t_o, Eigen::VectorXd const& eps_m_o, 
	Eigen::VectorXd const& k_pe, Eigen::VectorXd const& gamma, Eigen::VectorXd const& a_f, Eigen::VectorXd const& f_m_len, Eigen::VectorXd const& damping, Eigen::VectorXd const& v_m_max, double option,
	Eigen::VectorXd const& tau_act, Eigen::VectorXd const& tau_deact):
	_l_mt(l_mt),
	_l_m_opt(l_m_opt), 
	_pa_opt(pa_opt), 
	_l_t_sl(l_t_sl), 
	_eps_t_o(eps_t_o), 
	_eps_m_o(eps_m_o), 
	_k_pe(k_pe), 
	_gamma(gamma), 
	_a_f(a_f), 
	_f_m_len(f_m_len), 
	_damping(damping), 
	_v_m_max(v_m_max), 
	_tau_act(tau_act), 
	_tau_deact(tau_deact),
	_option(option)
		{
		}

};


void integrateMuscleDynamics(double duration, double muscleTimeStep, Eigen::VectorXd &a, Eigen::VectorXd &l_m, Eigen::VectorXd&dl_m, Eigen::VectorXd const& u, muscle_data const& d);
Eigen::VectorXd computeNormActiveFiberForce(Eigen::VectorXd const& norm_l_m_prev, Eigen::VectorXd const& norm_dl_m_prev, muscle_data const& d);
Eigen::VectorXd computePassiveFiberForce(Eigen::VectorXd const& f_m_o, Eigen::VectorXd const& norm_l_m_prev, Eigen::VectorXd const& dl_m_prev, muscle_data const& d);
double getMetabolicEnergyRate(double totalMass, Eigen::VectorXd const& mass, Eigen::VectorXd const& u, Eigen::VectorXd const& a, Eigen::VectorXd const& l_m, Eigen::VectorXd const& l_m_opt, Eigen::VectorXd const& dl_m, Eigen::VectorXd const& f_mtu, Eigen::VectorXd const& f_ce);

