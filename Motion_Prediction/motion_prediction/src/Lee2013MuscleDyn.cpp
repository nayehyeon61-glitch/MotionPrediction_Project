#include "stdafx.h"
#include "BaseLib/math/nr/nr.h"
#include "Lee2013MuscleDyn.h"
#include "PhysicsLib/TRL/eigenSupport.h"
#include "PhysicsLib/DynamicsSimulator.h"
#include <cmath>
// ported to cpp by taesoo

using namespace Eigen;
VectorXd sigmoid (const VectorXd& m1) {
    
    /*  Returns the value of the sigmoid function f(x) = 1/(1 + e^-x).
        Input: m1, a vector.
        Output: 1/(1 + e^-x) for every element of the input matrix m1.
    */
    
    const unsigned long VECTOR_SIZE = m1.size();
    VectorXd output (VECTOR_SIZE);
    
    
    for( unsigned i = 0; i != VECTOR_SIZE; ++i ) {
        output[ i ] = 1 / (1 + exp(-m1[ i ]));
    }
    
    return output;
}

inline double _ASIN(double x)
{
	if (x > 1.0 )
		return M_PI/2.0;
	else if( x < -1.0 )
		return -M_PI/2.0;
	else 					
		return asin(x);
}
inline double computeCosPennation_scalar(double l_m, double l_m_opt, double pa_opt)
{
	double pa;

	if (l_m < 0. )
		l_m = 0.;

	if (l_m==0. )
		pa = _ASIN(1.);
	else
		pa = _ASIN( l_m_opt * sin(pa_opt) / l_m);

	if (pa > TO_RADIAN(45))
		pa = TO_RADIAN(45);

	double cos_pa = cos(pa);
	return cos_pa;
}
VectorXd computeCosPennation(VectorXd const& l_m, VectorXd const& l_m_opt, VectorXd const& pa_opt)
{
	int n=l_m.size();
	VectorXd cos_pa (n);
	for (int i=0; i<n; i++) 
		cos_pa[i] = computeCosPennation_scalar(l_m[i], l_m_opt[i], pa_opt[i]);
    return cos_pa;
}

inline double  computeActivationDeriv_scalar(double u, double  a, double tau_act, double tau_deact)
{
	double tau_total;
	if (u<a )
		tau_total = tau_deact/(0.5+1.5*a);
	else
		tau_total = tau_act*(0.5+1.5*a);
	double dadt = (u-a) / tau_total;
	return dadt;
}

VectorXd computeActivationDeriv(VectorXd const& u, VectorXd const& a, VectorXd const& tau_act, VectorXd const& tau_deact)
{
	int n=u.size();
	VectorXd dadt (n);
	for (int i=0;i<n; i++){
		dadt[i] = computeActivationDeriv_scalar(u[i], a[i], tau_act[i], tau_deact[i]);
	}
    return dadt;
}

inline double computeNormTendonForce_scalar(double eps_t, double eps_t_o)
{
    double k_toe = 3.0;
    double F_T_toe = 0.33;
    double k_lin = 1.712 / eps_t_o;
    double eps_t_toe = 0.609 * eps_t_o;
    
	double f_t_norm;
    if (eps_t > eps_t_toe )
        f_t_norm = k_lin * (eps_t - eps_t_toe) + F_T_toe;
    else if( eps_t > 0.0 )
        f_t_norm = (F_T_toe / (exp(k_toe)-1.)) * (exp(k_toe * eps_t / eps_t_toe) - 1.);
    else
        f_t_norm = 0.0;

    return f_t_norm;
}


VectorXd computeNormTendonForce(VectorXd const& eps_t, VectorXd const& eps_t_o)
{
	int n=eps_t.size();
	VectorXd f_t_norm (n);
	for (int i=0;i<n; i++){
		f_t_norm[i] = computeNormTendonForce_scalar(eps_t[i], eps_t_o[i]);
	}
    return f_t_norm;
}

inline double computeNormPassiveFiberForceByLength_scalar(double l_m_norm, double eps_m_o, double k_pe)
{
	double f_p_norm;
    if (l_m_norm > 1. )
        f_p_norm = (exp(k_pe * (l_m_norm - 1.) / eps_m_o) - 1.) / (exp(k_pe) - 1.);
    else
        f_p_norm = 0.;
    return f_p_norm;
}

inline double computeNormActiveFiberForceByLength_scalar(double l_m_norm, double gamma)
{
    double f_l = exp(-(l_m_norm-1.)*(l_m_norm-1.) / gamma);
    return f_l;
}

inline double computeNormActiveFiberForceByVelocity_scalar(double dl_mdt_norm, double a_f, double f_m_len, double v_m_max)
{
	double gv_norm;
    if (dl_mdt_norm <= 0. )
        gv_norm = (dl_mdt_norm + v_m_max) / (v_m_max - dl_mdt_norm/a_f);
    else
	{
        double lm_term = dl_mdt_norm*(2.+2./a_f);
        double lmax_term = v_m_max*(f_m_len-1.);
        gv_norm = (f_m_len*lm_term + lmax_term) / (lm_term + lmax_term);
	}
    return gv_norm;
}

static const double modified_damping=0.0;
inline double computeNormFiberLengthDeriv_scalar(double f_m_norm, double _a, double f_l, double a_f, double f_m_len, double damping, double v_m_max, double option)
{
    double a_f_l = _a * f_l;
	double d = damping;
	double k = 1.;
	double a,b,c;

	if (f_m_norm <= a_f_l )
	{
		a = d/a_f;
		b = -(a_f_l + f_m_norm/a_f + k*d);
		c = k*(f_m_norm - a_f_l);
	}
	else
	{
		a = -(2.+2./a_f)*d/f_m_len;
		b = -((2.+2./a_f)*(a_f_l*f_m_len - f_m_norm)/(f_m_len-1.) + k*d);
		c = k*(f_m_norm - a_f_l);
	}
		
	double det = b*b-4*a*c;
	double dl_mdt_unit = (-b-sqrt(det))/(2*a);
	return dl_mdt_unit*v_m_max;
}

inline double getFiberLengthDeriv_scalar(double a, double l_m, double l_mt, double l_m_opt, double pa_opt, double l_t_sl, double eps_t_o, double eps_m_o, double k_pe, double gamma, double a_f, double f_m_len, double damping, double v_m_max, double option)
{
	
	double cos_pa = computeCosPennation_scalar(l_m, l_m_opt, pa_opt);

	double l_t = l_mt - l_m * cos_pa;
	double eps_t = (l_t - l_t_sl) / l_t_sl;
	double f_t_norm = computeNormTendonForce_scalar(eps_t, eps_t_o);
	
	double l_m_norm = l_m / l_m_opt;
	double f_p_norm = computeNormPassiveFiberForceByLength_scalar(l_m_norm, eps_m_o, k_pe)    ;
	double f_l = computeNormActiveFiberForceByLength_scalar(l_m_norm, gamma);
	
	double f_m_norm = f_t_norm / cos_pa - f_p_norm;
	double dl_mdt_norm = computeNormFiberLengthDeriv_scalar(f_m_norm, a, f_l, a_f, f_m_len, damping, v_m_max, option);
	
	double dl_mdt = l_m_opt * dl_mdt_norm;
	return dl_mdt;
}

VectorXd getFiberLengthDeriv(VectorXd const& a, VectorXd const& l_m, VectorXd const& l_mt, VectorXd const& l_m_opt, VectorXd const& pa_opt, VectorXd const& l_t_sl, VectorXd const& eps_t_o, VectorXd const& eps_m_o, VectorXd const& k_pe, VectorXd const& gamma, VectorXd const& a_f, VectorXd const& f_m_len, VectorXd const& damping, VectorXd const& v_m_max, double option)
{
	int n=l_m.size();
	VectorXd dl_mdt (n);
	for (int i=0; i<n; i++)
	{
		dl_mdt[i] = getFiberLengthDeriv_scalar(a[i], l_m[i], l_mt[i], l_m_opt[i], pa_opt[i], l_t_sl[i], eps_t_o[i], eps_m_o[i], 
						k_pe[i], gamma[i], a_f[i], f_m_len[i], damping[i], v_m_max[i], option);
	}

	return dl_mdt;
}


static double obj_dl_m(double l_m, void* data)
{
	double* val=(double*)data;

	double dl_mdt = getFiberLengthDeriv_scalar(val[0], l_m, val[2], val[3], 
			val[4], val[5], val[6], val[7],
			val[8], val[9], val[10], val[11],
			val[12], val[13], modified_damping);
	return dl_mdt*dl_mdt;
}


	inline void shft3(DP &a, DP &b, DP &c, const DP d)
	{
		a=b;
		b=c;
		c=d;
	}
DP brent2(const DP ax, const DP bx, const DP cx, DP f(const DP, void* data), void*data, const DP tol, DP &xmin)
{
	const int ITMAX=100;
	const DP CGOLD=0.3819660;
	const DP ZEPS=1.0e-10;
	int iter;
	DP a,b,d=0.0,etemp,fu,fv,fw,fx;
	DP p,q,r,tol1,tol2,u,v,w,x,xm;
	DP e=0.0;

	a=(ax < cx ? ax : cx);
	b=(ax > cx ? ax : cx);
	x=w=v=bx;
	fw=fv=fx=f(x, data);
	for (iter=0;iter<ITMAX;iter++) {
		xm=0.5*(a+b);
		tol2=2.0*(tol1=tol*fabs(x)+ZEPS);
		if (fabs(x-xm) <= (tol2-0.5*(b-a))) {
			xmin=x;
			return fx;
		}
		if (fabs(e) > tol1) {
			r=(x-w)*(fx-fv);
			q=(x-v)*(fx-fw);
			p=(x-v)*q-(x-w)*r;
			q=2.0*(q-r);
			if (q > 0.0) p = -p;
			q=fabs(q);
			etemp=e;
			e=d;
			if (fabs(p) >= fabs(0.5*q*etemp) || p <= q*(a-x) || p >= q*(b-x))
				d=CGOLD*(e=(x >= xm ? a-x : b-x));
			else {
				d=p/q;
				u=x+d;
				if (u-a < tol2 || b-u < tol2)
					d=SIGN(tol1,xm-x);
			}
		} else {
			d=CGOLD*(e=(x >= xm ? a-x : b-x));
		}
		u=(fabs(d) >= tol1 ? x+d : x+SIGN(tol1,d));
		fu=f(u, data);
		if (fu <= fx) {
			if (u >= x) a=x; else b=x;
			shft3(v,w,x,u);
			shft3(fv,fw,fx,fu);
		} else {
			if (u < x) a=u; else b=u;
			if (fu <= fw || w == x) {
				v=w;
				w=u;
				fv=fw;
				fw=fu;
			} else if (fu <= fv || v == x || v == w) {
				v=u;
				fv=fu;
			}
		}
	}
	xmin=x;
	return fx;
}
VectorXd getIsometricFiberLength(VectorXd const& a, VectorXd const& l_mt, VectorXd const& l_m_opt, VectorXd const& pa_opt, VectorXd const& l_t_sl, VectorXd const& eps_t_o, VectorXd const& eps_m_o, 
                        VectorXd const& k_pe,VectorXd const&  gamma,VectorXd const&  a_f,VectorXd const&  f_m_len,VectorXd const&  damping,VectorXd const&  v_m_max)
{
	int n=l_mt.size();
	VectorXd iso_l_m (n);
	for(int m=0; m<n; m++){

		double val[14];
		val[0]= a[m];
		//val[1]=l_m;
		val[2]=l_mt[m];
		val[3]=l_m_opt[m];
		val[4]=pa_opt[m]; 
		val[5]=l_t_sl[m]; 
		val[6]=eps_t_o[m]; 
		val[7]=eps_m_o[m]; 
		val[8]= k_pe[m];
		val[9]=gamma[m];
		val[10]=a_f[m];
		val[11]=f_m_len[m]; 
		val[12]= damping[m]; 
		val[13]=v_m_max[m];


		double ub;
		if (l_mt[m]-l_t_sl[m]>0. )
			ub = l_mt[m]-l_t_sl[m];
		else
			ub = 0.;
		double x0 = ub;
		/*
		--ub = ub + .2

		--print'-------------------'
		--print(m)
		--print(l_mt[m]-l_t_sl[m])
		--print('x0 ', x0)
		*/

		double fret;
		fret = brent2(.0, x0, ub, obj_dl_m, (void*)val, 2e-5, iso_l_m[m]);
	}
	return iso_l_m;
}
inline double fA(double u)
{
	double hpi = M_PI/2.;
	return 40*0.5*sin(hpi*u) + 133*0.5*(1-cos(hpi*u));
}
inline double g(double l_ce)
{
	if (l_ce <= 0.5 )
		return 0.5;
	else if (l_ce <= 1.0 )
		return l_ce;
	else if (l_ce <= 1.5 )
		return -2*l_ce + 3;
	else
		return 0;
}
inline double fM(double a)
{
	double hpi = M_PI/2.0;
	return 74*0.5*sin(hpi*a) + 111*0.5*(1-cos(hpi*a));
}

inline double getMetabolicEnergyRate_muscle(double mass, double u, double a, double l_m, double l_m_opt, double dl_m, double f_mtu, double f_ce)
{
	double dA = mass*fA(u);

	double dM = mass*g(l_m/l_m_opt)*fM(a);

	double dS ;
	if (dl_m < 0 )
		dS = 0.25*f_mtu*-dl_m;
	else
		dS = 0;

	double dW;
	if (dl_m < 0 )
		dW = f_ce*-dl_m;
	else
		dW = 0;

	return dA + dM + dS + dW;
}

//--unit : 1 J/s = (1/4.184) cal/s)
double getMetabolicEnergyRate(double totalMass, VectorXd const& mass, VectorXd const& u, VectorXd const& a, VectorXd const& l_m, VectorXd const& l_m_opt, VectorXd const& dl_m, VectorXd const& f_mtu, VectorXd const& f_ce)
{
	double dE = 0;
	for(int i=0; i<mass.size(); i++)
		dE += getMetabolicEnergyRate_muscle(mass[i], u[i], a[i], l_m[i], l_m_opt[i], dl_m[i], f_mtu[i], f_ce[i]);
	double dB = 1.51 * totalMass;
	return dE + dB;
}


inline void _clamp_dl_m(int n , double* dl_m, VectorXd const& l_m_opt, VectorXd const& v_m_max)
{
	for (int i=0; i<n; i++)
	{
		if (dl_m[i] < -l_m_opt[i]*v_m_max[i] )
			dl_m[i] = -l_m_opt[i]*v_m_max[i];
		else if (dl_m[i] > l_m_opt[i]*v_m_max[i] )
			dl_m[i] = l_m_opt[i]*v_m_max[i];
	}
}
void clamp_dl_m(VectorXd & dl_m, VectorXd const& l_m_opt, VectorXd const& v_m_max)
{
	_clamp_dl_m(dl_m.size(), &dl_m[0], l_m_opt, v_m_max);
}


inline VectorXd muscledyn(double t, VectorXd const&  state,  VectorXd const& u, muscle_data const& d)
{
	VectorXd newState(state.size());

	int n=state.size()/2;
	auto const& a=state.head(n);
	newState.head(n)=computeActivationDeriv(u, a, d._tau_act, d._tau_deact);
	newState.tail(n)=getFiberLengthDeriv(a, state.tail(n), d._l_mt, d._l_m_opt, 
			d._pa_opt, d._l_t_sl, d._eps_t_o, d._eps_m_o, 
			d._k_pe, d._gamma, d._a_f, d._f_m_len, d._damping, d._v_m_max, d._option);
	_clamp_dl_m(n, &newState[n], d._l_m_opt, d._v_m_max);

	return newState;
}

inline VectorXd s_mul_t(double sca, VectorXd const& m)
{
	return m*sca;
}
inline VectorXd t_add_t(VectorXd const& t1, VectorXd const& t2)
{
	return t1+t2;
}
inline double rk4_step( double t_curr, VectorXd &y_curr, VectorXd& dlm, double h, VectorXd const& u, muscle_data const& data)
{

#if 0
	auto k1 = s_mul_t(h, muscledyn(t_curr, y_curr, u,data));
	auto k2 = s_mul_t(h, muscledyn(t_curr+.5*h, t_add_t(y_curr, s_mul_t(.5, k1)), u,data));
	auto k3 = s_mul_t(h, muscledyn(t_curr+.5*h, t_add_t(y_curr, s_mul_t(.5, k2)), u,data));
	auto k4 = s_mul_t(h, muscledyn(t_curr+h, t_add_t(y_curr, k3), u,data));
	auto y_next = t_add_t(y_curr, s_mul_t(1.0/6.0, t_add_t(k1, t_add_t(s_mul_t(2., t_add_t(k2, k3)), k4))));
	y_curr=y_next;
	return t_curr+h;
#elif 1
	auto k1 = s_mul_t(h, muscledyn(t_curr, y_curr, u,data));
	auto k2 = s_mul_t(h, muscledyn(t_curr+.5*h, t_add_t(y_curr, s_mul_t(.5, k1)), u,data));
	auto k3 = s_mul_t(h, muscledyn(t_curr+.5*h, t_add_t(y_curr, s_mul_t(.5, k2)), u,data));
	auto k4 = s_mul_t(h, muscledyn(t_curr+h, t_add_t(y_curr, k3), u,data));
	auto dot_y=s_mul_t(1.0/6.0, t_add_t(k1, t_add_t(s_mul_t(2., t_add_t(k2, k3)), k4)));
	auto y_next = t_add_t(y_curr, dot_y);
	dlm=dot_y.tail(u.size())/h;
	y_curr=y_next;
	return t_curr+h;
#else

	// doesn't work -.- I hate eigen. possibly some strange aliasing is happening
	auto k1 = h*muscledyn(t_curr, y_curr,u,  data);
	auto k2 = h*muscledyn(t_curr+.5*h, y_curr+.5*k1, u, data);
	auto k3 = h*muscledyn(t_curr+.5*h, y_curr+.5*k2, u, data);
	auto k4 = h*muscledyn(t_curr+h, y_curr+k3, u, data);
	y_curr += (k1 + 2.*(k2+k3) + k4)/ 6.;
	return t_curr+h;
#endif
}

void integrateMuscleDynamics(double duration, double muscleTimeStep, VectorXd &a, VectorXd &l_m, VectorXd &dl_m, VectorXd const& u, muscle_data const& d)
{
	double  t = 0.;
	int n=a.size();
	VectorXd y (n*2);
	VectorXd doty(n*2);
	y<<a, l_m;
	for (int i=0,ni=ROUND(duration/muscleTimeStep); i<ni; i++)
	{
		t=rk4_step( t, y, doty, muscleTimeStep, u, d); 
		//y+=muscleTimeStep*muscledyn(t+muscleTimeStep, y, u, d);

	}
	a=y.head(n);
	l_m=y.tail(n);
	dl_m=doty;
}

path_points::path_points(int num_muscles, int numPathpoints)
{
	_data.resize(numPathpoints);
	eppis.resize(num_muscles);
	vppis.resize(num_muscles);
	pathPointPositions.setSize(numPathpoints);
	lm_t.resize(num_muscles);
}

void path_points::setPathPoint(int i, int ti, int mi, int cdof, double r1, double r2, vector3 const& lp)
{
	auto& pp=_data[i];
	pp.treeIndex=ti;
	pp.muscleIndex=mi;
	pp.conditionalDOFindex=cdof;
	pp.range[0]=r1;
	pp.range[1]=r2;
	pp.localPos=lp;
}
void path_points::updatePPindices(BoneForwardKinematics const& fk, vectorn const& posedof)
{
	int n_muscles=eppis.size();

	for (int i_muscle=0; i_muscle<n_muscles; i_muscle++)
		eppis[i_muscle].setSize(0);

	for (int i_pp=0; i_pp<_data.size(); i_pp++)
	{
		auto& pp=_data[i_pp];
		int i_muscle=pp.muscleIndex;
		int dofIndex=pp.conditionalDOFindex ;
		if (dofIndex>=-1 )
		{

			if (dofIndex!=-1 && posedof(dofIndex)>pp.range[0] && posedof(dofIndex)<pp.range[1] )
				eppis[i_muscle].pushBack(i_pp);
		}
		else
			eppis[i_muscle].pushBack(i_pp);
	}

	_calcPathPointPositions(fk, pathPointPositions);
	_calcTendonMuscleLengths(pathPointPositions, lm_t);

	for (int i_muscle=0; i_muscle<n_muscles; i_muscle++)
	{
		vppis[i_muscle].setSize(0);
		auto& ais = eppis[i_muscle];
		auto& fais = vppis[i_muscle];
		for (int i=1; i<ais.size(); i++)
		{
			if (_data[ais[i-1]].treeIndex!=_data[ais[i]].treeIndex)
			{
				if (fais.size()==0 || (fais(fais.size()-1)!=ais(i-1)) )
					fais.pushBack(ais(i-1));
				fais.pushBack(ais(i));
			}
		}
	}
}

inline void path_points::_calcPathPointPositions(BoneForwardKinematics const& fk, vector3N& pathPointPositions)
{
	int n_muscles=eppis.size();
	pathPointPositions.setSize(_data.size());
	for (int i_muscle=0; i_muscle<n_muscles; i_muscle++)
	{
		auto& ais = eppis[i_muscle];
		for (int i=0; i<ais.size(); i++)
		{
			int i_pp=ais(i);
			auto& pp=_data[i_pp];
			pathPointPositions(i_pp)=fk.global(pp.treeIndex)*pp.localPos;
		}
	}
}
void path_points::_calcTendonMuscleLengths(vector3N const& pathPointPositions, Eigen::VectorXd& lm_t)
{
	int n_muscles=eppis.size();
	lm_t.resize(n_muscles);
	for (int i_muscle=0; i_muscle<n_muscles; i_muscle++)
	{
		auto& ais = eppis[i_muscle];
		double len=0;
		for (int i=1;i<ais.size(); i++)
			len+=pathPointPositions(ais(i)).distance(pathPointPositions(ais(i-1)));
		lm_t[i_muscle]=len;
	}
}

inline void addWorldForceToBone(int ichara, int i_joint, vector3 const& point, vector3 const& force, OpenHRP::DynamicsSimulator & sim)
{
	sim.addGlobalForceToBone(ichara, i_joint, point, force);
}

void path_points::applyMuscleForces(int ichara, Eigen::VectorXd const& forces, OpenHRP::DynamicsSimulator & sim)
{
	int n_muscles=eppis.size();

	for (int i_muscle=0; i_muscle<n_muscles; i_muscle++)
	{
		auto& is_pp=vppis[i_muscle];
		double forceScalar=forces[i_muscle];

		for (int i=0,ni= is_pp.size(); i<ni; i++)
		{
			int i_pp=is_pp(i);
			auto& pp=_data[i_pp];
			auto const& ap=pathPointPositions(i_pp);
			if (i==0) {
				vector3 dn = pathPointPositions(is_pp(i+1))-ap; dn.normalize();
				addWorldForceToBone(ichara, pp.treeIndex, ap, forceScalar*dn, sim);
			}
			else if(i==ni-1) {
				vector3 dp = pathPointPositions(is_pp(i-1))-ap; dp.normalize();
				addWorldForceToBone(ichara, pp.treeIndex, ap, forceScalar*dp, sim);
			}
			else
			{
				vector3 dn = pathPointPositions(is_pp(i+1))-ap; dn.normalize();
				addWorldForceToBone(ichara, pp.treeIndex, ap, forceScalar*dn, sim);
				vector3 dp = pathPointPositions(is_pp(i-1))-ap; dp.normalize();
				addWorldForceToBone(ichara, pp.treeIndex, ap, forceScalar*dp, sim);
			}
		}
	}
}


Eigen::VectorXd computeNormActiveFiberForce(Eigen::VectorXd const& _norm_l_m_prev, Eigen::VectorXd const& _norm_dl_m_prev, muscle_data const& data)
{
	int n_muscles=_norm_l_m_prev.size();
	VectorXd gal_gv(n_muscles);

	for(int i_muscle=0; i_muscle<n_muscles; i_muscle++){
		double norm_l_m_prev=_norm_l_m_prev[i_muscle];
		double norm_dl_m_prev=_norm_dl_m_prev[i_muscle];
		/*
		//-- set larger activation only when solving qp.
		//-- because l_m and l_m_prev is constant so problem can be infeasible
		if (norm_l_m_prev < .1 )
			norm_l_m_prev = .1;
		if (norm_dl_m_prev < -.9 )
			norm_dl_m_prev = -.9;
			*/

		double gal = computeNormActiveFiberForceByLength_scalar(norm_l_m_prev, data._gamma[i_muscle]);
		double gv = computeNormActiveFiberForceByVelocity_scalar(norm_dl_m_prev, data._a_f[i_muscle], data._f_m_len[i_muscle], data._v_m_max[i_muscle]);
		gal_gv[i_muscle]=gal*gv;
	}
	return gal_gv;
}
Eigen::VectorXd computePassiveFiberForce(Eigen::VectorXd const& f_m_o, Eigen::VectorXd const& norm_l_m_prev, Eigen::VectorXd const& dl_m, muscle_data const& data)
{
	int n_muscles=norm_l_m_prev.size();
	VectorXd out(n_muscles);
	for(int i_muscle=0; i_muscle<n_muscles; i_muscle++){
		out[i_muscle]=f_m_o[i_muscle]*computeNormPassiveFiberForceByLength_scalar(norm_l_m_prev[i_muscle], data._eps_m_o[i_muscle], data._k_pe[i_muscle]) 
			+ data._damping[i_muscle]*dl_m[i_muscle];
	}
	return out;
}
