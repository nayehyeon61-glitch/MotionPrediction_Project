bindTargetClassification={
	classes={
			{
			name='ConvexDecomp',
			decl=[[
				namespace OBJloader
				{
					class ConvexDecomp;
				}
				]],
				headerFile='v-hacd/ConvexDecomp.h',
				className='OBJloader::ConvexDecomp',
				ctors={
					'(OBJloader::Mesh const&, bool performConvexDecomposition )',
					'(OBJloader::Mesh const&, bool performConvexDecomposition, int voxelRes)',
					'(OBJloader::Mesh const&, bool performConvexDecomposition, int voxelRes, int maxConvex)',
					'(OBJloader::Mesh const&, bool performConvexDecomposition, int voxelRes, int maxConvex, double volumeErr)',
					'(OBJloader::Mesh const&, bool performConvexDecomposition, int voxelRes, int maxConvex, double volumeErr, )',
				},
				memberFunctions=[[
				void performConvexDecomposition();
				void performConvexDecomposition(VoxelGraph::Image3D const& image, int value);
				vector3N const& centroids() const 
				int numConvex() const 
				OBJloader::Mesh const& convex(int h) const 
				vector3 getBoundsMin() const;
				vector3 getBoundsMax() const;
				vector3 getDimensions() const;
				int getVoxel(unsigned int x, unsigned int y, unsigned int z) const;
				int getVoxel(vector3 const& vi) const 
				int getVoxelSafe(vector3 const& vi) const 

				int numSurfaceVoxels() const;
				int numInteriorVoxels() const;
				vector3 getSurfaceVoxelIndex(int isurfvoxel) const;
				vector3 getInteriorVoxelIndex(int isurfvoxel) const;
				vector3 getVoxelIndex(vector3 const& worldPosition) const;
				vector3 getWorldPosition(vector3 const& voxelIndex) const;
				vector3 getWorldPosition(VoxelGraph::shortvector3 const& voxelIndex) const;

				VoxelGraph* createGraphFromAllOccupantVoxels() const; @ ;adopt=true;
				]]
			},

		{
			name='math.CMAwrap',
			cppname='CMAwrap',
			decl=true,
			ctors=[[
				(vectorn const& start_p, vectorn const& stdev, int populationSize, int mu);
				(vectorn const& start_p, vectorn const& stdev, int populationSize);
			]],
			memberFunctions=[[
			std::string testForTermination();	
			void samplePopulation();
			int numPopulation();
			int dim();
			vectornView getPopulation(int i);
			void setVal(int i, double eval);
			void resampleSingle(int i);
			void resampleSingleFrom(int i, vectorn const&);
			void update();
			void getMean(vectorn& out);
			void getBest(vectorn& out);
			]]
		},
		{
			luaname='VectorXd',
			cppname='Eigen::VectorXd',
			ctors=[[
			()
			(int)
			]],
			wrapperCode=[[
			inline static void assign(Eigen::VectorXd & out, Eigen::VectorXd const& other)
			{
				out=other;
			}
			inline static void assign(Eigen::VectorXd & out, vectorn const& other)
			{
				out=eigenView(other);
			}
			]],
			staticMemberFunctions=[[
			vectornView vecView(Eigen::VectorXd const& x);
			void assign(Eigen::VectorXd & out, Eigen::VectorXd const& other); 
			void assign(Eigen::VectorXd & out, vectorn const& other); 
			]],
		},
		{
			name='path_points',
			ctors=[[
			(int num_muscles, int numPathpoints)
			]],
			memberFunctions=[[
			void setPathPoint(int i, int ti, int mi, int cdof, double r1, double r2, vector3 const& lp);
			void updatePPindices(BoneForwardKinematics const& fk, vectorn const& posedof);
			intvectorn const& get_eppis(int imuscle) const { return eppis[imuscle];}
			intvectorn const& get_vppis(int imuscle) const { return vppis[imuscle];}
			vector3 getPathPointPos(int i_pp) const { return pathPointPositions(i_pp);}
			const Eigen::VectorXd& get_lm_t() const { return lm_t;}
			void applyMuscleForces(int ichara, Eigen::VectorXd const& forces, OpenHRP::DynamicsSimulator & sim);
			void _calcPathPointPositions(BoneForwardKinematics const& fk, vector3N& pathPointPositions);
			void _calcTendonMuscleLengths(vector3N const& pathPointPositions, Eigen::VectorXd& lm_t);
			]],
		},
		{
			name='muscle_data',
			ctors=[[
			( Eigen::VectorXd const& l_mt, Eigen::VectorXd const& l_m_opt, Eigen::VectorXd const& pa_opt, Eigen::VectorXd const& l_t_sl, Eigen::VectorXd const& eps_t_o, Eigen::VectorXd const& eps_m_o, Eigen::VectorXd const& k_pe, Eigen::VectorXd const& gamma, Eigen::VectorXd const& a_f, Eigen::VectorXd const& f_m_len, Eigen::VectorXd const& damping, Eigen::VectorXd const& v_m_max, double option, Eigen::VectorXd const& tau_act, Eigen::VectorXd const& tau_deact)
			]],
		}
	},
	modules={
		{
			namespace='Eigen',
			functions={[[
			double solve_quadprog( HessianQuadratic & problem, const matrixn & CE, const vectorn & ce0, const matrixn & CI, const vectorn & ci0, vectorn & x) @ solveQuadprog
			double solve_quadprog( HessianQuadratic & problem, const matrixn & CE, const vectorn & ce0, const matrixn & CI, const vectorn & ci0, vectorn & x, bool) @ solveQuadprog
			Eigen::VectorXd sigmoid (const Eigen::VectorXd& m1);
			Eigen::VectorXd computeCosPennation(Eigen::VectorXd const& l_m, Eigen::VectorXd const& l_m_opt, Eigen::VectorXd const& pa_opt);
			Eigen::VectorXd computeActivationDeriv(Eigen::VectorXd const& u, Eigen::VectorXd const& a, Eigen::VectorXd const& tau_act, Eigen::VectorXd const& tau_deact);
			Eigen::VectorXd computeNormTendonForce(Eigen::VectorXd const& eps_t, Eigen::VectorXd const& eps_t_o);
			Eigen::VectorXd computeNormActiveFiberForce(Eigen::VectorXd const& norm_l_m_prev, Eigen::VectorXd const& norm_dl_m_prev, muscle_data const& d);
			Eigen::VectorXd computePassiveFiberForce(Eigen::VectorXd const& f_m_o, Eigen::VectorXd const& norm_l_m_prev, Eigen::VectorXd const& dl_m_prev, muscle_data const& d);
			Eigen::VectorXd getFiberLengthDeriv(Eigen::VectorXd const& a, Eigen::VectorXd const& l_m, Eigen::VectorXd const& l_mt, Eigen::VectorXd const& l_m_opt, Eigen::VectorXd const& pa_opt, Eigen::VectorXd const& l_t_sl, Eigen::VectorXd const& eps_t_o, Eigen::VectorXd const& eps_m_o, Eigen::VectorXd const& k_pe, Eigen::VectorXd const& gamma, Eigen::VectorXd const& a_f, Eigen::VectorXd const& f_m_len, Eigen::VectorXd const& damping, Eigen::VectorXd const& v_m_max, double option);
			Eigen::VectorXd getIsometricFiberLength(Eigen::VectorXd const& a, Eigen::VectorXd const& l_mt, Eigen::VectorXd const& l_m_opt, Eigen::VectorXd const& pa_opt, Eigen::VectorXd const& l_t_sl, Eigen::VectorXd const& eps_t_o, Eigen::VectorXd const& eps_m_o, Eigen::VectorXd const& k_pe,Eigen::VectorXd const&  gamma,Eigen::VectorXd const&  a_f,Eigen::VectorXd const&  f_m_len,Eigen::VectorXd const&  damping,Eigen::VectorXd const&  v_m_max)
			void clamp_dl_m(Eigen::VectorXd & dl_m, Eigen::VectorXd const& l_m_opt, Eigen::VectorXd const& v_m_max);
			void integrateMuscleDynamics(double duration, double muscleTimeStep, Eigen::VectorXd &a, Eigen::VectorXd &l_m, Eigen::VectorXd& dl_m, Eigen::VectorXd const& u, muscle_data const& d);
			double getMetabolicEnergyRate(double totalMass, Eigen::VectorXd const& mass, Eigen::VectorXd const& u, Eigen::VectorXd const& a, Eigen::VectorXd const& l_m, Eigen::VectorXd const& l_m_opt, Eigen::VectorXd const& dl_m, Eigen::VectorXd const& f_mtu, Eigen::VectorXd const& f_ce)
			]]}
		}, 
		{
			namespace='MPI',
			ifdef='USE_MPI',
			wrapperCode=
			[[
			#ifdef USE_MPI

			static int size()
			{
			int size, rc;
			rc = MPI_Comm_size(MPI_COMM_WORLD, &size);
			return size;
			}

			static int rank()
			{
			int rank, rc;
			rc = MPI_Comm_rank(MPI_COMM_WORLD, &rank);
			return rank;
			}

			static void send(const char* msg, int i)
			{
			int rc, tag=100;
			int len=strlen(msg);
			rc = MPI_Send(&len, 1, MPI_INT, i, tag, MPI_COMM_WORLD);
			rc = MPI_Send((void*)msg, len+1, MPI_CHAR, i, tag, MPI_COMM_WORLD);
			}

			static std::string receive(int i)
			{
			int rc, tag=100;
			int len;
			rc = MPI_Recv(&len, 1, MPI_INT, i, tag, MPI_COMM_WORLD, &status);
			TString temp;
			temp.reserve(len+1);
			rc = MPI_Recv((void*)&temp[0], len+1, MPI_CHAR, status.MPI_SOURCE, tag, MPI_COMM_WORLD, &status);
			temp.updateLength();
			return std::string(temp.ptr());
			}

			static int source()
			{
			  return status.MPI_SOURCE;
			}
			static void test()
			{
			vectorn a(1);
			a.set(0,1);

			for(int i=0;i<1000; i++)
			{
				for(int j=0; j<1000; j++)
				{
					a(0)=a(0)+1;
				}
			}
			}
			static void test2()
			{
			int a=1;
			for(int i=0;i<1000; i++)
			{
				for(int j=0; j<1000; j++)
				{
					a=a+1;
				}
			}
			}

			#endif
			]],
			functions={[[
			static int rank()
			static int size()
			static void send(const char* msg, int i)
			static std::string receive(int i)
			static int source()
			static void test()
			static void test2()
			]]},
			enums={
			{"ANY_SOURCE","MPI_ANY_SOURCE"},
			{"ANY_TAG","MPI_ANY_TAG"}
			}
		},
	},
}
function generate()
	loadDefinitionDB(script_path..'/luna_baselib_db.lua')
	loadDefinitionDB(script_path..'/../../PhysicsLib/luna_physicslib_db.lua')
	buildDefinitionDB(bindTargetClassification)
	write(
	[[
	#include "stdafx.h"
	#include <cstring>
	#include <string>
	#include <cstdio>
	#include <iostream>
	#include <cstdlib>
	#include <sstream>
	//#include "GPDservo.h"

	//#define INCLUDE_PHYSX // to enable physX, also uncomment the dofile line above.
	#ifdef INCLUDE_PHYSX
	namespace OpenHRP {
		class DynamicsSimulator_physX; 
	}
	#endif
	]]);
	writeIncludeBlock()
	write('#include "../../MainLib/WrapperLua/luna.h"')
	write('#include "../../MainLib/WrapperLua/luna_baselib.h"')
	write('#include "../../PhysicsLib/luna_physics.h"')
	write([[
	#include "MainlibPython.h"
	#include "../../Samples/QP_controller/quadprog.h"
	#ifdef INCLUDE_PHYSX
	#include "../../PhysicsLib/PhysX_implementation/DynamicsSimulator_physX.h"
	#endif
	#include "Lee2013MuscleDyn.h"
	#include "../../PhysicsLib/TRL/eigenSupport.h"
	]])
	--write('#include "KNearestInterpolationFast.h"')
	writeHeader(bindTargetClassification)
	--flushWritten(source_path..'/luna_QP.h') -- write to cpp file only when there exist modifications -> no-recompile.
	write([[
	//#include "luna_QP.h"
	#ifdef USE_MPI
	#include <mpi.h>
	static  MPI_Status status;
	#endif
	#include "../../Samples/classification/cma/CMAwrap.h"
	]])
	writeDefinitions(bindTargetClassification, 'Register_QP') -- input bindTarget can be non-overlapping subset of entire bindTarget 
	flushWritten(source_path..'/luna_QP.cpp') -- write to cpp file only when there exist modifications -> no-recompile.
end
