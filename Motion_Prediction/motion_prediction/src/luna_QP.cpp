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
	
		#include <stdio.h>
		#include <string.h>
		#include <string>
		extern "C"
		{
			#include "lua.h"
			#include "lualib.h"
			#include "lauxlib.h"
			//#include "luadebug.h"
		}
		
#include "../../MainLib/WrapperLua/luna.h"
#include "../../MainLib/WrapperLua/luna_baselib.h"
#include "../../PhysicsLib/luna_physics.h"
	#include "MainlibPython.h"
	#include "../../Samples/QP_controller/quadprog.h"
	#ifdef INCLUDE_PHYSX
	#include "../../PhysicsLib/PhysX_implementation/DynamicsSimulator_physX.h"
	#endif
	#include "Lee2013MuscleDyn.h"
	#include "../../PhysicsLib/TRL/eigenSupport.h"
	
#ifndef genlua_luna_QP_lua15473501_def12                      // 1303
#define genlua_luna_QP_lua15473501_def12                      // 1304
// Declare all classes before including this file or using the "decl" property in the luna_gen definition file. // 1305
// e.g. class LMat; class LMatView; ....                      // 1306
// The forward declaration is not automatically made because luna_gen cannot distinguish struct, class, or namespace. // 1307
// But you can declare a class explictly by using something like decl='class AAA;'  // 1308
// Also, you can add a "#include" sentence by using headerFile="AAA.h"  // 1309
// The number at the end of each line denotes the line number of luna_gen.lua which generated that line // 1312
                namespace OBJloader
                {
                    class ConvexDecomp;
                }
                                                              // 1316
class CMAwrap;                                                // 1316
template<>                                                    // 1327
 class LunaTraits<OBJloader ::ConvexDecomp > {
public:                                                       // 1329
    static const char className[];                            // 1338
    static const int uniqueID;                                // 1339
    static luna_RegType methods[];                            // 1340
    static OBJloader ::ConvexDecomp* _bind_ctor(lua_State *L); // 1342
    static void _bind_dtor(OBJloader ::ConvexDecomp* obj);    // 1343
    typedef OBJloader ::ConvexDecomp base_t;                  // 1345
};                                                            // 1351
template<>                                                    // 1327
 class LunaTraits<CMAwrap > {
public:                                                       // 1329
    static const char className[];                            // 1338
    static const int uniqueID;                                // 1339
    static luna_RegType methods[];                            // 1340
    static CMAwrap* _bind_ctor(lua_State *L);                 // 1342
    static void _bind_dtor(CMAwrap* obj);                     // 1343
    typedef CMAwrap base_t;                                   // 1345
};                                                            // 1351
template<>                                                    // 1327
 class LunaTraits<Eigen ::VectorXd > {
public:                                                       // 1329
    static const char className[];                            // 1338
    static const int uniqueID;                                // 1339
    static luna_RegType methods[];                            // 1340
    static Eigen ::VectorXd* _bind_ctor(lua_State *L);        // 1342
    static void _bind_dtor(Eigen ::VectorXd* obj);            // 1343
    typedef Eigen ::VectorXd base_t;                          // 1345
};                                                            // 1351
template<>                                                    // 1327
 class LunaTraits<path_points > {
public:                                                       // 1329
    static const char className[];                            // 1338
    static const int uniqueID;                                // 1339
    static luna_RegType methods[];                            // 1340
    static path_points* _bind_ctor(lua_State *L);             // 1342
    static void _bind_dtor(path_points* obj);                 // 1343
    typedef path_points base_t;                               // 1345
};                                                            // 1351
template<>                                                    // 1327
 class LunaTraits<muscle_data > {
public:                                                       // 1329
    static const char className[];                            // 1338
    static const int uniqueID;                                // 1339
    static luna_RegType methods[];                            // 1340
    static muscle_data* _bind_ctor(lua_State *L);             // 1342
    static void _bind_dtor(muscle_data* obj);                 // 1343
    typedef muscle_data base_t;                               // 1345
};                                                            // 1351
 class luna__interface_15473501_Eigen {
public:                                                       // 1329
    static const char moduleName[];                           // 1334
    typedef LunaModule<luna__interface_15473501_Eigen> luna_t; // 1335
    static luna_RegType methods[];                            // 1336
};                                                            // 1351
#if defined (USE_MPI)                                         // 1323
 class luna__interface_15473501_MPI {
public:                                                       // 1329
    static const char moduleName[];                           // 1334
    typedef LunaModule<luna__interface_15473501_MPI> luna_t;  // 1335
    static luna_RegType methods[];                            // 1336
};                                                            // 1351
#endif //defined (USE_MPI)                                    // 1353
#endif                                                        // 1356
	//#include "luna_QP.h"
	#ifdef USE_MPI
	#include <mpi.h>
	static  MPI_Status status;
	#endif
	#include "../../Samples/classification/cma/CMAwrap.h"
	
#include "v-hacd/ConvexDecomp.h"
                                                              // 1380
template<>                                                    // 1389
 class impl_LunaTraits<OBJloader ::ConvexDecomp > {
public:                                                       // 1392
    typedef Luna<OBJloader ::ConvexDecomp > luna_t;           // 1396
// : number denotes the line number of luna_gen.lua that generated the sentence // 1399
  inline static bool _lg_typecheck_ctor_overload_1(lua_State *L)
  {                                                           // 1408
    if( lua_gettop(L)!=2) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=90129155) return false; // OBJloader ::Mesh // 629
    if( lua_isboolean(L,2)==0) return false;                  // 641
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_ctor_overload_2(lua_State *L)
  {                                                           // 1408
    if( lua_gettop(L)!=3) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=90129155) return false; // OBJloader ::Mesh // 629
    if( lua_isboolean(L,2)==0) return false;                  // 641
    if( lua_isnumber(L,3)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_ctor_overload_3(lua_State *L)
  {                                                           // 1408
    if( lua_gettop(L)!=4) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=90129155) return false; // OBJloader ::Mesh // 629
    if( lua_isboolean(L,2)==0) return false;                  // 641
    if( lua_isnumber(L,3)==0) return false;                   // 631
    if( lua_isnumber(L,4)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_ctor_overload_4(lua_State *L)
  {                                                           // 1408
    if( lua_gettop(L)!=5) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=90129155) return false; // OBJloader ::Mesh // 629
    if( lua_isboolean(L,2)==0) return false;                  // 641
    if( lua_isnumber(L,3)==0) return false;                   // 631
    if( lua_isnumber(L,4)==0) return false;                   // 631
    if( lua_isnumber(L,5)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_ctor_overload_5(lua_State *L)
  {                                                           // 1408
    if( lua_gettop(L)!=5) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=90129155) return false; // OBJloader ::Mesh // 629
    if( lua_isboolean(L,2)==0) return false;                  // 641
    if( lua_isnumber(L,3)==0) return false;                   // 631
    if( lua_isnumber(L,4)==0) return false;                   // 631
    if( lua_isnumber(L,5)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_performConvexDecomposition_overload_1(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=1) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=21496888) return false; // OBJloader ::ConvexDecomp // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_performConvexDecomposition_overload_2(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=3) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=21496888) return false; // OBJloader ::ConvexDecomp // 629
    if( Luna<void>::get_uniqueid(L,2)!=99273694) return false; // VoxelGraph ::Image3D // 629
    if( lua_isnumber(L,3)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_centroids(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=1) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=21496888) return false; // OBJloader ::ConvexDecomp // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_numConvex(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=1) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=21496888) return false; // OBJloader ::ConvexDecomp // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_convex(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=2) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=21496888) return false; // OBJloader ::ConvexDecomp // 629
    if( lua_isnumber(L,2)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_getBoundsMin(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=1) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=21496888) return false; // OBJloader ::ConvexDecomp // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_getBoundsMax(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=1) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=21496888) return false; // OBJloader ::ConvexDecomp // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_getDimensions(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=1) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=21496888) return false; // OBJloader ::ConvexDecomp // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_getVoxel_overload_1(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=4) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=21496888) return false; // OBJloader ::ConvexDecomp // 629
    if( lua_isnumber(L,2)==0) return false;                   // 631
    if( lua_isnumber(L,3)==0) return false;                   // 631
    if( lua_isnumber(L,4)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_getVoxel_overload_2(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=2) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=21496888) return false; // OBJloader ::ConvexDecomp // 629
    if( Luna<void>::get_uniqueid(L,2)!=10150151) return false; // vector3 // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_getVoxelSafe(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=2) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=21496888) return false; // OBJloader ::ConvexDecomp // 629
    if( Luna<void>::get_uniqueid(L,2)!=10150151) return false; // vector3 // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_numSurfaceVoxels(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=1) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=21496888) return false; // OBJloader ::ConvexDecomp // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_numInteriorVoxels(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=1) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=21496888) return false; // OBJloader ::ConvexDecomp // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_getSurfaceVoxelIndex(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=2) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=21496888) return false; // OBJloader ::ConvexDecomp // 629
    if( lua_isnumber(L,2)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_getInteriorVoxelIndex(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=2) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=21496888) return false; // OBJloader ::ConvexDecomp // 629
    if( lua_isnumber(L,2)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_getVoxelIndex(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=2) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=21496888) return false; // OBJloader ::ConvexDecomp // 629
    if( Luna<void>::get_uniqueid(L,2)!=10150151) return false; // vector3 // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_getWorldPosition_overload_1(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=2) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=21496888) return false; // OBJloader ::ConvexDecomp // 629
    if( Luna<void>::get_uniqueid(L,2)!=10150151) return false; // vector3 // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_getWorldPosition_overload_2(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=2) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=21496888) return false; // OBJloader ::ConvexDecomp // 629
    if( Luna<void>::get_uniqueid(L,2)!=81608400) return false; // VoxelGraph ::shortvector3 // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_createGraphFromAllOccupantVoxels(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=1) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=21496888) return false; // OBJloader ::ConvexDecomp // 629
    return true;
  }                                                           // 656
  inline static OBJloader ::ConvexDecomp* _bind_ctor_overload_1(lua_State *L)
  {                                                           // 1430
    OBJloader ::Mesh const & _arg1=static_cast<OBJloader ::Mesh &>(*Luna<OBJloader ::Mesh >::check(L,1)); // 568
    bool performConvexDecomposition=(bool)lua_toboolean(L,2); // 579
    return new OBJloader ::ConvexDecomp( _arg1, performConvexDecomposition); // 1435
  }                                                           // 1436
  inline static OBJloader ::ConvexDecomp* _bind_ctor_overload_2(lua_State *L)
  {                                                           // 1430
    OBJloader ::Mesh const & _arg1=static_cast<OBJloader ::Mesh &>(*Luna<OBJloader ::Mesh >::check(L,1)); // 568
    bool performConvexDecomposition=(bool)lua_toboolean(L,2); // 579
    int voxelRes=(int)lua_tonumber(L,3);                      // 576
    return new OBJloader ::ConvexDecomp( _arg1, performConvexDecomposition, voxelRes); // 1435
  }                                                           // 1436
  inline static OBJloader ::ConvexDecomp* _bind_ctor_overload_3(lua_State *L)
  {                                                           // 1430
    OBJloader ::Mesh const & _arg1=static_cast<OBJloader ::Mesh &>(*Luna<OBJloader ::Mesh >::check(L,1)); // 568
    bool performConvexDecomposition=(bool)lua_toboolean(L,2); // 579
    int voxelRes=(int)lua_tonumber(L,3);                      // 576
    int maxConvex=(int)lua_tonumber(L,4);                     // 576
    return new OBJloader ::ConvexDecomp( _arg1, performConvexDecomposition, voxelRes, maxConvex); // 1435
  }                                                           // 1436
  inline static OBJloader ::ConvexDecomp* _bind_ctor_overload_4(lua_State *L)
  {                                                           // 1430
    OBJloader ::Mesh const & _arg1=static_cast<OBJloader ::Mesh &>(*Luna<OBJloader ::Mesh >::check(L,1)); // 568
    bool performConvexDecomposition=(bool)lua_toboolean(L,2); // 579
    int voxelRes=(int)lua_tonumber(L,3);                      // 576
    int maxConvex=(int)lua_tonumber(L,4);                     // 576
    double volumeErr=(double)lua_tonumber(L,5);               // 576
    return new OBJloader ::ConvexDecomp( _arg1, performConvexDecomposition, voxelRes, maxConvex, volumeErr); // 1435
  }                                                           // 1436
  inline static OBJloader ::ConvexDecomp* _bind_ctor_overload_5(lua_State *L)
  {                                                           // 1430
    OBJloader ::Mesh const & _arg1=static_cast<OBJloader ::Mesh &>(*Luna<OBJloader ::Mesh >::check(L,1)); // 568
    bool performConvexDecomposition=(bool)lua_toboolean(L,2); // 579
    int voxelRes=(int)lua_tonumber(L,3);                      // 576
    int maxConvex=(int)lua_tonumber(L,4);                     // 576
    double volumeErr=(double)lua_tonumber(L,5);               // 576
    return new OBJloader ::ConvexDecomp( _arg1, performConvexDecomposition, voxelRes, maxConvex, volumeErr); // 1435
  }                                                           // 1436
  static OBJloader ::ConvexDecomp* _bind_ctor(lua_State *L)
  {                                                           // 233
    if (_lg_typecheck_ctor_overload_1(L)) return _bind_ctor_overload_1(L); // 236
    if (_lg_typecheck_ctor_overload_2(L)) return _bind_ctor_overload_2(L); // 236
    if (_lg_typecheck_ctor_overload_3(L)) return _bind_ctor_overload_3(L); // 236
    if (_lg_typecheck_ctor_overload_4(L)) return _bind_ctor_overload_4(L); // 236
    if (_lg_typecheck_ctor_overload_5(L)) return _bind_ctor_overload_5(L); // 236
    luaL_error(L, "ctor ( cannot find overloads:)\n(OBJloader ::Mesh const & _arg1,bool performConvexDecomposition,)\n(OBJloader ::Mesh const & _arg1,bool performConvexDecomposition,int voxelRes,)\n(OBJloader ::Mesh const & _arg1,bool performConvexDecomposition,int voxelRes,int maxConvex,)\n(OBJloader ::Mesh const & _arg1,bool performConvexDecomposition,int voxelRes,int maxConvex,double volumeErr,)\n(OBJloader ::Mesh const & _arg1,bool performConvexDecomposition,int voxelRes,int maxConvex,double volumeErr,)\n");
                                                              // 243
    return NULL;                                              // 244
  }                                                           // 245
  static int _bind_performConvexDecomposition_overload_1(lua_State *L)
  {                                                           // 1451
    OBJloader ::ConvexDecomp& self=static_cast<OBJloader ::ConvexDecomp &>(*Luna<OBJloader ::ConvexDecomp >::check(L,1)); // 568
    try {                                                     // 295
    self.performConvexDecomposition();                        // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
  static int _bind_performConvexDecomposition_overload_2(lua_State *L)
  {                                                           // 1451
    OBJloader ::ConvexDecomp& self=static_cast<OBJloader ::ConvexDecomp &>(*Luna<OBJloader ::ConvexDecomp >::check(L,1)); // 568
    VoxelGraph ::Image3D const & image=static_cast<VoxelGraph ::Image3D &>(*Luna<VoxelGraph ::Image3D >::check(L,2)); // 568
    int value=(int)lua_tonumber(L,3);                         // 576
    try {                                                     // 295
    self.performConvexDecomposition( image, value);           // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
  static int _bind_centroids(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_centroids(L)) { char msg[]="luna typecheck failed:\n  centroids(OBJloader ::ConvexDecomp& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    OBJloader ::ConvexDecomp& self=static_cast<OBJloader ::ConvexDecomp &>(*Luna<OBJloader ::ConvexDecomp >::check(L,1)); // 568
    try {                                                     // 321
    vector3N const & ret=self.centroids();                    // 322
    Luna<vector3N >::push(L,&ret,false,"_vector3N");          // 323
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 324
    return 1;                                                 // 338
  }                                                           // 374
  static int _bind_numConvex(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_numConvex(L)) { char msg[]="luna typecheck failed:\n  numConvex(OBJloader ::ConvexDecomp& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    OBJloader ::ConvexDecomp& self=static_cast<OBJloader ::ConvexDecomp &>(*Luna<OBJloader ::ConvexDecomp >::check(L,1)); // 568
    try {                                                     // 340
    int ret=self.numConvex();                                 // 341
    lua_pushnumber(L, ret);                                   // 342
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 343
    return 1;                                                 // 344
  }                                                           // 374
  static int _bind_convex(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_convex(L)) { char msg[]="luna typecheck failed:\n  convex(OBJloader ::ConvexDecomp& self,int h,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    OBJloader ::ConvexDecomp& self=static_cast<OBJloader ::ConvexDecomp &>(*Luna<OBJloader ::ConvexDecomp >::check(L,1)); // 568
    int h=(int)lua_tonumber(L,2);                             // 576
    try {                                                     // 321
    OBJloader ::Mesh const & ret=self.convex( h);             // 322
    Luna<OBJloader ::Mesh >::push(L,&ret,false,"_Mesh");      // 323
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 324
    return 1;                                                 // 338
  }                                                           // 374
  static int _bind_getBoundsMin(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_getBoundsMin(L)) { char msg[]="luna typecheck failed:\n  getBoundsMin(OBJloader ::ConvexDecomp& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    OBJloader ::ConvexDecomp& self=static_cast<OBJloader ::ConvexDecomp &>(*Luna<OBJloader ::ConvexDecomp >::check(L,1)); // 568
    try {                                                     // 328
    ::vector3* ret=new ::vector3(self.getBoundsMin());        // 333
    Luna< ::vector3 >::push(L,ret,true,"_vector3");           // 334
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 335
    return 1;                                                 // 338
  }                                                           // 374
  static int _bind_getBoundsMax(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_getBoundsMax(L)) { char msg[]="luna typecheck failed:\n  getBoundsMax(OBJloader ::ConvexDecomp& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    OBJloader ::ConvexDecomp& self=static_cast<OBJloader ::ConvexDecomp &>(*Luna<OBJloader ::ConvexDecomp >::check(L,1)); // 568
    try {                                                     // 328
    ::vector3* ret=new ::vector3(self.getBoundsMax());        // 333
    Luna< ::vector3 >::push(L,ret,true,"_vector3");           // 334
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 335
    return 1;                                                 // 338
  }                                                           // 374
  static int _bind_getDimensions(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_getDimensions(L)) { char msg[]="luna typecheck failed:\n  getDimensions(OBJloader ::ConvexDecomp& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    OBJloader ::ConvexDecomp& self=static_cast<OBJloader ::ConvexDecomp &>(*Luna<OBJloader ::ConvexDecomp >::check(L,1)); // 568
    try {                                                     // 328
    ::vector3* ret=new ::vector3(self.getDimensions());       // 333
    Luna< ::vector3 >::push(L,ret,true,"_vector3");           // 334
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 335
    return 1;                                                 // 338
  }                                                           // 374
  static int _bind_getVoxel_overload_1(lua_State *L)
  {                                                           // 1451
    OBJloader ::ConvexDecomp& self=static_cast<OBJloader ::ConvexDecomp &>(*Luna<OBJloader ::ConvexDecomp >::check(L,1)); // 568
    unsigned int x=(unsigned int)lua_tonumber(L,2);           // 576
    unsigned int y=(unsigned int)lua_tonumber(L,3);           // 576
    unsigned int z=(unsigned int)lua_tonumber(L,4);           // 576
    try {                                                     // 340
    int ret=self.getVoxel( x, y, z);                          // 341
    lua_pushnumber(L, ret);                                   // 342
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 343
    return 1;                                                 // 344
  }                                                           // 374
  static int _bind_getVoxel_overload_2(lua_State *L)
  {                                                           // 1451
    OBJloader ::ConvexDecomp& self=static_cast<OBJloader ::ConvexDecomp &>(*Luna<OBJloader ::ConvexDecomp >::check(L,1)); // 568
    vector3 const & vi=static_cast<vector3 &>(*Luna<vector3 >::check(L,2)); // 568
    try {                                                     // 340
    int ret=self.getVoxel( vi);                               // 341
    lua_pushnumber(L, ret);                                   // 342
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 343
    return 1;                                                 // 344
  }                                                           // 374
  static int _bind_getVoxelSafe(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_getVoxelSafe(L)) { char msg[]="luna typecheck failed:\n  getVoxelSafe(OBJloader ::ConvexDecomp& self,vector3 const & vi,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    OBJloader ::ConvexDecomp& self=static_cast<OBJloader ::ConvexDecomp &>(*Luna<OBJloader ::ConvexDecomp >::check(L,1)); // 568
    vector3 const & vi=static_cast<vector3 &>(*Luna<vector3 >::check(L,2)); // 568
    try {                                                     // 340
    int ret=self.getVoxelSafe( vi);                           // 341
    lua_pushnumber(L, ret);                                   // 342
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 343
    return 1;                                                 // 344
  }                                                           // 374
  static int _bind_numSurfaceVoxels(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_numSurfaceVoxels(L)) { char msg[]="luna typecheck failed:\n  numSurfaceVoxels(OBJloader ::ConvexDecomp& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    OBJloader ::ConvexDecomp& self=static_cast<OBJloader ::ConvexDecomp &>(*Luna<OBJloader ::ConvexDecomp >::check(L,1)); // 568
    try {                                                     // 340
    int ret=self.numSurfaceVoxels();                          // 341
    lua_pushnumber(L, ret);                                   // 342
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 343
    return 1;                                                 // 344
  }                                                           // 374
  static int _bind_numInteriorVoxels(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_numInteriorVoxels(L)) { char msg[]="luna typecheck failed:\n  numInteriorVoxels(OBJloader ::ConvexDecomp& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    OBJloader ::ConvexDecomp& self=static_cast<OBJloader ::ConvexDecomp &>(*Luna<OBJloader ::ConvexDecomp >::check(L,1)); // 568
    try {                                                     // 340
    int ret=self.numInteriorVoxels();                         // 341
    lua_pushnumber(L, ret);                                   // 342
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 343
    return 1;                                                 // 344
  }                                                           // 374
  static int _bind_getSurfaceVoxelIndex(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_getSurfaceVoxelIndex(L)) { char msg[]="luna typecheck failed:\n  getSurfaceVoxelIndex(OBJloader ::ConvexDecomp& self,int isurfvoxel,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    OBJloader ::ConvexDecomp& self=static_cast<OBJloader ::ConvexDecomp &>(*Luna<OBJloader ::ConvexDecomp >::check(L,1)); // 568
    int isurfvoxel=(int)lua_tonumber(L,2);                    // 576
    try {                                                     // 328
    ::vector3* ret=new ::vector3(self.getSurfaceVoxelIndex( isurfvoxel)); // 333
    Luna< ::vector3 >::push(L,ret,true,"_vector3");           // 334
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 335
    return 1;                                                 // 338
  }                                                           // 374
  static int _bind_getInteriorVoxelIndex(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_getInteriorVoxelIndex(L)) { char msg[]="luna typecheck failed:\n  getInteriorVoxelIndex(OBJloader ::ConvexDecomp& self,int isurfvoxel,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    OBJloader ::ConvexDecomp& self=static_cast<OBJloader ::ConvexDecomp &>(*Luna<OBJloader ::ConvexDecomp >::check(L,1)); // 568
    int isurfvoxel=(int)lua_tonumber(L,2);                    // 576
    try {                                                     // 328
    ::vector3* ret=new ::vector3(self.getInteriorVoxelIndex( isurfvoxel)); // 333
    Luna< ::vector3 >::push(L,ret,true,"_vector3");           // 334
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 335
    return 1;                                                 // 338
  }                                                           // 374
  static int _bind_getVoxelIndex(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_getVoxelIndex(L)) { char msg[]="luna typecheck failed:\n  getVoxelIndex(OBJloader ::ConvexDecomp& self,vector3 const & worldPosition,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    OBJloader ::ConvexDecomp& self=static_cast<OBJloader ::ConvexDecomp &>(*Luna<OBJloader ::ConvexDecomp >::check(L,1)); // 568
    vector3 const & worldPosition=static_cast<vector3 &>(*Luna<vector3 >::check(L,2)); // 568
    try {                                                     // 328
    ::vector3* ret=new ::vector3(self.getVoxelIndex( worldPosition)); // 333
    Luna< ::vector3 >::push(L,ret,true,"_vector3");           // 334
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 335
    return 1;                                                 // 338
  }                                                           // 374
  static int _bind_getWorldPosition_overload_1(lua_State *L)
  {                                                           // 1451
    OBJloader ::ConvexDecomp& self=static_cast<OBJloader ::ConvexDecomp &>(*Luna<OBJloader ::ConvexDecomp >::check(L,1)); // 568
    vector3 const & voxelIndex=static_cast<vector3 &>(*Luna<vector3 >::check(L,2)); // 568
    try {                                                     // 328
    ::vector3* ret=new ::vector3(self.getWorldPosition( voxelIndex)); // 333
    Luna< ::vector3 >::push(L,ret,true,"_vector3");           // 334
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 335
    return 1;                                                 // 338
  }                                                           // 374
  static int _bind_getWorldPosition_overload_2(lua_State *L)
  {                                                           // 1451
    OBJloader ::ConvexDecomp& self=static_cast<OBJloader ::ConvexDecomp &>(*Luna<OBJloader ::ConvexDecomp >::check(L,1)); // 568
    VoxelGraph ::shortvector3 const & voxelIndex=static_cast<VoxelGraph ::shortvector3 &>(*Luna<VoxelGraph ::shortvector3 >::check(L,2)); // 568
    try {                                                     // 328
    ::vector3* ret=new ::vector3(self.getWorldPosition( voxelIndex)); // 333
    Luna< ::vector3 >::push(L,ret,true,"_vector3");           // 334
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 335
    return 1;                                                 // 338
  }                                                           // 374
  static int _bind_createGraphFromAllOccupantVoxels(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_createGraphFromAllOccupantVoxels(L)) { char msg[]="luna typecheck failed:\n  createGraphFromAllOccupantVoxels(OBJloader ::ConvexDecomp& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    OBJloader ::ConvexDecomp& self=static_cast<OBJloader ::ConvexDecomp &>(*Luna<OBJloader ::ConvexDecomp >::check(L,1)); // 568
    try {                                                     // 311
    VoxelGraph * ret=self.createGraphFromAllOccupantVoxels(); // 312
   if (ret==NULL) lua_pushnil(L); else                        // 313
    Luna<VoxelGraph >::push(L,(VoxelGraph *)ret,true,"_VoxelGraph"); // 314
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 315
    return 1;                                                 // 338
  }                                                           // 374
  static int _bind_getVoxel(lua_State *L)
  {                                                           // 233
    if (_lg_typecheck_getVoxel_overload_1(L)) return _bind_getVoxel_overload_1(L); // 236
    if (_lg_typecheck_getVoxel_overload_2(L)) return _bind_getVoxel_overload_2(L); // 236
    luaL_error(L, "getVoxel ( cannot find overloads:)\n(unsigned int x,unsigned int y,unsigned int z,)\n(vector3 const & vi,)\n");
                                                              // 243
    return 0;                                                 // 244
  }                                                           // 245
  static int _bind_performConvexDecomposition(lua_State *L)
  {                                                           // 233
    if (_lg_typecheck_performConvexDecomposition_overload_1(L)) return _bind_performConvexDecomposition_overload_1(L); // 236
    if (_lg_typecheck_performConvexDecomposition_overload_2(L)) return _bind_performConvexDecomposition_overload_2(L); // 236
    luaL_error(L, "performConvexDecomposition ( cannot find overloads:)\n()\n(VoxelGraph ::Image3D const & image,int value,)\n");
                                                              // 243
    return 0;                                                 // 244
  }                                                           // 245
  static int _bind_getWorldPosition(lua_State *L)
  {                                                           // 233
    if (_lg_typecheck_getWorldPosition_overload_1(L)) return _bind_getWorldPosition_overload_1(L); // 236
    if (_lg_typecheck_getWorldPosition_overload_2(L)) return _bind_getWorldPosition_overload_2(L); // 236
    luaL_error(L, "getWorldPosition ( cannot find overloads:)\n(vector3 const & voxelIndex,)\n(VoxelGraph ::shortvector3 const & voxelIndex,)\n");
                                                              // 243
    return 0;                                                 // 244
  }                                                           // 245
}; // end of class impl_LunaTraits<OBJloader ::ConvexDecomp > // 1612
  OBJloader ::ConvexDecomp* LunaTraits<OBJloader ::ConvexDecomp >::_bind_ctor(lua_State *L)
  {                                                           // 1618
    return impl_LunaTraits<OBJloader ::ConvexDecomp >::_bind_ctor(L); // 1619
  }                                                           // 1620
  void LunaTraits<OBJloader ::ConvexDecomp >::_bind_dtor(OBJloader ::ConvexDecomp* obj){ // 1622
    delete obj;                                               // 1623
  }                                                           // 1624
const char LunaTraits<OBJloader ::ConvexDecomp >::className[] = "_ConvexDecomp"; // 1642
const int LunaTraits<OBJloader ::ConvexDecomp >::uniqueID = 21496888; // 1643
luna_RegType LunaTraits<OBJloader ::ConvexDecomp >::methods[] = { // 1649
    {"centroids", &impl_LunaTraits<OBJloader ::ConvexDecomp >::_bind_centroids}, // 1654
    {"numConvex", &impl_LunaTraits<OBJloader ::ConvexDecomp >::_bind_numConvex}, // 1654
    {"convex", &impl_LunaTraits<OBJloader ::ConvexDecomp >::_bind_convex}, // 1654
    {"getBoundsMin", &impl_LunaTraits<OBJloader ::ConvexDecomp >::_bind_getBoundsMin}, // 1654
    {"getBoundsMax", &impl_LunaTraits<OBJloader ::ConvexDecomp >::_bind_getBoundsMax}, // 1654
    {"getDimensions", &impl_LunaTraits<OBJloader ::ConvexDecomp >::_bind_getDimensions}, // 1654
    {"getVoxelSafe", &impl_LunaTraits<OBJloader ::ConvexDecomp >::_bind_getVoxelSafe}, // 1654
    {"numSurfaceVoxels", &impl_LunaTraits<OBJloader ::ConvexDecomp >::_bind_numSurfaceVoxels}, // 1654
    {"numInteriorVoxels", &impl_LunaTraits<OBJloader ::ConvexDecomp >::_bind_numInteriorVoxels}, // 1654
    {"getSurfaceVoxelIndex", &impl_LunaTraits<OBJloader ::ConvexDecomp >::_bind_getSurfaceVoxelIndex}, // 1654
    {"getInteriorVoxelIndex", &impl_LunaTraits<OBJloader ::ConvexDecomp >::_bind_getInteriorVoxelIndex}, // 1654
    {"getVoxelIndex", &impl_LunaTraits<OBJloader ::ConvexDecomp >::_bind_getVoxelIndex}, // 1654
    {"createGraphFromAllOccupantVoxels", &impl_LunaTraits<OBJloader ::ConvexDecomp >::_bind_createGraphFromAllOccupantVoxels}, // 1654
    {"getVoxel", &impl_LunaTraits<OBJloader ::ConvexDecomp >::_bind_getVoxel}, // 1654
    {"performConvexDecomposition", &impl_LunaTraits<OBJloader ::ConvexDecomp >::_bind_performConvexDecomposition}, // 1654
    {"getWorldPosition", &impl_LunaTraits<OBJloader ::ConvexDecomp >::_bind_getWorldPosition}, // 1654
    {0,0}                                                     // 1657
};                                                            // 1658
template<>                                                    // 1389
 class impl_LunaTraits<CMAwrap > {
public:                                                       // 1392
    typedef Luna<CMAwrap > luna_t;                            // 1396
// : number denotes the line number of luna_gen.lua that generated the sentence // 1399
  inline static bool _lg_typecheck_ctor_overload_1(lua_State *L)
  {                                                           // 1408
    if( lua_gettop(L)!=4) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=10150210) return false; // vectorn // 629
    if( Luna<void>::get_uniqueid(L,2)!=10150210) return false; // vectorn // 629
    if( lua_isnumber(L,3)==0) return false;                   // 631
    if( lua_isnumber(L,4)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_ctor_overload_2(lua_State *L)
  {                                                           // 1408
    if( lua_gettop(L)!=3) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=10150210) return false; // vectorn // 629
    if( Luna<void>::get_uniqueid(L,2)!=10150210) return false; // vectorn // 629
    if( lua_isnumber(L,3)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_testForTermination(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=1) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_samplePopulation(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=1) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_numPopulation(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=1) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_dim(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=1) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_getPopulation(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=2) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 629
    if( lua_isnumber(L,2)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_setVal(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=3) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 629
    if( lua_isnumber(L,2)==0) return false;                   // 631
    if( lua_isnumber(L,3)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_resampleSingle(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=2) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 629
    if( lua_isnumber(L,2)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_resampleSingleFrom(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=3) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 629
    if( lua_isnumber(L,2)==0) return false;                   // 631
    if( Luna<void>::get_uniqueid(L,3)!=10150210) return false; // vectorn // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_update(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=1) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_getMean(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=2) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 629
    if( Luna<void>::get_uniqueid(L,2)!=10150210) return false; // vectorn // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_getBest(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=2) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 629
    if( Luna<void>::get_uniqueid(L,2)!=10150210) return false; // vectorn // 629
    return true;
  }                                                           // 656
  inline static CMAwrap* _bind_ctor_overload_1(lua_State *L)
  {                                                           // 1430
    vectorn const & start_p=static_cast<vectorn &>(*Luna<vectorn >::check(L,1)); // 568
    vectorn const & stdev=static_cast<vectorn &>(*Luna<vectorn >::check(L,2)); // 568
    int populationSize=(int)lua_tonumber(L,3);                // 576
    int mu=(int)lua_tonumber(L,4);                            // 576
    return new CMAwrap( start_p, stdev, populationSize, mu);  // 1435
  }                                                           // 1436
  inline static CMAwrap* _bind_ctor_overload_2(lua_State *L)
  {                                                           // 1430
    vectorn const & start_p=static_cast<vectorn &>(*Luna<vectorn >::check(L,1)); // 568
    vectorn const & stdev=static_cast<vectorn &>(*Luna<vectorn >::check(L,2)); // 568
    int populationSize=(int)lua_tonumber(L,3);                // 576
    return new CMAwrap( start_p, stdev, populationSize);      // 1435
  }                                                           // 1436
  static CMAwrap* _bind_ctor(lua_State *L)
  {                                                           // 233
    if (_lg_typecheck_ctor_overload_1(L)) return _bind_ctor_overload_1(L); // 236
    if (_lg_typecheck_ctor_overload_2(L)) return _bind_ctor_overload_2(L); // 236
    luaL_error(L, "ctor ( cannot find overloads:)\n(vectorn const & start_p,vectorn const & stdev,int populationSize,int mu,)\n(vectorn const & start_p,vectorn const & stdev,int populationSize,)\n");
                                                              // 243
    return NULL;                                              // 244
  }                                                           // 245
  static int _bind_testForTermination(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_testForTermination(L)) { char msg[]="luna typecheck failed:\n  testForTermination(CMAwrap& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 568
    try {                                                     // 346
    std ::string ret=self.testForTermination();               // 347
    lua_pushstring(L, ret.c_str());                           // 350
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 354
    return 1;                                                 // 355
  }                                                           // 374
  static int _bind_samplePopulation(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_samplePopulation(L)) { char msg[]="luna typecheck failed:\n  samplePopulation(CMAwrap& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 568
    try {                                                     // 295
    self.samplePopulation();                                  // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
  static int _bind_numPopulation(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_numPopulation(L)) { char msg[]="luna typecheck failed:\n  numPopulation(CMAwrap& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 568
    try {                                                     // 340
    int ret=self.numPopulation();                             // 341
    lua_pushnumber(L, ret);                                   // 342
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 343
    return 1;                                                 // 344
  }                                                           // 374
  static int _bind_dim(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_dim(L)) { char msg[]="luna typecheck failed:\n  dim(CMAwrap& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 568
    try {                                                     // 340
    int ret=self.dim();                                       // 341
    lua_pushnumber(L, ret);                                   // 342
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 343
    return 1;                                                 // 344
  }                                                           // 374
  static int _bind_getPopulation(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_getPopulation(L)) { char msg[]="luna typecheck failed:\n  getPopulation(CMAwrap& self,int i,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 568
    int i=(int)lua_tonumber(L,2);                             // 576
    try {                                                     // 328
    ::vectornView* ret=new ::vectornView(self.getPopulation( i)); // 333
    Luna< ::vectorn >::push(L,ret,true,"_vectornView");       // 334
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 335
    return 1;                                                 // 338
  }                                                           // 374
  static int _bind_setVal(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_setVal(L)) { char msg[]="luna typecheck failed:\n  setVal(CMAwrap& self,int i,double eval,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 568
    int i=(int)lua_tonumber(L,2);                             // 576
    double eval=(double)lua_tonumber(L,3);                    // 576
    try {                                                     // 295
    self.setVal( i, eval);                                    // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
  static int _bind_resampleSingle(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_resampleSingle(L)) { char msg[]="luna typecheck failed:\n  resampleSingle(CMAwrap& self,int i,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 568
    int i=(int)lua_tonumber(L,2);                             // 576
    try {                                                     // 295
    self.resampleSingle( i);                                  // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
  static int _bind_resampleSingleFrom(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_resampleSingleFrom(L)) { char msg[]="luna typecheck failed:\n  resampleSingleFrom(CMAwrap& self,int i,vectorn const & _arg2,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 568
    int i=(int)lua_tonumber(L,2);                             // 576
    vectorn const & _arg2=static_cast<vectorn &>(*Luna<vectorn >::check(L,3)); // 568
    try {                                                     // 295
    self.resampleSingleFrom( i, _arg2);                       // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
  static int _bind_update(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_update(L)) { char msg[]="luna typecheck failed:\n  update(CMAwrap& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 568
    try {                                                     // 295
    self.update();                                            // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
  static int _bind_getMean(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_getMean(L)) { char msg[]="luna typecheck failed:\n  getMean(CMAwrap& self,vectorn & out,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 568
    vectorn & out=static_cast<vectorn &>(*Luna<vectorn >::check(L,2)); // 568
    try {                                                     // 295
    self.getMean( out);                                       // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
  static int _bind_getBest(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_getBest(L)) { char msg[]="luna typecheck failed:\n  getBest(CMAwrap& self,vectorn & out,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 568
    vectorn & out=static_cast<vectorn &>(*Luna<vectorn >::check(L,2)); // 568
    try {                                                     // 295
    self.getBest( out);                                       // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
}; // end of class impl_LunaTraits<CMAwrap >                  // 1612
  CMAwrap* LunaTraits<CMAwrap >::_bind_ctor(lua_State *L)
  {                                                           // 1618
    return impl_LunaTraits<CMAwrap >::_bind_ctor(L);          // 1619
  }                                                           // 1620
  void LunaTraits<CMAwrap >::_bind_dtor(CMAwrap* obj){        // 1622
    delete obj;                                               // 1623
  }                                                           // 1624
const char LunaTraits<CMAwrap >::className[] = "math_CMAwrap"; // 1642
const int LunaTraits<CMAwrap >::uniqueID = 64707780;          // 1643
luna_RegType LunaTraits<CMAwrap >::methods[] = {              // 1649
    {"testForTermination", &impl_LunaTraits<CMAwrap >::_bind_testForTermination}, // 1654
    {"samplePopulation", &impl_LunaTraits<CMAwrap >::_bind_samplePopulation}, // 1654
    {"numPopulation", &impl_LunaTraits<CMAwrap >::_bind_numPopulation}, // 1654
    {"dim", &impl_LunaTraits<CMAwrap >::_bind_dim},           // 1654
    {"getPopulation", &impl_LunaTraits<CMAwrap >::_bind_getPopulation}, // 1654
    {"setVal", &impl_LunaTraits<CMAwrap >::_bind_setVal},     // 1654
    {"resampleSingle", &impl_LunaTraits<CMAwrap >::_bind_resampleSingle}, // 1654
    {"resampleSingleFrom", &impl_LunaTraits<CMAwrap >::_bind_resampleSingleFrom}, // 1654
    {"update", &impl_LunaTraits<CMAwrap >::_bind_update},     // 1654
    {"getMean", &impl_LunaTraits<CMAwrap >::_bind_getMean},   // 1654
    {"getBest", &impl_LunaTraits<CMAwrap >::_bind_getBest},   // 1654
    {0,0}                                                     // 1657
};                                                            // 1658
template<>                                                    // 1389
 class impl_LunaTraits<Eigen ::VectorXd > {
public:                                                       // 1392
    typedef Luna<Eigen ::VectorXd > luna_t;                   // 1396
// : number denotes the line number of luna_gen.lua that generated the sentence // 1399
  inline static bool _lg_typecheck_ctor_overload_1(lua_State *L)
  {                                                           // 1408
    if( lua_gettop(L)!=0) return false;                       // 621
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_ctor_overload_2(lua_State *L)
  {                                                           // 1408
    if( lua_gettop(L)!=1) return false;                       // 621
    if( lua_isnumber(L,1)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_vecView(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=1) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=11065442) return false; // Eigen ::VectorXd // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_assign_overload_1(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=2) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,2)!=11065442) return false; // Eigen ::VectorXd // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_assign_overload_2(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=2) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,2)!=10150210) return false; // vectorn // 629
    return true;
  }                                                           // 656
  inline static Eigen ::VectorXd* _bind_ctor_overload_1(lua_State *L)
  {                                                           // 1430
    return new Eigen ::VectorXd();                            // 1435
  }                                                           // 1436
  inline static Eigen ::VectorXd* _bind_ctor_overload_2(lua_State *L)
  {                                                           // 1430
    int _arg1=(int)lua_tonumber(L,1);                         // 576
    return new Eigen ::VectorXd( _arg1);                      // 1435
  }                                                           // 1436
  static Eigen ::VectorXd* _bind_ctor(lua_State *L)
  {                                                           // 233
    if (_lg_typecheck_ctor_overload_1(L)) return _bind_ctor_overload_1(L); // 236
    if (_lg_typecheck_ctor_overload_2(L)) return _bind_ctor_overload_2(L); // 236
    luaL_error(L, "ctor ( cannot find overloads:)\n()\n(int _arg1,)\n");
                                                              // 243
    return NULL;                                              // 244
  }                                                           // 245
            inline static void assign(Eigen::VectorXd & out, Eigen::VectorXd const& other)
            {
                out=other;
            }
            inline static void assign(Eigen::VectorXd & out, vectorn const& other)
            {
                out=eigenView(other);
            }
                                                              // 1444
  static int _bind_vecView(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_vecView(L)) { char msg[]="luna typecheck failed:\n  vecView(Eigen ::VectorXd const & x,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    Eigen ::VectorXd const & x=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,1)); // 568
    try {                                                     // 328
    ::vectornView* ret=new ::vectornView(vecView( x));        // 333
    Luna< ::vectorn >::push(L,ret,true,"_vectornView");       // 334
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 335
    return 1;                                                 // 338
  }                                                           // 374
  static int _bind_assign_overload_1(lua_State *L)
  {                                                           // 1451
    Eigen ::VectorXd & out=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,1)); // 568
    Eigen ::VectorXd const & other=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,2)); // 568
    try {                                                     // 295
    assign( out, other);                                      // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
  static int _bind_assign_overload_2(lua_State *L)
  {                                                           // 1451
    Eigen ::VectorXd & out=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,1)); // 568
    vectorn const & other=static_cast<vectorn &>(*Luna<vectorn >::check(L,2)); // 568
    try {                                                     // 295
    assign( out, other);                                      // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
  static int _bind_assign(lua_State *L)
  {                                                           // 233
    if (_lg_typecheck_assign_overload_1(L)) return _bind_assign_overload_1(L); // 236
    if (_lg_typecheck_assign_overload_2(L)) return _bind_assign_overload_2(L); // 236
    luaL_error(L, "assign ( cannot find overloads:)\n(Eigen ::VectorXd & out,Eigen ::VectorXd const & other,)\n(Eigen ::VectorXd & out,vectorn const & other,)\n");
                                                              // 243
    return 0;                                                 // 244
  }                                                           // 245
}; // end of class impl_LunaTraits<Eigen ::VectorXd >         // 1612
  Eigen ::VectorXd* LunaTraits<Eigen ::VectorXd >::_bind_ctor(lua_State *L)
  {                                                           // 1618
    return impl_LunaTraits<Eigen ::VectorXd >::_bind_ctor(L); // 1619
  }                                                           // 1620
  void LunaTraits<Eigen ::VectorXd >::_bind_dtor(Eigen ::VectorXd* obj){ // 1622
    delete obj;                                               // 1623
  }                                                           // 1624
const char LunaTraits<Eigen ::VectorXd >::className[] = "_VectorXd"; // 1642
const int LunaTraits<Eigen ::VectorXd >::uniqueID = 11065442; // 1643
luna_RegType LunaTraits<Eigen ::VectorXd >::methods[] = {     // 1649
    {"vecView", &impl_LunaTraits<Eigen ::VectorXd >::_bind_vecView}, // 1654
    {"assign", &impl_LunaTraits<Eigen ::VectorXd >::_bind_assign}, // 1654
    {0,0}                                                     // 1657
};                                                            // 1658
template<>                                                    // 1389
 class impl_LunaTraits<path_points > {
public:                                                       // 1392
    typedef Luna<path_points > luna_t;                        // 1396
// : number denotes the line number of luna_gen.lua that generated the sentence // 1399
  inline static bool _lg_typecheck_ctor(lua_State *L)
  {                                                           // 1408
    if( lua_gettop(L)!=2) return false;                       // 621
    if( lua_isnumber(L,1)==0) return false;                   // 631
    if( lua_isnumber(L,2)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_setPathPoint(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=8) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=13989110) return false; // path_points // 629
    if( lua_isnumber(L,2)==0) return false;                   // 631
    if( lua_isnumber(L,3)==0) return false;                   // 631
    if( lua_isnumber(L,4)==0) return false;                   // 631
    if( lua_isnumber(L,5)==0) return false;                   // 631
    if( lua_isnumber(L,6)==0) return false;                   // 631
    if( lua_isnumber(L,7)==0) return false;                   // 631
    if( Luna<void>::get_uniqueid(L,8)!=10150151) return false; // vector3 // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_updatePPindices(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=3) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=13989110) return false; // path_points // 629
    if( Luna<void>::get_uniqueid(L,2)!=86922217) return false; // BoneForwardKinematics // 629
    if( Luna<void>::get_uniqueid(L,3)!=10150210) return false; // vectorn // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_get_eppis(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=2) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=13989110) return false; // path_points // 629
    if( lua_isnumber(L,2)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_get_vppis(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=2) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=13989110) return false; // path_points // 629
    if( lua_isnumber(L,2)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_getPathPointPos(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=2) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=13989110) return false; // path_points // 629
    if( lua_isnumber(L,2)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_get_lm_t(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=1) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=13989110) return false; // path_points // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_applyMuscleForces(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=4) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=13989110) return false; // path_points // 629
    if( lua_isnumber(L,2)==0) return false;                   // 631
    if( Luna<void>::get_uniqueid(L,3)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,4)!=66780458) return false; // OpenHRP ::DynamicsSimulator // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck__calcPathPointPositions(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=3) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=13989110) return false; // path_points // 629
    if( Luna<void>::get_uniqueid(L,2)!=86922217) return false; // BoneForwardKinematics // 629
    if( Luna<void>::get_uniqueid(L,3)!=14654042) return false; // vector3N // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck__calcTendonMuscleLengths(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=3) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=13989110) return false; // path_points // 629
    if( Luna<void>::get_uniqueid(L,2)!=14654042) return false; // vector3N // 629
    if( Luna<void>::get_uniqueid(L,3)!=11065442) return false; // Eigen ::VectorXd // 629
    return true;
  }                                                           // 656
  inline static path_points* _bind_ctor(lua_State *L)
  {                                                           // 1430
    if (!_lg_typecheck_ctor(L)) { char msg[]="luna typecheck failed:\n  ctor(int num_muscles,int numPathpoints,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    int num_muscles=(int)lua_tonumber(L,1);                   // 576
    int numPathpoints=(int)lua_tonumber(L,2);                 // 576
    return new path_points( num_muscles, numPathpoints);      // 1435
  }                                                           // 1436
  static int _bind_setPathPoint(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_setPathPoint(L)) { char msg[]="luna typecheck failed:\n  setPathPoint(path_points& self,int i,int ti,int mi,int cdof,double r1,double r2,vector3 const & lp,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    path_points& self=static_cast<path_points &>(*Luna<path_points >::check(L,1)); // 568
    int i=(int)lua_tonumber(L,2);                             // 576
    int ti=(int)lua_tonumber(L,3);                            // 576
    int mi=(int)lua_tonumber(L,4);                            // 576
    int cdof=(int)lua_tonumber(L,5);                          // 576
    double r1=(double)lua_tonumber(L,6);                      // 576
    double r2=(double)lua_tonumber(L,7);                      // 576
    vector3 const & lp=static_cast<vector3 &>(*Luna<vector3 >::check(L,8)); // 568
    try {                                                     // 295
    self.setPathPoint( i, ti, mi, cdof, r1, r2, lp);          // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
  static int _bind_updatePPindices(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_updatePPindices(L)) { char msg[]="luna typecheck failed:\n  updatePPindices(path_points& self,BoneForwardKinematics const & fk,vectorn const & posedof,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    path_points& self=static_cast<path_points &>(*Luna<path_points >::check(L,1)); // 568
    BoneForwardKinematics const & fk=static_cast<BoneForwardKinematics &>(*Luna<BoneForwardKinematics >::check(L,2)); // 568
    vectorn const & posedof=static_cast<vectorn &>(*Luna<vectorn >::check(L,3)); // 568
    try {                                                     // 295
    self.updatePPindices( fk, posedof);                       // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
  static int _bind_get_eppis(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_get_eppis(L)) { char msg[]="luna typecheck failed:\n  get_eppis(path_points& self,int imuscle,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    path_points& self=static_cast<path_points &>(*Luna<path_points >::check(L,1)); // 568
    int imuscle=(int)lua_tonumber(L,2);                       // 576
    try {                                                     // 321
    intvectorn const & ret=self.get_eppis( imuscle);          // 322
    Luna<intvectorn >::push(L,&ret,false,"_intvectorn");      // 323
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 324
    return 1;                                                 // 338
  }                                                           // 374
  static int _bind_get_vppis(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_get_vppis(L)) { char msg[]="luna typecheck failed:\n  get_vppis(path_points& self,int imuscle,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    path_points& self=static_cast<path_points &>(*Luna<path_points >::check(L,1)); // 568
    int imuscle=(int)lua_tonumber(L,2);                       // 576
    try {                                                     // 321
    intvectorn const & ret=self.get_vppis( imuscle);          // 322
    Luna<intvectorn >::push(L,&ret,false,"_intvectorn");      // 323
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 324
    return 1;                                                 // 338
  }                                                           // 374
  static int _bind_getPathPointPos(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_getPathPointPos(L)) { char msg[]="luna typecheck failed:\n  getPathPointPos(path_points& self,int i_pp,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    path_points& self=static_cast<path_points &>(*Luna<path_points >::check(L,1)); // 568
    int i_pp=(int)lua_tonumber(L,2);                          // 576
    try {                                                     // 328
    ::vector3* ret=new ::vector3(self.getPathPointPos( i_pp)); // 333
    Luna< ::vector3 >::push(L,ret,true,"_vector3");           // 334
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 335
    return 1;                                                 // 338
  }                                                           // 374
  static int _bind_get_lm_t(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_get_lm_t(L)) { char msg[]="luna typecheck failed:\n  get_lm_t(path_points& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    path_points& self=static_cast<path_points &>(*Luna<path_points >::check(L,1)); // 568
    try {                                                     // 321
    const Eigen ::VectorXd & ret=self.get_lm_t();             // 322
    Luna<Eigen ::VectorXd >::push(L,&ret,false,"_VectorXd");  // 323
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 324
    return 1;                                                 // 338
  }                                                           // 374
  static int _bind_applyMuscleForces(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_applyMuscleForces(L)) { char msg[]="luna typecheck failed:\n  applyMuscleForces(path_points& self,int ichara,Eigen ::VectorXd const & forces,OpenHRP ::DynamicsSimulator & sim,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    path_points& self=static_cast<path_points &>(*Luna<path_points >::check(L,1)); // 568
    int ichara=(int)lua_tonumber(L,2);                        // 576
    Eigen ::VectorXd const & forces=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,3)); // 568
    OpenHRP ::DynamicsSimulator & sim=static_cast<OpenHRP ::DynamicsSimulator &>(*Luna<OpenHRP ::DynamicsSimulator >::check(L,4)); // 568
    try {                                                     // 295
    self.applyMuscleForces( ichara, forces, sim);             // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
  static int _bind__calcPathPointPositions(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck__calcPathPointPositions(L)) { char msg[]="luna typecheck failed:\n  _calcPathPointPositions(path_points& self,BoneForwardKinematics const & fk,vector3N & pathPointPositions,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    path_points& self=static_cast<path_points &>(*Luna<path_points >::check(L,1)); // 568
    BoneForwardKinematics const & fk=static_cast<BoneForwardKinematics &>(*Luna<BoneForwardKinematics >::check(L,2)); // 568
    vector3N & pathPointPositions=static_cast<vector3N &>(*Luna<vector3N >::check(L,3)); // 568
    try {                                                     // 295
    self._calcPathPointPositions( fk, pathPointPositions);    // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
  static int _bind__calcTendonMuscleLengths(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck__calcTendonMuscleLengths(L)) { char msg[]="luna typecheck failed:\n  _calcTendonMuscleLengths(path_points& self,vector3N const & pathPointPositions,Eigen ::VectorXd & lm_t,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    path_points& self=static_cast<path_points &>(*Luna<path_points >::check(L,1)); // 568
    vector3N const & pathPointPositions=static_cast<vector3N &>(*Luna<vector3N >::check(L,2)); // 568
    Eigen ::VectorXd & lm_t=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,3)); // 568
    try {                                                     // 295
    self._calcTendonMuscleLengths( pathPointPositions, lm_t); // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
}; // end of class impl_LunaTraits<path_points >              // 1612
  path_points* LunaTraits<path_points >::_bind_ctor(lua_State *L)
  {                                                           // 1618
    return impl_LunaTraits<path_points >::_bind_ctor(L);      // 1619
  }                                                           // 1620
  void LunaTraits<path_points >::_bind_dtor(path_points* obj){ // 1622
    delete obj;                                               // 1623
  }                                                           // 1624
const char LunaTraits<path_points >::className[] = "_path_points"; // 1642
const int LunaTraits<path_points >::uniqueID = 13989110;      // 1643
luna_RegType LunaTraits<path_points >::methods[] = {          // 1649
    {"setPathPoint", &impl_LunaTraits<path_points >::_bind_setPathPoint}, // 1654
    {"updatePPindices", &impl_LunaTraits<path_points >::_bind_updatePPindices}, // 1654
    {"get_eppis", &impl_LunaTraits<path_points >::_bind_get_eppis}, // 1654
    {"get_vppis", &impl_LunaTraits<path_points >::_bind_get_vppis}, // 1654
    {"getPathPointPos", &impl_LunaTraits<path_points >::_bind_getPathPointPos}, // 1654
    {"get_lm_t", &impl_LunaTraits<path_points >::_bind_get_lm_t}, // 1654
    {"applyMuscleForces", &impl_LunaTraits<path_points >::_bind_applyMuscleForces}, // 1654
    {"_calcPathPointPositions", &impl_LunaTraits<path_points >::_bind__calcPathPointPositions}, // 1654
    {"_calcTendonMuscleLengths", &impl_LunaTraits<path_points >::_bind__calcTendonMuscleLengths}, // 1654
    {0,0}                                                     // 1657
};                                                            // 1658
template<>                                                    // 1389
 class impl_LunaTraits<muscle_data > {
public:                                                       // 1392
    typedef Luna<muscle_data > luna_t;                        // 1396
// : number denotes the line number of luna_gen.lua that generated the sentence // 1399
  inline static bool _lg_typecheck_ctor(lua_State *L)
  {                                                           // 1408
    if( lua_gettop(L)!=15) return false;                      // 621
    if( Luna<void>::get_uniqueid(L,1)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,2)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,3)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,4)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,5)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,6)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,7)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,8)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,9)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,10)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,11)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,12)!=11065442) return false; // Eigen ::VectorXd // 629
    if( lua_isnumber(L,13)==0) return false;                  // 631
    if( Luna<void>::get_uniqueid(L,14)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,15)!=11065442) return false; // Eigen ::VectorXd // 629
    return true;
  }                                                           // 656
  inline static muscle_data* _bind_ctor(lua_State *L)
  {                                                           // 1430
    if (!_lg_typecheck_ctor(L)) { char msg[]="luna typecheck failed:\n  ctor(Eigen ::VectorXd const & l_mt,Eigen ::VectorXd const & l_m_opt,Eigen ::VectorXd const & pa_opt,Eigen ::VectorXd const & l_t_sl,Eigen ::VectorXd const & eps_t_o,Eigen ::VectorXd const & eps_m_o,Eigen ::VectorXd const & k_pe,Eigen ::VectorXd const & gamma,Eigen ::VectorXd const & a_f,Eigen ::VectorXd const & f_m_len,Eigen ::VectorXd const & damping,Eigen ::VectorXd const & v_m_max,double option,Eigen ::VectorXd const & tau_act,Eigen ::VectorXd const & tau_deact,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    Eigen ::VectorXd const & l_mt=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,1)); // 568
    Eigen ::VectorXd const & l_m_opt=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,2)); // 568
    Eigen ::VectorXd const & pa_opt=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,3)); // 568
    Eigen ::VectorXd const & l_t_sl=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,4)); // 568
    Eigen ::VectorXd const & eps_t_o=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,5)); // 568
    Eigen ::VectorXd const & eps_m_o=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,6)); // 568
    Eigen ::VectorXd const & k_pe=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,7)); // 568
    Eigen ::VectorXd const & gamma=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,8)); // 568
    Eigen ::VectorXd const & a_f=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,9)); // 568
    Eigen ::VectorXd const & f_m_len=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,10)); // 568
    Eigen ::VectorXd const & damping=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,11)); // 568
    Eigen ::VectorXd const & v_m_max=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,12)); // 568
    double option=(double)lua_tonumber(L,13);                 // 576
    Eigen ::VectorXd const & tau_act=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,14)); // 568
    Eigen ::VectorXd const & tau_deact=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,15)); // 568
    return new muscle_data( l_mt, l_m_opt, pa_opt, l_t_sl, eps_t_o, eps_m_o, k_pe, gamma, a_f, f_m_len, damping, v_m_max, option, tau_act, tau_deact); // 1435
  }                                                           // 1436
}; // end of class impl_LunaTraits<muscle_data >              // 1612
  muscle_data* LunaTraits<muscle_data >::_bind_ctor(lua_State *L)
  {                                                           // 1618
    return impl_LunaTraits<muscle_data >::_bind_ctor(L);      // 1619
  }                                                           // 1620
  void LunaTraits<muscle_data >::_bind_dtor(muscle_data* obj){ // 1622
    delete obj;                                               // 1623
  }                                                           // 1624
const char LunaTraits<muscle_data >::className[] = "_muscle_data"; // 1642
const int LunaTraits<muscle_data >::uniqueID = 75621382;      // 1643
luna_RegType LunaTraits<muscle_data >::methods[] = {          // 1649
    {0,0}                                                     // 1657
};                                                            // 1658
 class impl_luna__interface_15473501_Eigen {
public:                                                       // 1392
    typedef LunaModule<luna__interface_15473501_Eigen> luna_t; // 1394
// : number denotes the line number of luna_gen.lua that generated the sentence // 1399
  inline static bool _lg_typecheck_solveQuadprog_overload_1(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=6) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=55023510) return false; // HessianQuadratic // 629
    if( Luna<void>::get_uniqueid(L,2)!=23735758) return false; // matrixn // 629
    if( Luna<void>::get_uniqueid(L,3)!=10150210) return false; // vectorn // 629
    if( Luna<void>::get_uniqueid(L,4)!=23735758) return false; // matrixn // 629
    if( Luna<void>::get_uniqueid(L,5)!=10150210) return false; // vectorn // 629
    if( Luna<void>::get_uniqueid(L,6)!=10150210) return false; // vectorn // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_solveQuadprog_overload_2(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=7) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=55023510) return false; // HessianQuadratic // 629
    if( Luna<void>::get_uniqueid(L,2)!=23735758) return false; // matrixn // 629
    if( Luna<void>::get_uniqueid(L,3)!=10150210) return false; // vectorn // 629
    if( Luna<void>::get_uniqueid(L,4)!=23735758) return false; // matrixn // 629
    if( Luna<void>::get_uniqueid(L,5)!=10150210) return false; // vectorn // 629
    if( Luna<void>::get_uniqueid(L,6)!=10150210) return false; // vectorn // 629
    if( lua_isboolean(L,7)==0) return false;                  // 641
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_sigmoid(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=1) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=11065442) return false; // Eigen ::VectorXd // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_computeCosPennation(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=3) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,2)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,3)!=11065442) return false; // Eigen ::VectorXd // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_computeActivationDeriv(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=4) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,2)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,3)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,4)!=11065442) return false; // Eigen ::VectorXd // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_computeNormTendonForce(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=2) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,2)!=11065442) return false; // Eigen ::VectorXd // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_computeNormActiveFiberForce(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=3) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,2)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,3)!=75621382) return false; // muscle_data // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_computePassiveFiberForce(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=4) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,2)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,3)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,4)!=75621382) return false; // muscle_data // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_getFiberLengthDeriv(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=15) return false;                      // 621
    if( Luna<void>::get_uniqueid(L,1)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,2)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,3)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,4)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,5)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,6)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,7)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,8)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,9)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,10)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,11)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,12)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,13)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,14)!=11065442) return false; // Eigen ::VectorXd // 629
    if( lua_isnumber(L,15)==0) return false;                  // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_getIsometricFiberLength(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=13) return false;                      // 621
    if( Luna<void>::get_uniqueid(L,1)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,2)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,3)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,4)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,5)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,6)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,7)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,8)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,9)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,10)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,11)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,12)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,13)!=11065442) return false; // Eigen ::VectorXd // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_clamp_dl_m(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=3) return false;                       // 621
    if( Luna<void>::get_uniqueid(L,1)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,2)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,3)!=11065442) return false; // Eigen ::VectorXd // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_integrateMuscleDynamics(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=7) return false;                       // 621
    if( lua_isnumber(L,1)==0) return false;                   // 631
    if( lua_isnumber(L,2)==0) return false;                   // 631
    if( Luna<void>::get_uniqueid(L,3)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,4)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,5)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,6)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,7)!=75621382) return false; // muscle_data // 629
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_getMetabolicEnergyRate(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=9) return false;                       // 621
    if( lua_isnumber(L,1)==0) return false;                   // 631
    if( Luna<void>::get_uniqueid(L,2)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,3)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,4)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,5)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,6)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,7)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,8)!=11065442) return false; // Eigen ::VectorXd // 629
    if( Luna<void>::get_uniqueid(L,9)!=11065442) return false; // Eigen ::VectorXd // 629
    return true;
  }                                                           // 656
  static int _bind_solveQuadprog_overload_1(lua_State *L)
  {                                                           // 1451
    HessianQuadratic & problem=static_cast<HessianQuadratic &>(*Luna<HessianQuadratic >::check(L,1)); // 568
    const matrixn & CE=static_cast<matrixn &>(*Luna<matrixn >::check(L,2)); // 568
    const vectorn & ce0=static_cast<vectorn &>(*Luna<vectorn >::check(L,3)); // 568
    const matrixn & CI=static_cast<matrixn &>(*Luna<matrixn >::check(L,4)); // 568
    const vectorn & ci0=static_cast<vectorn &>(*Luna<vectorn >::check(L,5)); // 568
    vectorn & x=static_cast<vectorn &>(*Luna<vectorn >::check(L,6)); // 568
    try {                                                     // 340
    double ret=solve_quadprog( problem, CE, ce0, CI, ci0, x); // 341
    lua_pushnumber(L, ret);                                   // 342
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 343
    return 1;                                                 // 344
  }                                                           // 374
  static int _bind_solveQuadprog_overload_2(lua_State *L)
  {                                                           // 1451
    HessianQuadratic & problem=static_cast<HessianQuadratic &>(*Luna<HessianQuadratic >::check(L,1)); // 568
    const matrixn & CE=static_cast<matrixn &>(*Luna<matrixn >::check(L,2)); // 568
    const vectorn & ce0=static_cast<vectorn &>(*Luna<vectorn >::check(L,3)); // 568
    const matrixn & CI=static_cast<matrixn &>(*Luna<matrixn >::check(L,4)); // 568
    const vectorn & ci0=static_cast<vectorn &>(*Luna<vectorn >::check(L,5)); // 568
    vectorn & x=static_cast<vectorn &>(*Luna<vectorn >::check(L,6)); // 568
    bool _arg7=(bool)lua_toboolean(L,7);                      // 579
    try {                                                     // 340
    double ret=solve_quadprog( problem, CE, ce0, CI, ci0, x, _arg7); // 341
    lua_pushnumber(L, ret);                                   // 342
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 343
    return 1;                                                 // 344
  }                                                           // 374
  static int _bind_sigmoid(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_sigmoid(L)) { char msg[]="luna typecheck failed:\n  sigmoid(const Eigen ::VectorXd & m1,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    const Eigen ::VectorXd & m1=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,1)); // 568
    try {                                                     // 328
    ::Eigen ::VectorXd* ret=new ::Eigen ::VectorXd(sigmoid( m1)); // 333
    Luna< ::Eigen ::VectorXd >::push(L,ret,true,"_VectorXd"); // 334
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 335
    return 1;                                                 // 338
  }                                                           // 374
  static int _bind_computeCosPennation(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_computeCosPennation(L)) { char msg[]="luna typecheck failed:\n  computeCosPennation(Eigen ::VectorXd const & l_m,Eigen ::VectorXd const & l_m_opt,Eigen ::VectorXd const & pa_opt,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    Eigen ::VectorXd const & l_m=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,1)); // 568
    Eigen ::VectorXd const & l_m_opt=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,2)); // 568
    Eigen ::VectorXd const & pa_opt=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,3)); // 568
    try {                                                     // 328
    ::Eigen ::VectorXd* ret=new ::Eigen ::VectorXd(computeCosPennation( l_m, l_m_opt, pa_opt)); // 333
    Luna< ::Eigen ::VectorXd >::push(L,ret,true,"_VectorXd"); // 334
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 335
    return 1;                                                 // 338
  }                                                           // 374
  static int _bind_computeActivationDeriv(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_computeActivationDeriv(L)) { char msg[]="luna typecheck failed:\n  computeActivationDeriv(Eigen ::VectorXd const & u,Eigen ::VectorXd const & a,Eigen ::VectorXd const & tau_act,Eigen ::VectorXd const & tau_deact,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    Eigen ::VectorXd const & u=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,1)); // 568
    Eigen ::VectorXd const & a=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,2)); // 568
    Eigen ::VectorXd const & tau_act=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,3)); // 568
    Eigen ::VectorXd const & tau_deact=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,4)); // 568
    try {                                                     // 328
    ::Eigen ::VectorXd* ret=new ::Eigen ::VectorXd(computeActivationDeriv( u, a, tau_act, tau_deact)); // 333
    Luna< ::Eigen ::VectorXd >::push(L,ret,true,"_VectorXd"); // 334
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 335
    return 1;                                                 // 338
  }                                                           // 374
  static int _bind_computeNormTendonForce(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_computeNormTendonForce(L)) { char msg[]="luna typecheck failed:\n  computeNormTendonForce(Eigen ::VectorXd const & eps_t,Eigen ::VectorXd const & eps_t_o,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    Eigen ::VectorXd const & eps_t=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,1)); // 568
    Eigen ::VectorXd const & eps_t_o=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,2)); // 568
    try {                                                     // 328
    ::Eigen ::VectorXd* ret=new ::Eigen ::VectorXd(computeNormTendonForce( eps_t, eps_t_o)); // 333
    Luna< ::Eigen ::VectorXd >::push(L,ret,true,"_VectorXd"); // 334
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 335
    return 1;                                                 // 338
  }                                                           // 374
  static int _bind_computeNormActiveFiberForce(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_computeNormActiveFiberForce(L)) { char msg[]="luna typecheck failed:\n  computeNormActiveFiberForce(Eigen ::VectorXd const & norm_l_m_prev,Eigen ::VectorXd const & norm_dl_m_prev,muscle_data const & d,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    Eigen ::VectorXd const & norm_l_m_prev=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,1)); // 568
    Eigen ::VectorXd const & norm_dl_m_prev=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,2)); // 568
    muscle_data const & d=static_cast<muscle_data &>(*Luna<muscle_data >::check(L,3)); // 568
    try {                                                     // 328
    ::Eigen ::VectorXd* ret=new ::Eigen ::VectorXd(computeNormActiveFiberForce( norm_l_m_prev, norm_dl_m_prev, d)); // 333
    Luna< ::Eigen ::VectorXd >::push(L,ret,true,"_VectorXd"); // 334
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 335
    return 1;                                                 // 338
  }                                                           // 374
  static int _bind_computePassiveFiberForce(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_computePassiveFiberForce(L)) { char msg[]="luna typecheck failed:\n  computePassiveFiberForce(Eigen ::VectorXd const & f_m_o,Eigen ::VectorXd const & norm_l_m_prev,Eigen ::VectorXd const & dl_m_prev,muscle_data const & d,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    Eigen ::VectorXd const & f_m_o=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,1)); // 568
    Eigen ::VectorXd const & norm_l_m_prev=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,2)); // 568
    Eigen ::VectorXd const & dl_m_prev=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,3)); // 568
    muscle_data const & d=static_cast<muscle_data &>(*Luna<muscle_data >::check(L,4)); // 568
    try {                                                     // 328
    ::Eigen ::VectorXd* ret=new ::Eigen ::VectorXd(computePassiveFiberForce( f_m_o, norm_l_m_prev, dl_m_prev, d)); // 333
    Luna< ::Eigen ::VectorXd >::push(L,ret,true,"_VectorXd"); // 334
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 335
    return 1;                                                 // 338
  }                                                           // 374
  static int _bind_getFiberLengthDeriv(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_getFiberLengthDeriv(L)) { char msg[]="luna typecheck failed:\n  getFiberLengthDeriv(Eigen ::VectorXd const & a,Eigen ::VectorXd const & l_m,Eigen ::VectorXd const & l_mt,Eigen ::VectorXd const & l_m_opt,Eigen ::VectorXd const & pa_opt,Eigen ::VectorXd const & l_t_sl,Eigen ::VectorXd const & eps_t_o,Eigen ::VectorXd const & eps_m_o,Eigen ::VectorXd const & k_pe,Eigen ::VectorXd const & gamma,Eigen ::VectorXd const & a_f,Eigen ::VectorXd const & f_m_len,Eigen ::VectorXd const & damping,Eigen ::VectorXd const & v_m_max,double option,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    Eigen ::VectorXd const & a=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,1)); // 568
    Eigen ::VectorXd const & l_m=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,2)); // 568
    Eigen ::VectorXd const & l_mt=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,3)); // 568
    Eigen ::VectorXd const & l_m_opt=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,4)); // 568
    Eigen ::VectorXd const & pa_opt=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,5)); // 568
    Eigen ::VectorXd const & l_t_sl=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,6)); // 568
    Eigen ::VectorXd const & eps_t_o=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,7)); // 568
    Eigen ::VectorXd const & eps_m_o=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,8)); // 568
    Eigen ::VectorXd const & k_pe=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,9)); // 568
    Eigen ::VectorXd const & gamma=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,10)); // 568
    Eigen ::VectorXd const & a_f=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,11)); // 568
    Eigen ::VectorXd const & f_m_len=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,12)); // 568
    Eigen ::VectorXd const & damping=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,13)); // 568
    Eigen ::VectorXd const & v_m_max=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,14)); // 568
    double option=(double)lua_tonumber(L,15);                 // 576
    try {                                                     // 328
    ::Eigen ::VectorXd* ret=new ::Eigen ::VectorXd(getFiberLengthDeriv( a, l_m, l_mt, l_m_opt, pa_opt, l_t_sl, eps_t_o, eps_m_o, k_pe, gamma, a_f, f_m_len, damping, v_m_max, option)); // 333
    Luna< ::Eigen ::VectorXd >::push(L,ret,true,"_VectorXd"); // 334
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 335
    return 1;                                                 // 338
  }                                                           // 374
  static int _bind_getIsometricFiberLength(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_getIsometricFiberLength(L)) { char msg[]="luna typecheck failed:\n  getIsometricFiberLength(Eigen ::VectorXd const & a,Eigen ::VectorXd const & l_mt,Eigen ::VectorXd const & l_m_opt,Eigen ::VectorXd const & pa_opt,Eigen ::VectorXd const & l_t_sl,Eigen ::VectorXd const & eps_t_o,Eigen ::VectorXd const & eps_m_o,Eigen ::VectorXd const & k_pe,Eigen ::VectorXd const & gamma,Eigen ::VectorXd const & a_f,Eigen ::VectorXd const & f_m_len,Eigen ::VectorXd const & damping,Eigen ::VectorXd const & v_m_max,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    Eigen ::VectorXd const & a=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,1)); // 568
    Eigen ::VectorXd const & l_mt=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,2)); // 568
    Eigen ::VectorXd const & l_m_opt=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,3)); // 568
    Eigen ::VectorXd const & pa_opt=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,4)); // 568
    Eigen ::VectorXd const & l_t_sl=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,5)); // 568
    Eigen ::VectorXd const & eps_t_o=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,6)); // 568
    Eigen ::VectorXd const & eps_m_o=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,7)); // 568
    Eigen ::VectorXd const & k_pe=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,8)); // 568
    Eigen ::VectorXd const & gamma=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,9)); // 568
    Eigen ::VectorXd const & a_f=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,10)); // 568
    Eigen ::VectorXd const & f_m_len=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,11)); // 568
    Eigen ::VectorXd const & damping=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,12)); // 568
    Eigen ::VectorXd const & v_m_max=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,13)); // 568
    try {                                                     // 328
    ::Eigen ::VectorXd* ret=new ::Eigen ::VectorXd(getIsometricFiberLength( a, l_mt, l_m_opt, pa_opt, l_t_sl, eps_t_o, eps_m_o, k_pe, gamma, a_f, f_m_len, damping, v_m_max)); // 333
    Luna< ::Eigen ::VectorXd >::push(L,ret,true,"_VectorXd"); // 334
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 335
    return 1;                                                 // 338
  }                                                           // 374
  static int _bind_clamp_dl_m(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_clamp_dl_m(L)) { char msg[]="luna typecheck failed:\n  clamp_dl_m(Eigen ::VectorXd & dl_m,Eigen ::VectorXd const & l_m_opt,Eigen ::VectorXd const & v_m_max,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    Eigen ::VectorXd & dl_m=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,1)); // 568
    Eigen ::VectorXd const & l_m_opt=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,2)); // 568
    Eigen ::VectorXd const & v_m_max=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,3)); // 568
    try {                                                     // 295
    clamp_dl_m( dl_m, l_m_opt, v_m_max);                      // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
  static int _bind_integrateMuscleDynamics(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_integrateMuscleDynamics(L)) { char msg[]="luna typecheck failed:\n  integrateMuscleDynamics(double duration,double muscleTimeStep,Eigen ::VectorXd & a,Eigen ::VectorXd & l_m,Eigen ::VectorXd & dl_m,Eigen ::VectorXd const & u,muscle_data const & d,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    double duration=(double)lua_tonumber(L,1);                // 576
    double muscleTimeStep=(double)lua_tonumber(L,2);          // 576
    Eigen ::VectorXd & a=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,3)); // 568
    Eigen ::VectorXd & l_m=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,4)); // 568
    Eigen ::VectorXd & dl_m=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,5)); // 568
    Eigen ::VectorXd const & u=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,6)); // 568
    muscle_data const & d=static_cast<muscle_data &>(*Luna<muscle_data >::check(L,7)); // 568
    try {                                                     // 295
    integrateMuscleDynamics( duration, muscleTimeStep, a, l_m, dl_m, u, d); // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
  static int _bind_getMetabolicEnergyRate(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_getMetabolicEnergyRate(L)) { char msg[]="luna typecheck failed:\n  getMetabolicEnergyRate(double totalMass,Eigen ::VectorXd const & mass,Eigen ::VectorXd const & u,Eigen ::VectorXd const & a,Eigen ::VectorXd const & l_m,Eigen ::VectorXd const & l_m_opt,Eigen ::VectorXd const & dl_m,Eigen ::VectorXd const & f_mtu,Eigen ::VectorXd const & f_ce,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    double totalMass=(double)lua_tonumber(L,1);               // 576
    Eigen ::VectorXd const & mass=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,2)); // 568
    Eigen ::VectorXd const & u=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,3)); // 568
    Eigen ::VectorXd const & a=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,4)); // 568
    Eigen ::VectorXd const & l_m=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,5)); // 568
    Eigen ::VectorXd const & l_m_opt=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,6)); // 568
    Eigen ::VectorXd const & dl_m=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,7)); // 568
    Eigen ::VectorXd const & f_mtu=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,8)); // 568
    Eigen ::VectorXd const & f_ce=static_cast<Eigen ::VectorXd &>(*Luna<Eigen ::VectorXd >::check(L,9)); // 568
    try {                                                     // 340
    double ret=getMetabolicEnergyRate( totalMass, mass, u, a, l_m, l_m_opt, dl_m, f_mtu, f_ce); // 341
    lua_pushnumber(L, ret);                                   // 342
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 343
    return 1;                                                 // 344
  }                                                           // 374
  static int _bind_solveQuadprog(lua_State *L)
  {                                                           // 233
    if (_lg_typecheck_solveQuadprog_overload_1(L)) return _bind_solveQuadprog_overload_1(L); // 236
    if (_lg_typecheck_solveQuadprog_overload_2(L)) return _bind_solveQuadprog_overload_2(L); // 236
    luaL_error(L, "solveQuadprog ( cannot find overloads:)\n(HessianQuadratic & problem,const matrixn & CE,const vectorn & ce0,const matrixn & CI,const vectorn & ci0,vectorn & x,)\n(HessianQuadratic & problem,const matrixn & CE,const vectorn & ce0,const matrixn & CI,const vectorn & ci0,vectorn & x,bool _arg7,)\n");
                                                              // 243
    return 0;                                                 // 244
  }                                                           // 245
}; // end of class impl_luna__interface_15473501_Eigen        // 1612
const char luna__interface_15473501_Eigen::moduleName[] = "_Eigen"; // 1640
luna_RegType luna__interface_15473501_Eigen::methods[] = {    // 1649
    {"sigmoid", &impl_luna__interface_15473501_Eigen::_bind_sigmoid}, // 1654
    {"computeCosPennation", &impl_luna__interface_15473501_Eigen::_bind_computeCosPennation}, // 1654
    {"computeActivationDeriv", &impl_luna__interface_15473501_Eigen::_bind_computeActivationDeriv}, // 1654
    {"computeNormTendonForce", &impl_luna__interface_15473501_Eigen::_bind_computeNormTendonForce}, // 1654
    {"computeNormActiveFiberForce", &impl_luna__interface_15473501_Eigen::_bind_computeNormActiveFiberForce}, // 1654
    {"computePassiveFiberForce", &impl_luna__interface_15473501_Eigen::_bind_computePassiveFiberForce}, // 1654
    {"getFiberLengthDeriv", &impl_luna__interface_15473501_Eigen::_bind_getFiberLengthDeriv}, // 1654
    {"getIsometricFiberLength", &impl_luna__interface_15473501_Eigen::_bind_getIsometricFiberLength}, // 1654
    {"clamp_dl_m", &impl_luna__interface_15473501_Eigen::_bind_clamp_dl_m}, // 1654
    {"integrateMuscleDynamics", &impl_luna__interface_15473501_Eigen::_bind_integrateMuscleDynamics}, // 1654
    {"getMetabolicEnergyRate", &impl_luna__interface_15473501_Eigen::_bind_getMetabolicEnergyRate}, // 1654
    {"solveQuadprog", &impl_luna__interface_15473501_Eigen::_bind_solveQuadprog}, // 1654
    {0,0}                                                     // 1657
};                                                            // 1658
#if defined (USE_MPI)                                         // 1372
 class impl_luna__interface_15473501_MPI {
public:                                                       // 1392
    typedef LunaModule<luna__interface_15473501_MPI> luna_t;  // 1394
// : number denotes the line number of luna_gen.lua that generated the sentence // 1399
  inline static bool _lg_typecheck_rank(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=0) return false;                       // 621
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_size(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=0) return false;                       // 621
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_send(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=2) return false;                       // 621
    if( lua_isstring(L,1)==0) return false;                   // 636
    if( lua_isnumber(L,2)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_receive(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=1) return false;                       // 621
    if( lua_isnumber(L,1)==0) return false;                   // 631
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_source(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=0) return false;                       // 621
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_test(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=0) return false;                       // 621
    return true;
  }                                                           // 656
  inline static bool _lg_typecheck_test2(lua_State *L)
  {                                                           // 1418
    if( lua_gettop(L)!=0) return false;                       // 621
    return true;
  }                                                           // 656
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
                                                              // 1444
  static int _bind_rank(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_rank(L)) { char msg[]="luna typecheck failed:\n  rank()"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    try {                                                     // 340
    int ret=rank();                                           // 341
    lua_pushnumber(L, ret);                                   // 342
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 343
    return 1;                                                 // 344
  }                                                           // 374
  static int _bind_size(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_size(L)) { char msg[]="luna typecheck failed:\n  size()"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    try {                                                     // 340
    int ret=size();                                           // 341
    lua_pushnumber(L, ret);                                   // 342
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 343
    return 1;                                                 // 344
  }                                                           // 374
  static int _bind_send(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_send(L)) { char msg[]="luna typecheck failed:\n  send(const char * msg,int i,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    const char * msg=(const char *)lua_tostring(L,1);         // 571
    int i=(int)lua_tonumber(L,2);                             // 576
    try {                                                     // 295
    send( msg, i);                                            // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
  static int _bind_receive(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_receive(L)) { char msg[]="luna typecheck failed:\n  receive(int i,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    int i=(int)lua_tonumber(L,1);                             // 576
    try {                                                     // 346
    std ::string ret=receive( i);                             // 347
    lua_pushstring(L, ret.c_str());                           // 350
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 354
    return 1;                                                 // 355
  }                                                           // 374
  static int _bind_source(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_source(L)) { char msg[]="luna typecheck failed:\n  source()"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    try {                                                     // 340
    int ret=source();                                         // 341
    lua_pushnumber(L, ret);                                   // 342
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 343
    return 1;                                                 // 344
  }                                                           // 374
  static int _bind_test(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_test(L)) { char msg[]="luna typecheck failed:\n  test()"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    try {                                                     // 295
    test();                                                   // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
  static int _bind_test2(lua_State *L)
  {                                                           // 1451
    if (!_lg_typecheck_test2(L)) { char msg[]="luna typecheck failed:\n  test2()"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 548
    try {                                                     // 295
    test2();                                                  // 296
    } 
    catch(std::exception& e) { ASSERT(false); luaL_error( L,e.what()); }
    catch(...) { ASSERT(false); luaL_error( L,"unknown_error");}
                                                              // 297
    return 0;                                                 // 298
  }                                                           // 374
}; // end of class impl_luna__interface_15473501_MPI          // 1612
const char luna__interface_15473501_MPI::moduleName[] = "_MPI"; // 1640
luna_RegType luna__interface_15473501_MPI::methods[] = {      // 1649
    {"rank", &impl_luna__interface_15473501_MPI::_bind_rank}, // 1654
    {"size", &impl_luna__interface_15473501_MPI::_bind_size}, // 1654
    {"send", &impl_luna__interface_15473501_MPI::_bind_send}, // 1654
    {"receive", &impl_luna__interface_15473501_MPI::_bind_receive}, // 1654
    {"source", &impl_luna__interface_15473501_MPI::_bind_source}, // 1654
    {"test", &impl_luna__interface_15473501_MPI::_bind_test}, // 1654
    {"test2", &impl_luna__interface_15473501_MPI::_bind_test2}, // 1654
    {0,0}                                                     // 1657
};                                                            // 1658
#endif //defined (USE_MPI)                                    // 1661
void Register_QP(lua_State* L) {                              // 1665
    luna_dostring(L,"if __luna==nil then __luna={} end");     // 1666
    luna_dostring(L,"    if __luna.copyMethodsFrom==nil then\n        function __luna.copyMethodsFrom(methodsChild, methodsParent)\n            for k,v in pairs(methodsParent) do\n                if k~='__index' and k~='__newindex' and methodsChild[k]==nil then\n                    methodsChild[k]=v\n                end\n            end\n        end\n        function __luna.overwriteMethodsFrom(methodsChild, methodsParent)\n            for k,v in pairs(methodsParent) do\n                if k~='__index' and k~='__newindex' then\n                    if verbose then print('registering', k, methodsChild[k]) end\n                    methodsChild[k]=v\n                end\n            end\n        end\n    end\n    "); // 1667
    Luna<OBJloader ::ConvexDecomp >::Register(L);             // 1708
    luna_dostring(L, "ConvexDecomp=__luna._ConvexDecomp");    // 1731
    luna_dostring(L,"                __luna._ConvexDecomp.luna_class='ConvexDecomp'"); // 1732
    Luna<CMAwrap >::Register(L);                              // 1708
    luna_dostring(L, "if not math then math={} end math.CMAwrap=__luna.math_CMAwrap"); // 1715
    luna_dostring(L,"                __luna.math_CMAwrap.luna_class='h.CMAwrap'"); // 1716
    Luna<Eigen ::VectorXd >::Register(L);                     // 1708
    luna_dostring(L, "VectorXd=__luna._VectorXd");            // 1731
    luna_dostring(L,"                __luna._VectorXd.luna_class='VectorXd'"); // 1732
    Luna<path_points >::Register(L);                          // 1708
    luna_dostring(L, "path_points=__luna._path_points");      // 1731
    luna_dostring(L,"                __luna._path_points.luna_class='path_points'"); // 1732
    Luna<muscle_data >::Register(L);                          // 1708
    luna_dostring(L, "muscle_data=__luna._muscle_data");      // 1731
    luna_dostring(L,"                __luna._muscle_data.luna_class='muscle_data'"); // 1732
   LunaModule<luna__interface_15473501_Eigen >::Register(L);  // 1706
    luna_dostring(L," \n                if Eigen==nil then \n                    Eigen={}\n                end \n                __luna.overwriteMethodsFrom(Eigen, __luna._Eigen)\n                "); // 1721
#if defined (USE_MPI)                                         // 1696
   LunaModule<luna__interface_15473501_MPI >::Register(L);    // 1706
    luna_dostring(L," \n                if MPI==nil then \n                    MPI={}\n                end \n                __luna.overwriteMethodsFrom(MPI, __luna._MPI)\n                "); // 1721
{
  std::stringstream stringStreams;// defining enums           // 1746
  stringStreams <<"MPI.ANY_SOURCE="<< MPI_ANY_SOURCE;         // 1750
  luna_dostring(L, stringStreams.str().c_str());
}                                                             // 1753
{
  std::stringstream stringStreams;// defining enums           // 1746
  stringStreams <<"MPI.ANY_TAG="<< MPI_ANY_TAG;               // 1750
  luna_dostring(L, stringStreams.str().c_str());
}                                                             // 1753
#endif //defined (USE_MPI)                                    // 1758
}                                                             // 1765