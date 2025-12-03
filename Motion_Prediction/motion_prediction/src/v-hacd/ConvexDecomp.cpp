#include "stdafx.h"

#define ENABLE_VHACD_IMPLEMENTATION 1
#include "app/VHACD.h"
#include "ConvexDecomp.h"
using namespace OBJloader;



ConvexDecomp::ConvexDecomp(OBJloader::Mesh const& inputMesh, bool performConvexDecomposition, int voxelRes, int maxConvex, double volumePercentError)
{

	std::vector<uint32_t> Triangles;
	int nTriangles=inputMesh.numFace();
	Triangles.resize(nTriangles*3);
	for (uint32_t i = 0; i < nTriangles; i++) {
		auto const& face=inputMesh.getFace(i);
		
		for(int j=0; j<3; j++)
			Triangles[i*3+j]=face.vertexIndex(j);
	}

	std::vector<double> Points;
	int nVertex=inputMesh.numVertex();
	Points.resize(nVertex*3);
	for (uint32_t i = 0; i < nVertex; i++) {
		auto& v=inputMesh.getVertex(i);
		Points[i*3]=v.x;
		Points[i*3+1]=v.y;
		Points[i*3+2]=v.z;
	}
	//
	//	Setup VHACD Parameters and create its interface
	VHACD::IVHACD::Parameters params;
	params.m_maxConvexHulls=maxConvex;
	params.m_minimumVolumePercentErrorAllowed=volumePercentError;
	params.m_resolution=voxelRes;

	VHACD::IVHACD* interfaceVHACD = VHACD::CreateVHACD();

	if(!performConvexDecomposition)
	{
		//	Compute approximate convex decomposition
		//printf("Compute V-HACD: Points %i Triangles %i\n", Points.size(), Triangles.size());
		bool res = interfaceVHACD->ComputeVolumeOnly(Points.data(), (uint32_t)(Points.size() / 3),
				Triangles.data(), (uint32_t)(Triangles.size() / 3), params);
		_convexes.clear();
		_centroid.resize(0);
	}
	else
	{
		//	Compute approximate convex decomposition
		//printf("Compute V-HACD: Points %i Triangles %i\n", Points.size(), Triangles.size());
		bool res = interfaceVHACD->Compute(Points.data(), (uint32_t)(Points.size() / 3),
				Triangles.data(), (uint32_t)(Triangles.size() / 3), params);
	}

	
	VHACD::VHACDImpl* impl=static_cast<VHACD::VHACDImpl* >(interfaceVHACD);
	_data=(void*)impl;
	if(performConvexDecomposition)
		_copyConvexes();
}

inline VHACD::VHACDImpl* getInterface(void* _data)
{
	return (VHACD::VHACDImpl*)_data;
}

inline VHACD::VoxelizeImpl* getVoxelize(void* _data)
{
	return static_cast<VHACD::VoxelizeImpl*>(getInterface(_data)->mVoxelize);
}
inline VHACD::Volume* getVolume(void* _data)
{
	return getVoxelize(_data)->mVolume;
}

ConvexDecomp::~ConvexDecomp()
{
	delete getInterface(_data);
}

static inline void toWorld(void* _data, vector3& out)
{
	auto* impl=getInterface(_data);
	auto mScale=impl->mScale;
	auto* mCenter=impl->mCenter;
	out.x = (out.x*mScale)+mCenter[0];
	out.y = (out.y*mScale)+mCenter[1];
	out.z = (out.z*mScale)+mCenter[2];
}

vector3 ConvexDecomp::getBoundsMin() const
{
	vector3 out;
	getVoxelize(_data)->getBoundsMin(&out.x);
	toWorld(_data, out);
	return out;
}
vector3 ConvexDecomp::getBoundsMax() const
{
	vector3 out;
	getVoxelize(_data)->getBoundsMax(&out.x);
	toWorld(_data, out);
	return out;
}
vector3 ConvexDecomp::getDimensions() const
{
	vector3 out;
	uint32_t dim[3];
	getVoxelize(_data)->getDimensions(dim);
	out.x=dim[0];
	out.y=dim[1];
	out.z=dim[2];
	return out;
}
int ConvexDecomp::getVoxel(unsigned int x, unsigned int y, unsigned int z) const
{
    return getVoxelize(_data)->getVoxel(x,y,z);
}
int ConvexDecomp::getVoxelSafe(vector3 const& v) const
{
	auto* voxelize=getVoxelize(_data);
	auto* dim=voxelize->mVolume->m_dim;
	unsigned int x=(unsigned int)v.x;
	unsigned int y=(unsigned int)v.y;
	unsigned int z=(unsigned int)v.z;

	if(x>=0 && y>=0 && z>=0 &&
			x<dim[0] && y<dim[1]&& z<dim[2])
		return voxelize->getVoxel(x,y,z);
	return 0;
}

int ConvexDecomp::numSurfaceVoxels() const
{
	return getVolume(_data)->GetNPrimitivesOnSurf();
}
int ConvexDecomp::numInteriorVoxels() const
{
	return getVolume(_data)->GetNPrimitivesInsideSurf();
}
vector3 ConvexDecomp::getSurfaceVoxelIndex(int isurfvoxel) const
{
	uint32_t x,y,z;
	getVolume(_data)->getSurfaceVoxels()[isurfvoxel].getVoxel(x,y,z);
	return vector3(double(x), double(y), double(z));
}
vector3 ConvexDecomp::getInteriorVoxelIndex(int ivoxel) const
{
	uint32_t x,y,z;
	getVolume(_data)->getInteriorVoxels()[ivoxel].getVoxel(x,y,z);
	return vector3(double(x), double(y), double(z));
}
vector3 ConvexDecomp::getVoxelIndex(vector3 const& worldPosition) const
{
	auto* impl=getInterface(_data);
	vector3 localPos=(worldPosition-*(reinterpret_cast<vector3*>(impl->mCenter)))*(1.0/impl->mScale);

	const double* pos=&localPos.x;
	uint32_t x,y,z;

#if 0
	// with error checking
	// basically x == uint32_t( (pos[0] - bmin[0])*recipScale);
	getVoxelize(_data)->getVoxel(pos, x,y,z);
#else
	auto* v=getVoxelize(_data);
	double recipScale=1.0/v->getScale();
	double bmin[3];
	v->getBoundsMin(bmin);
	x = uint32_t( (pos[0] - bmin[0])*recipScale+0.01);
	y = uint32_t( (pos[1] - bmin[1])*recipScale+0.01);
	z = uint32_t( (pos[2] - bmin[2])*recipScale+0.01);
#endif
	
	return vector3(double(x), double(y), double(z));
}
vector3 ConvexDecomp::getWorldPosition(vector3 const& voxelIndex) const
{
	auto* v=getVoxelize(_data);
	double s=v->getScale();
	vector3 bmin_internal;
	v->getBoundsMin(&bmin_internal.x);
	vector3 out=(voxelIndex)*s+bmin_internal;
	toWorld(_data, out);
	return out;
}
vector3 ConvexDecomp::getWorldPosition(VoxelGraph::shortvector3 const& t) const
{
	vector3 voxelIndex(t.x(), t.y(), t.z());
	return getWorldPosition(voxelIndex);
}

#include "3Dthinning/itkBinaryThinningImageFilter3D.h"


VoxelGraph* ConvexDecomp::createGraphFromAllOccupantVoxels() const
{
	vector3 ndim=getDimensions();
	short nx=ndim.x;
	short ny=ndim.y;
	short nz=ndim.z;

	auto* filter=new VoxelGraph::Image3D(nx, ny, nz,0);

	int INTERIOR_VOXEL=3;
	int SURFACE_VOXEL=4;
	for(int i=0; i<nx; i++)
		for(int j=0; j<ny; j++)
			for(int k=0; k<nz; k++)
			{
				int t=getVoxel(i, j, k);

				if(t==SURFACE_VOXEL || t==INTERIOR_VOXEL )
					filter->setPixel(i,j,k,1);
			}

	return new VoxelGraph(nx, ny, nz, filter);
}
void ConvexDecomp::_copyConvexes()
{
	auto* interfaceVHACD=getInterface(_data);

	VHACD::IVHACD::ConvexHull Hull;
	//	Get the number of convex hulls
	unsigned int nConvexHulls = interfaceVHACD->GetNConvexHulls();
	//printf("V-HACD Done: Hull Count %i\n", nConvexHulls);
	
	_convexes.resize(nConvexHulls);
	_centroid.resize(nConvexHulls);
	//	Iterate through each convex hull and fill results
	for (unsigned int h = 0; h < nConvexHulls; ++h)
	{
		//printf("\tHull: %i\n", h);
		//printf("\t\tPoints: %i\n", Hull.m_points);
		//printf("\t\tTriangles: %i\n", Hull.m_triangles);
		//printf("\t\tVertices: %i\n", vertices.size());
		//
		//	Fill 'Hull' for each individual convex hull
		interfaceVHACD->GetConvexHull(h, Hull);
		//
		//	Grab the hulls center position
		vector3 centroid(Hull.m_center[0], Hull.m_center[1], Hull.m_center[2]);
		_centroid[h]=centroid;

		auto& mesh=_convexes[h];
		mesh.resize(Hull.m_nPoints, Hull.m_nTriangles);  

		for (unsigned int index0 = 0; index0 < Hull.m_nPoints; index0++) {
			const vector3 vertex0(Hull.m_points[index0 * 3], Hull.m_points[index0 * 3 + 1], Hull.m_points[index0 * 3 + 2]);
			mesh.getVertex(index0)=vertex0;
		}
		//
		//	Iterate through this hulls triangles
		for (unsigned int i = 0; i < Hull.m_nTriangles; i++) {
			//
			//	Calculate indices
			const unsigned int index0 = Hull.m_triangles[i * 3];
			const unsigned int index1 = Hull.m_triangles[i * 3 + 1];
			const unsigned int index2 = Hull.m_triangles[i * 3 + 2];

			mesh.getFace(i).setIndex(index0, index1, index2);
		}
	}
}
void ConvexDecomp::performConvexDecomposition()
{
	getInterface(_data)->ComputeConvexHulls();
	_copyConvexes();
}
void ConvexDecomp::performConvexDecomposition(VoxelGraph::Image3D const& image, int value)
{
	getInterface(_data)->ComputeConvexHulls(image, value);
	_copyConvexes();
}
