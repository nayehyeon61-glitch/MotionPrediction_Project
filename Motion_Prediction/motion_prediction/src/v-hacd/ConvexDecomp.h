
#include <BaseLib/motion/Mesh.h>
#include <BaseLib/utility/VoxelGraph.h>


// by taesoo

namespace OBJloader
{
	class ConvexDecomp
	{
		std::vector<OBJloader::Mesh> _convexes;
		vector3N _centroid;
		void *_data;
		void _copyConvexes();
		public:

		ConvexDecomp(OBJloader::Mesh const& inputMesh, bool performConvexDecomposition, int voxelRes=400000, int maxConvex=64, double volumePercentError=1.0);
		virtual ~ConvexDecomp();

		void performConvexDecomposition();
		// use only subset of voxels having the value.
		void performConvexDecomposition(VoxelGraph::Image3D const& image, int value);

		vector3N const& centroids() const { return _centroid;}
		int numConvex() const { return _convexes.size();}
		OBJloader::Mesh const& convex(int h) const { return _convexes[h];}


		// retrieve raw voxel data
		vector3 getBoundsMin() const;
		vector3 getBoundsMax() const;
		vector3 getDimensions() const;
		int getVoxel(unsigned int x, unsigned int y, unsigned int z) const; // 2:outside, 3: interior_voxel, 4: surface_voxel, 
		inline int getVoxel(vector3 const& vi) const { return getVoxel(vi.x, vi.y, vi.z);}
		int getVoxelSafe(vector3 const& vi) const; // vi can be errorneous such as (-1,0,0)

		int numSurfaceVoxels() const;
		int numInteriorVoxels() const;
		vector3 getSurfaceVoxelIndex(int isurfvoxel) const; // for efficient iteration
		vector3 getInteriorVoxelIndex(int iinteriorvoxel) const;

		vector3 getVoxelIndex(vector3 const& worldPosition) const; // returns vector3(i,j,k) : integer indices.
		vector3 getWorldPosition(vector3 const& voxelIndex) const;
		vector3 getWorldPosition(VoxelGraph::shortvector3 const& voxelIndex) const;

		VoxelGraph* createGraphFromAllOccupantVoxels() const;
	};
}

