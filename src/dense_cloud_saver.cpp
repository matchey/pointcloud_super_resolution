
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include "pointcloud_super_resolution/dense_cloud_saver.h"

namespace pointcloud_super_resolution
{
	template<typename PointT>
	DenseCloudSaver<PointT>::DenseCloudSaver()
	  : radius(0.1f), scan(new PointCloud), map(new PointCloud), npoints(0),
	    pc_dense(new PointCloud)
	{
	}

	template<typename PointT>
	void DenseCloudSaver<PointT>::setSource(const PointCloudPtr& input)
	{
		scan = input;
		if(npoints){
			computeIndices();
		}
	}

	template<typename PointT>
	void DenseCloudSaver<PointT>::setSource(const PointCloudPtr& input, const Eigen::Affine3d& m)
	{
		transform = m;
		pcl::transformPointCloud(*input, *scan, transform);
		if(npoints){
			computeIndices();
		}
		pcl::transformPointCloud(*pc_dense, *pc_dense, transform.inverse());
	}

	template<typename PointT>
	void DenseCloudSaver<PointT>::setTarget(const PointCloudPtr& input)
	{
		map = input;
		kdtree.setInputCloud(map);
		npoints = map->points.size();
	}

	template<typename PointT>
	void DenseCloudSaver<PointT>::setRadius(const float& search_dist)
	{
		radius = search_dist;
	}

	template<typename PointT>
	void DenseCloudSaver<PointT>::getDenseCloud(PointCloudPtr& pc)
	{
		pc = pc_dense;
	}

	template<typename PointT>
	void DenseCloudSaver<PointT>::saveDenseCloud(const std::string& filename)
	{
		if(pc_dense->points.size()){
			pcl::io::savePCDFileBinary(filename, *pc_dense);
		}else{
			std::cerr << filename << " : dence point cloud size 0" << std::endl;
		}
	}

	template<typename PointT>
	void DenseCloudSaver<PointT>::saveDenseCloud(const std::string& filename,
												 const Eigen::Affine3d& trans)
	{
		if(pc_dense->points.size()){
			pcl::transformPointCloud(*pc_dense, *pc_dense, trans);
			pcl::io::savePCDFileBinary(filename, *pc_dense);
		}else{
			std::cerr << filename << " : dence point cloud size 0" << std::endl;
		}
	}

	// private
	template<typename PointT>
	void DenseCloudSaver<PointT>::computeIndices()
	{
		std::vector<int> indices;
		std::vector<float> distances;

		std::set<int> id_dense;

		int nscan = scan->points.size();

		for(size_t i = 0; i != nscan; ++i){
			if(kdtree.radiusSearch(scan->points[i], radius, indices, distances) > 0){
				for(size_t i = 0; i != indices.size (); ++i){
					id_dense.insert(indices[i]);
				}
			}
		}

		pc_dense->points.clear();
		for(auto id : id_dense){
			pc_dense->points.push_back(map->points[id]);
		}
	}
} // namespace pointcloud_super_resolution


