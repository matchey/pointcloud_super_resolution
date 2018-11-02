
#ifndef __DENSE_CLOUD_SAVER_H
#define __DENSE_CLOUD_SAVER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace pointcloud_super_resolution
{
	template<typename PointT>
	class DenseCloudSaver
	{
		public:
		using PointCloud = pcl::PointCloud<PointT>;
		using PointCloudPtr = typename PointCloud::Ptr;

		DenseCloudSaver();
		void setSource(const PointCloudPtr&);
		void setSource(const PointCloudPtr&, const Eigen::Affine3d&);
		void setTarget(const PointCloudPtr&);
		void setRadius(const float&);
		void getDenseCloud(PointCloudPtr&);
		void saveDenseCloud(const std::string&);
		void saveDenseCloud(const std::string&, const Eigen::Affine3d&);

		private:
		void computeIndices();

		float radius; // pclのDocumentに合わせとこうfloat
		pcl::KdTreeFLANN<PointT> kdtree;

		PointCloudPtr scan;
		PointCloudPtr map;
		int npoints;
		PointCloudPtr pc_dense;
		Eigen::Affine3d transform;
	};

	template class DenseCloudSaver<pcl::PointXYZ>;
	template class DenseCloudSaver<pcl::PointXYZI>;
	// template class DenseCloudSaver<pcl::PointXYZRGBA>;
	template class DenseCloudSaver<pcl::PointXYZRGB>;
	// template class DenseCloudSaver<pcl::PointXY>;
	// template class DenseCloudSaver<pcl::Normal>;
	template class DenseCloudSaver<pcl::PointNormal>;
	// template class DenseCloudSaver<pcl::PointXYZRGBNormal>;
	template class DenseCloudSaver<pcl::PointXYZINormal>;
} // namespace pointcloud_super_resolution

#endif

