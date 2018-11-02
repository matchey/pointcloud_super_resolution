
#include <ros/ros.h>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include "mstring/mstring.h"
#include "pointcloud_super_resolution/dense_cloud_saver.h"

using std::cout;
using std::endl;
using std::string;
// using m::string;

namespace pointcloud_super_resolution
{
	class WriteDense
	{
		public:
		using PointT = pcl::PointXYZINormal;
		using PointCloud = pcl::PointCloud<PointT>;
		using PointCloudPtr = PointCloud::Ptr;

		WriteDense();
		void process();

		private:
		void loadPCD();
		// void transform();

		DenseCloudSaver<PointT> dcs;
	};

	WriteDense::WriteDense()
	{
	}

	void WriteDense::process()
	{
		std::ifstream ifs("aft.csv");
		string str;
		std::vector<string> v;

		if(ifs.fail()){
			std::cerr << "cant read aft.csv" << endl;
			return;
		}

		PointCloudPtr map(new PointCloud);
		if(pcl::io::loadPCDFile<PointT>("map/map_ds.pcd", *map) < 0){
			std::cerr << "cant read map_ds.pcd" << endl;
			return;
		}

		dcs.setTarget(map);
		dcs.setRadius(0.15);

		// int id;
		Eigen::Affine3d transform;

		PointCloudPtr pc(new PointCloud);
		PointCloudPtr transformed(new PointCloud);

		while(getline(ifs, str)){
			if(!str.empty()){
				v = split(str, ' ');
				//  v[0] v[1] v[2] v[3] v[4] v[5] v[6] v[7] v[8]
				// vertex id   x    y    z    qx   qy   qz   qw
				if(v[0] == "VERTEX_SE3:QUAT"){
					// id = std::stoi(v[1]);
					string filename = "clouds/cloud_" + v[1] + ".pcd";
					if(pcl::io::loadPCDFile<PointT>(filename, *pc) < 0){
						std::cerr << "cant read cloud_" << v[1] << ".pcd" << endl;
					}else{
						Eigen::Translation<double, 3> t(stod(v[2]), stod(v[3]), stod(v[4]));
						Eigen::Quaterniond q(stod(v[8]), stod(v[5]), stod(v[6]), stod(v[7]));
						transform = t * q;
						dcs.setSource(pc, transform);
						dcs.saveDenseCloud("dense/dense_" + v[1] + ".pcd");
						// pcl::transformPointCloud(*pc, *transformed, transform);
						// dcs.setSource(transformed);
						// t = Eigen::Translation<double, 3>(-stod(v[2]), -stod(v[3]), -stod(v[4]));
						// transform = t * q.inverse();
						// dcs.saveDenseCloud("dense/dense_" + v[1] + ".pcd", transform);
					}
				}
			}
		}
	}

	// private
	void WriteDense::loadPCD()
	{
	}

	// void WriteDense::transform()
	// {
	// }
}
			// string pcd_filename = "cloud_0.pcd";
			// ostringstream ostr;
			// ostr.str("");
			// ostr << "cloud_" << counter << ".pcd";
			// pcd_filename = ostr.str();
            //
			// pcl::io::savePCDFileBinary(pcd_filename, *cloud);
            //
			// cout<<"saved cloud_"<<counter<<".pcd"<<endl;
            //
			// counter++;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_save_dense_clouds");

	cout << "test_save_dense_clouds" << endl;

	pointcloud_super_resolution::WriteDense wd;

	wd.process();

	return 0;
}
