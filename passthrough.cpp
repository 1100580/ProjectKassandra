#include <iostream>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/pcl_base.h>
#include <boost/make_shared.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/keyboard_event.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

//numero de metros até onde os pontos apanhados pela kinect vão ser considerados
#define pthrough 1.0

//bool para ver se a tecla para apanhar a point cloud foi pressionada
bool keyDown = false;

//Recipiente da point cloud apanhada pela kinect, sem transforações
pcl::PointCloud<pcl::PointXYZ>::Ptr first_cloud (new pcl::PointCloud<pcl::PointXYZ>);

class SimpleOpenNIViewer
 {
   public:
	//Abre uma janela OpenGL? onde são mostrados os dados que a kinect está a processar
     pcl::visualization::CloudViewer viewer;

     //função que trata o que acontece quando se usa o teclado na janela do visualizador.
     void keyboard_callback (const pcl::visualization::KeyboardEvent& event, void* cookie)
     {
       //pressionar S para apanhar a cloud
       if (event.getKeySym () == "s" && event.keyDown ())
       {
         std::cout << "Grabbed cloud" << std::endl;
         keyDown = true;
       }else{
       }
     }

     //construtor do visualizador de point clouds
     SimpleOpenNIViewer () : viewer ("Project Kassandra"){
         	 viewer.registerKeyboardCallback(&SimpleOpenNIViewer::keyboard_callback, *this, 0);
     }

     //função de callback que trata o que acontece á point cloud que está a ser mostrada pelo visualizador
     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
     {
       if (!viewer.wasStopped()){
         viewer.showCloud (cloud);
         if(keyDown){
        	 *first_cloud = *cloud;
        	 keyDown = false;
         }
       }
     }

     void run ()
     {
       pcl::Grabber* interface = new pcl::OpenNIGrabber();

       boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
              boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

       interface->registerCallback (f);

       interface->start ();

       while (!viewer.wasStopped())
       {
         boost::this_thread::sleep (boost::posix_time::seconds (1));
       }
       interface->stop ();
     }

 };


void passthrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered){
	  // Corta a point cloud apanhada num plano de profundidade "ptrhough"
	  pcl::PassThrough<pcl::PointXYZ> pass;
	  pass.setInputCloud (cloud);
	  pass.setFilterFieldName ("z");
	  pass.setFilterLimits (0.0, pthrough);

	  //aplica o filtro à cloud
	  pass.filter (*cloud_filtered);
}

//reduz o numero de pontos da point cloud encontrando pontos que representem os pontos à sua volta
void voxel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel){

	pcl::PCLPointCloud2 cloud2;

	pcl::toPCLPointCloud2(*cloud, cloud2);
	pcl::PCLPointCloud2ConstPtr cloud2Ptr (new pcl::PCLPointCloud2(cloud2));

	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud2Ptr);
	sor.setLeafSize (0.005f, 0.005f, 0.005f);

	pcl::PCLPointCloud2Ptr cloud_2 (new pcl::PCLPointCloud2 ());
	sor.filter (*cloud_2);
	pcl::fromPCLPointCloud2(*cloud_2, *cloud_voxel);
}

//Remove os planos da point cloud e guarda todos os clusters resultantes
//em ficheiros diferentes com o nome: cloud_cluster_N.pcd
void planar_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	  *cloud = *cloud_voxel;

	    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	  // Create the segmentation object
	    pcl::SACSegmentation<pcl::PointXYZ> seg;
	  // Optional
	  	seg.setOptimizeCoefficients (true);
	  // Mandatory
	    seg.setModelType (pcl::SACMODEL_PLANE);
	    seg.setMethodType (pcl::SAC_RANSAC);
	    seg.setMaxIterations (1000);
	    seg.setDistanceThreshold (0.01);

	  // Create the filtering object
	    pcl::ExtractIndices<pcl::PointXYZ> extract;
	  //the filtered final cloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
      //represents the planes
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());

	  int i=0, nr_points = (int) cloud->width*cloud->height;
	    while (cloud->width*cloud->height > 0.3 * nr_points)
	    {
	      // Segment the largest planar component from the remaining cloud
	      seg.setInputCloud (cloud);
	      seg.segment (*inliers, *coefficients);
	      if (inliers->indices.size () == 0)
	      {
	        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
	        break;
	      }

	      // Extract the planar inliers from the input cloud
//	      pcl::ExtractIndices<pcl::PointXYZ> extract;
	      extract.setInputCloud (cloud);
	      extract.setIndices (inliers);
	      extract.setNegative (false);

	      // Get the points associated with the planar surface
	      extract.filter (*cloud_plane);
	      std::cout << "PointCloud representing the planar component: " << cloud_plane->width*cloud_plane->height << " data points." << std::endl;

	      // Remove the planar inliers, extract the rest
	      extract.setNegative (true);
	      extract.filter (*cloud_f);
	      *cloud = *cloud_f;
	      i++;
	    }

	    // Creating the KdTree object for the search method of the extraction
	      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	      tree->setInputCloud (cloud);

	      std::vector<pcl::PointIndices> cluster_indices;
	      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	      ec.setClusterTolerance (0.02); // 2cm
	      ec.setMinClusterSize (100);
	      ec.setMaxClusterSize (25000);
	      ec.setSearchMethod (tree);
	      ec.setInputCloud (cloud);
	      ec.extract (cluster_indices);

	      /****Search for the biggest cluster? What if it picks it up as two clusters? This should have user input****/
	      /****And why is a program being compiled and running smoothly with sintax errors?
	       * I bet this wont do when run as a standalone executable, it's probably a problem when declaring objects
	       * I need to figure out the differences when instanciating objects in different ways****/

	      int j = 0;
	      //Going with biggest until I get help with keyboard/mouse events
	      pcl::PointCloud<pcl::PointXYZ>::Ptr biggest_cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
	      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	      {
	        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
	        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
	            cloud_cluster->push_back(cloud->points[*pit]); //*

	        cloud_cluster->width = cloud_cluster->width*cloud_cluster->height;
	        cloud_cluster->height = 1;
	        cloud_cluster->is_dense = true;

	        /*****Saves the biggest cluster  only*****/

	        if(j == 0 || cloud_cluster->size() > biggest_cloud_cluster->size()){
	        	*biggest_cloud_cluster = *cloud_cluster;
	        }

	        pcl::io::savePCDFileASCII("biggestcluster.pcd", *biggest_cloud_cluster);
	        *cloud = *biggest_cloud_cluster;

	        /****Saves all clusters****/

	        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
	        std::stringstream ss;
	        ss << "cloud_cluster_" << j << ".pcd";
	        std::string name = ss.str();
	        pcl::io::savePCDFileASCII(name, *cloud_cluster);

	        j++;
	      }
}

//metodo a estudar quando se for implementar movimento livre do objeto em vez do que existe
/*void addClouds(PointCloud::Ptr src, PointCloud::Ptr tgt, PointCloud::Ptr output){

	  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
	  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

	  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
	  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	  norm_est.setSearchMethod (tree);
	  norm_est.setKSearch (30);

	  norm_est.setInputCloud (src);
	  norm_est.compute (*points_with_normals_src);
	  pcl::copyPointCloud (*src, *points_with_normals_src);

	  norm_est.setInputCloud (tgt);
	  norm_est.compute (*points_with_normals_tgt);
	  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

	  //
	  // Instantiate our custom point representation (defined above) ...
	  MyPointRepresentation point_representation;
	  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
	  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
	  point_representation.setRescaleValues (alpha);


	  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
	  reg.setTransformationEpsilon (1e-6);
	  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
	  // Note: adjust this based on the size of your datasets
	  reg.setMaxCorrespondenceDistance (0.1);
	  // Set the point representation
	  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

	  reg.setInputCloud (points_with_normals_src);
	  reg.setInputTarget (points_with_normals_tgt);

	  // Run the same optimization in a loop and visualize the results
	  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
	  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
	  reg.setMaximumIterations (2);
	  for (int i = 0; i < 30; ++i)
	  {
	    PCL_INFO ("Iteration Nr. %d.\n", i);

	    // save cloud for visualization purpose
	    points_with_normals_src = reg_result;

	    // Estimate
	    reg.setInputCloud (points_with_normals_src);
	    reg.align (*reg_result);

	    //accumulate transformation between each Iteration
	    Ti = reg.getFinalTransformation () * Ti;

	  	//if the difference between this transformation and the previous one
	  	//is smaller than the threshold, refine the process by reducing
	  	//the maximal correspondence distance
	    if (fabs((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
	      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

	      prev = reg.getLastIncrementalTransformation ();
	    }

	    targetToSource = Ti.inverse();

	    // Transform target back in source frame
	    pcl::transformPointCloud (*tgt, *output, targetToSource);

	    //trocar void por Eigen::Matrix4f & quando tiver resolvido o type not found
	    //return targetToSource;


}*/

//método que adiciona uma point cloud a um modelo global
void addToGlobalModel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr model, Eigen::Matrix4f transformationMatrix){

	//rotação apenas dos cloud. ao modelo apenas sao adicionados os pontos
	//average dos pontos para centroid
	//translate para 000
	//rotate
	//translate oposto
	//pcl::transformPointCloud(*cloud,*cloud,transformationMatrix);
	//*model += *cloud;

}

//roda uma point cloud
void rotateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Matrix4f transformationMatrix){

	pcl::PointCloud<pcl::PointXYZ>::Ptr rotatedCloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*cloud, *rotatedCloud, transformationMatrix);

	//para já, apenas a guarda até conseguir rodá-la como quero
	pcl::io::savePCDFileASCII("rotatedOpen.pcd", *rotatedCloud);
	pcl::io::savePCDFileASCII("nonRotatedOpen.pcd", *cloud);
}

int main (int argc, char** argv)
{
	//clouds a ser usadas
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_planes (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr model (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr c2 (new pcl::PointCloud<pcl::PointXYZ>);

  Eigen::Matrix4f transformationMatrix;

  transformationMatrix <<
		  sin(90),  cos(90), 0, 0,
		  cos(90),  -sin(90),0, 0,
		  0,        0,       1, 0,
  		  0, 	    0, 		 0, 1;

  SimpleOpenNIViewer v;
    	v.run ();

      passthrough(first_cloud, cloud_passthrough);

      voxel(cloud_passthrough, cloud_voxel);

      planar_removal(cloud_voxel, cloud_no_planes);

      rotateCloud(cloud_no_planes, transformationMatrix);


  //Save Result, é importante que as point cloud tenham dados ou estas linhas estouram
  pcl::io::savePCDFileASCII("normal.pcd", *first_cloud);
  pcl::io::savePCDFileASCII("passthroughfilter.pcd", *cloud_passthrough);
  pcl::io::savePCDFileASCII("voxelfilter.pcd", *cloud_voxel);
  pcl::io::savePCDFileASCII("clustersonly.pcd", *cloud_no_planes);
  pcl::io::savePCDFileASCII("final.pcd", *cloud_final);

  return (0);
}
