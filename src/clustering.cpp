#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/impl/point_types.hpp>
#include <sensor_msgs/ChannelFloat32.h>
#include <pcl/common/projection_matrix.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pointcloud_msgs/PointCloud2_Segments.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <limits>       // std::numeric_limits
#include <stdlib.h>     /* div, div_t */
#include <algorithm>    // std::max()



    #include <string>

// #include <pcl/point_types.h>
// #include <pcl/visualization/point_cloud_geometry_handlers.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/search/organized.h>
// #include <pcl/search/kdtree.h>
// #include <pcl/features/normal_3d_omp.h>
// #include <pcl/filters/conditional_removal.h>
// #include <pcl/segmentation/extract_clusters.h>

// #include <pcl/features/don.h>

using namespace pcl;
using namespace std;





ros::Publisher pub, stationary_pub;

double clusterTolerance, distanceThreshold, percentOfpoints;

int maxIterations;
int minClusterSize, maxClusterSize;

bool moving_clusters_only, linear_check, vertical_check;
double vertical_dist, linear_dist;



std::pair<double,double> minmaxz (sensor_msgs::PointCloud2 clust){  //return max and min z of cluster


    pcl::PCLPointCloud2 p2;
    pcl_conversions::toPCL ( clust , p2 ); //from sensor_msgs::pointcloud2 to pcl::pointcloud2

    pcl::PointCloud<pcl::PointXYZ> p3;
    pcl::fromPCLPointCloud2 ( p2 , p3 );       //from pcl::pointcloud2 to pcl::pointcloud

    double max_z = p3.points[0].z;
    double min_z = p3.points[0].z;
    for (int i=1; i < p3.points.size(); i++){   //find max and min z of cluster 
        if(p3.points[i].z > max_z){
            max_z = p3.points[i].z;
        }
        if(p3.points[i].z < min_z){
            min_z = p3.points[i].z;
        }
    }

    return std::make_pair( max_z , min_z );
}

pcl::PointCloud<pcl::PointXYZ> saveAllPoints(sensor_msgs::PointCloud2 clust){  //save all points of a cluster to pointcloud form


    pcl::PCLPointCloud2 pc2;
    pcl_conversions::toPCL ( clust , pc2 );   //from sensor_msgs::pointcloud2 to pcl::pointcloud2

    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::fromPCLPointCloud2 ( pc2 , pc );               //from pcl::pointcloud2 to pcl::pointcloud

    return pc;
}

pcl::PointCloud<pcl::PointXYZ> saveAllZValuePoints(sensor_msgs::PointCloud2 clust, double zvalue){  //save all points whose z is equal to zvalue

    pcl::PointCloud<pcl::PointXYZ> pcz3;
    pcl::PointCloud<pcl::PointXYZ> pc;
    pc=saveAllPoints(clust);

    for(int i=0; i < pc.points.size(); i++){        //add points with zvalue to a new pointcloud
        if(pc.points[i].z == zvalue){
            pcz3.push_back(pc.points[i]);
        }
    }
    return pcz3;
}

pcl::PointCloud<pcl::PointXYZ> saveAllZPointsFrom(sensor_msgs::PointCloud2 clust, double zFrom){   // save all points whose z is grater than or equal to zFrom

    pcl::PointCloud<pcl::PointXYZ> pcz3;
    pcl::PointCloud<pcl::PointXYZ> pc;
    pc=saveAllPoints(clust); 

    for(int i=0; i < pc.points.size(); i++){        //add points with max z to a new pointcloud
        if(pc.points[i].z >= zFrom  ){
            pcz3.push_back(pc.points[i]);
        }
    }

    return pcz3;
}

pcl::PointCloud<pcl::PointXYZ> saveAllZPointsUntil(sensor_msgs::PointCloud2 clust, double zUntil){   // save all points whose z is less than or equal to zUntil

    pcl::PointCloud<pcl::PointXYZ> pcz3;
    pcl::PointCloud<pcl::PointXYZ> pc;
    pc=saveAllPoints(clust);

    for(int i=0; i < pc.points.size(); i++){        //add points with max z to a new pointcloud
        if(pc.points[i].z <= zUntil  ){
            pcz3.push_back(pc.points[i]);
        }
    }

    return pcz3;
}


bool checkforsameXYpoints(pcl::PointCloud<pcl::PointXYZ> pcz_max, pcl::PointCloud<pcl::PointXYZ> pcz_min){ // check if a maxz point is enclosed to minz points

   
    double xmin,xmax, ymin, ymax;

    xmin= pcz_min.points[0].x;
    xmax= pcz_min.points[0].x;
    ymin= pcz_min.points[0].y;
    ymax= pcz_min.points[0].y;

    for(int i=1; i < pcz_min.points.size(); i++){        
        if(pcz_min.points[i].x < xmin){
            xmin= pcz_min.points[i].x;
        }
        if(pcz_min.points[i].x > xmax){
            xmax= pcz_min.points[i].x;
        }
        if(pcz_min.points[i].y < ymin){
            ymin= pcz_min.points[i].y;
        }
        if(pcz_min.points[i].y > ymax){
            ymax= pcz_min.points[i].y;
        }
    }

    bool same=false;

    for(int k=0; k < pcz_max.points.size(); k++){        
        if(pcz_max.points[k].x >= xmin && pcz_max.points[k].x <= xmax && pcz_max.points[k].y >= ymin && pcz_max.points[k].y <= ymax){
            same=true;
            break;
        }
    }
    return same;
}






void cloud_callback (const pointcloud_msgs::PointCloud2_Segments& c_)
{

    pcl::PCLPointCloud2 cloud2;

    pcl_conversions::toPCL( c_.clusters[0] , cloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::fromPCLPointCloud2(cloud2, *cloud);

    //Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ> ());
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    int i=0, nr_points = (int) cloud_filtered->points.size ();
    while (cloud_filtered->points.size () > percentOfpoints * nr_points)
    {

        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
          break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
      

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f;
    }

    *cloud=*cloud_filtered;




  // // Create a search tree, use KDTreee for non-organized data.

  // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  // // Set the input pointcloud for the search tree
  // tree->setInputCloud (cloud);


  // // Compute normals using both small and large scales at each point
  // pcl::NormalEstimationOMP<PointXYZ, PointNormal> ne;
  // ne.setInputCloud (cloud);
  // ne.setSearchMethod (tree);

  // /**
  //  * NOTE: setting viewpoint is very important, so that we can ensure
  //  * normals are all pointed in the same direction!
  //  */

  // // ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());
  // ne.setViewPoint (0.0,0.0,0.0);

  // // calculate normals with the small scale
  // //std::cout << "Calculating normals for scale..." << scale1 << std::endl;
  // pcl::PointCloud<PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<PointNormal>);

  // ne.setRadiusSearch (0.1);
  // ne.compute (*normals_small_scale);

  // // calculate normals with the large scale
  // // std::cout << "Calculating normals for scale..." << scale2 << std::endl;
  //   pcl::PointCloud<PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<PointNormal>);

  // ne.setRadiusSearch (0.4);
  // ne.compute (*normals_large_scale);

  // // Create output cloud for DoN results
  // PointCloud<PointNormal>::Ptr doncloud (new pcl::PointCloud<PointNormal>);
  // copyPointCloud (*cloud, *doncloud);

  // std::cout << "Calculating DoN... " << std::endl;
  // // Create DoN operator
  // pcl::DifferenceOfNormalsEstimation<PointXYZ, PointNormal, PointNormal> don;
  // don.setInputCloud (cloud);
  // don.setNormalScaleLarge (normals_large_scale);
  // don.setNormalScaleSmall (normals_small_scale);

  // if (!don.initCompute ())
  // {
  //   std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
  //   exit (EXIT_FAILURE);
  // }

  // // Compute DoN
  // don.computeFeature (*doncloud);


  // // Filter by magnitude
  // // std::cout << "Filtering out DoN mag <= " << threshold << "..." << std::endl;

  // // Build the condition for filtering
  // pcl::ConditionOr<PointNormal>::Ptr range_cond (
  //   new pcl::ConditionOr<PointNormal> ()
  //   );
  // range_cond->addComparison (pcl::FieldComparison<PointNormal>::ConstPtr (
  //                              new pcl::FieldComparison<PointNormal> ("curvature", pcl::ComparisonOps::GT, 0.25))
  //                            );
  // // Build the filter
  // pcl::ConditionalRemoval<PointNormal> condrem;
  // condrem.setCondition (range_cond);
  // condrem.setInputCloud (doncloud);

  // pcl::PointCloud<PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<PointNormal>);

  // // Apply filter
  // condrem.filter (*doncloud_filtered);

  // doncloud = doncloud_filtered;

  // // Save filtered output
  // std::cout << "Filtered Pointcloud: " << doncloud->points.size () << " data points." << std::endl;
 
  // // Filter by magnitude
  // //std::cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << std::endl;

  // pcl::search::KdTree<PointNormal>::Ptr segtree (new pcl::search::KdTree<PointNormal>);
  // segtree->setInputCloud (doncloud);

  // std::vector<pcl::PointIndices> cluster_indices;
  // pcl::EuclideanClusterExtraction<PointNormal> ec;

  // ec.setClusterTolerance (clusterTolerance);
  // ec.setMinClusterSize (50);
  // ec.setMaxClusterSize (100000);
  // ec.setSearchMethod (segtree);
  // ec.setInputCloud (doncloud);
  // ec.extract (cluster_indices);


  // pointcloud_msgs::PointCloud2_Segments msg_;
  // std::vector<sensor_msgs::PointCloud2> temp_clusters;

  // int j = 0;
  // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, j++)
  // {
  //   pcl::PointCloud<PointNormal>::Ptr cloud_cluster_don (new pcl::PointCloud<PointNormal>);
  //   for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
  //   {
  //     cloud_cluster_don->points.push_back (doncloud->points[*pit]);
  //   }

  //   cloud_cluster_don->width = int (cloud_cluster_don->points.size ());
  //   cloud_cluster_don->height = 1;
  //   cloud_cluster_don->is_dense = true;

  //       sensor_msgs::PointCloud2 msgout;
  //       pcl::PCLPointCloud2 cloud2;
  //       pcl::toPCLPointCloud2(*cloud_cluster_don, cloud2);

  //       pcl_conversions::fromPCL(cloud2, msgout);
  //       // msg_.clusters.push_back(msgout);
  //       temp_clusters.push_back(msgout);

  // }







    //Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (clusterTolerance);// 2cm
    ec.setMinClusterSize (minClusterSize); //100
    ec.setMaxClusterSize (maxClusterSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);


    pointcloud_msgs::PointCloud2_Segments msg_;


    std::vector<sensor_msgs::PointCloud2> temp_clusters;


    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            cloud_cluster->points.push_back (cloud->points[*pit]); 
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

        sensor_msgs::PointCloud2 msgout;
        pcl::PCLPointCloud2 cloud2;
        pcl::toPCLPointCloud2(*cloud_cluster, cloud2);

        pcl_conversions::fromPCL(cloud2, msgout);

        temp_clusters.push_back(msgout);
        //msg_.clusters.push_back(msgout);
    }


    if(moving_clusters_only==true){


        int initial_size = temp_clusters.size();

        //int numOfPoints;
        float maxz, minz, maxy, miny, maxy_x, miny_x, maxx, minx, maxx_y, minx_y, minz_maxz, minz_minz, minz_maxy, minz_miny, minz_maxy_x, minz_miny_x, minz_maxx, minz_minx, minz_maxx_y, minz_minx_y, total_maxy, total_miny, total_maxx, total_minx;
        bool linear_flag;

        for(int i=initial_size-1; i>=0; i--){

            pcl::PCLPointCloud2 pc2;
            pcl_conversions::toPCL ( temp_clusters[i] , pc2 );
            pcl::PointCloud<pcl::PointXYZ> cloud2;
            pcl::fromPCLPointCloud2 ( pc2 , cloud2 );


            
            linear_flag=false;

            maxz=cloud2.points[0].z;
            minz=cloud2.points[0].z;

            maxy=cloud2.points[0].y;
            miny=cloud2.points[0].y;
            maxy_x=cloud2.points[0].x;
            miny_x=cloud2.points[0].x;

            maxx=cloud2.points[0].x;
            minx=cloud2.points[0].x;
            maxx_y=cloud2.points[0].y;
            minx_y=cloud2.points[0].y;


            minz_maxz=cloud2.points[0].z;
            minz_minz=cloud2.points[0].z;

            minz_maxy=cloud2.points[0].y;
            minz_miny=cloud2.points[0].y;
            minz_maxy_x=cloud2.points[0].x;
            minz_miny_x=cloud2.points[0].x;

            minz_maxx=cloud2.points[0].x;
            minz_minx=cloud2.points[0].x;
            minz_maxx_y=cloud2.points[0].y;
            minz_minx_y=cloud2.points[0].y;

            total_maxy=cloud2.points[0].y;
            total_miny=cloud2.points[0].y;
            total_maxx=cloud2.points[0].x;
            total_minx=cloud2.points[0].x;


             std::vector<std::vector<double>> min_max_points;

             

             min_max_points.push_back(std::vector<double>(30, std::numeric_limits<double>::lowest()));
             min_max_points.push_back(std::vector<double>(30, std::numeric_limits<double>::max()));
             min_max_points.push_back(std::vector<double>(30, -0.01));
             min_max_points.push_back(std::vector<double>(30, 40.00));

             int index= (100*cloud2.points[0].z)/5;

             if(min_max_points[0][index]<cloud2.points[0].y) min_max_points[0][index]= cloud2.points[0].y;
             if(min_max_points[1][index]>cloud2.points[0].y) min_max_points[1][index]= cloud2.points[0].y;
             if(min_max_points[2][index]<cloud2.points[0].x) min_max_points[2][index]= cloud2.points[0].x;
             if(min_max_points[3][index]>cloud2.points[0].x) min_max_points[3][index]= cloud2.points[0].x;



            for(int j=1; j < cloud2.points.size(); j++){   //find max and min z of cluster

                index= (100*cloud2.points[j].z)/5;

                if(min_max_points[0][index]<cloud2.points[j].y) min_max_points[0][index]= cloud2.points[j].y;
                if(min_max_points[1][index]>cloud2.points[j].y) min_max_points[1][index]= cloud2.points[j].y;
                if(min_max_points[2][index]<cloud2.points[j].x) min_max_points[2][index]= cloud2.points[j].x;
                if(min_max_points[3][index]>cloud2.points[j].x) min_max_points[3][index]= cloud2.points[j].x;






                if(cloud2.points[j].z > maxz){
                    maxz = cloud2.points[j].z;

                    maxy=cloud2.points[j].y;
                    miny=cloud2.points[j].y;
                    maxy_x=cloud2.points[j].x;
                    miny_x=cloud2.points[j].x;

                    maxx=cloud2.points[j].x;
                    minx=cloud2.points[j].x;
                    maxx_y=cloud2.points[j].y;
                    minx_y=cloud2.points[j].y;
                }
                else if(cloud2.points[j].z == maxz){

                    if(cloud2.points[j].y > maxy){
                        maxy = cloud2.points[j].y;
                        maxy_x = cloud2.points[j].x;
                    }
                    if(cloud2.points[j].y < miny){
                        miny = cloud2.points[j].y;
                        miny_x = cloud2.points[j].x;
                    }
                    if(cloud2.points[j].x > maxx){
                        maxx = cloud2.points[j].x;
                        maxx_y = cloud2.points[j].y;
                    }
                    if(cloud2.points[j].x < minx){
                        minx = cloud2.points[j].x;
                        minx_y = cloud2.points[j].y;
                    }
                } 
                if(cloud2.points[j].z < minz){

                    minz = cloud2.points[j].z;
                    
                    minz_maxy=cloud2.points[j].y;
                    minz_miny=cloud2.points[j].y;
                    minz_maxy_x=cloud2.points[j].x;
                    minz_miny_x=cloud2.points[j].x;

                    minz_maxx=cloud2.points[j].x;
                    minz_minx=cloud2.points[j].x;
                    minz_maxx_y=cloud2.points[j].y;
                    minz_minx_y=cloud2.points[j].y;
                }
                else if(cloud2.points[j].z == minz){

                    if(cloud2.points[j].y > minz_maxy){
                        minz_maxy = cloud2.points[j].y;
                        minz_maxy_x = cloud2.points[j].x;
                    }
                    if(cloud2.points[j].y < minz_miny){
                        minz_miny = cloud2.points[j].y;
                        minz_miny_x = cloud2.points[j].x;
                    }
                    if(cloud2.points[j].x > minz_maxx){
                        minz_maxx = cloud2.points[j].x;
                        minz_maxx_y = cloud2.points[j].y;
                    }
                    if(cloud2.points[j].x < minz_minx){
                        minz_minx = cloud2.points[j].x;
                        minz_minx_y = cloud2.points[j].y;
                    }
                }
                if(cloud2.points[j].y > total_maxy) total_maxy=cloud2.points[j].y;
                if(cloud2.points[j].y < total_miny) total_miny=cloud2.points[j].y;
                if(cloud2.points[j].x > total_maxx) total_maxx=cloud2.points[j].x;
                if(cloud2.points[j].x < total_minx) total_minx=cloud2.points[j].x;    
            }
            // double max_ylinear_dist=0;
            // double max_xlinear_dist=0;
            //         int linear_counter=0;
            //         double max_slote=0;

            if(linear_check == true){

                // numOfPoints= cloud2.points.size();
            
                //if((maxy!=miny or maxy_x!=miny_x) and maxx!=minx  and cloud2.points.size()>3 and (minz_maxy!=minz_miny or minz_maxy_x!=minz_miny_x) and minz_maxx!=minz_minx){ 

                    // double yy = maxy-miny;
                    // double yx = maxy_x-miny_x;
                    // double xx = maxx-minx;
                    // double xy = maxx_y-minx_y;
                    // double disty, distx;


                    // double minz_yy = minz_maxy-minz_miny;
                    // double minz_yx = minz_maxy_x-minz_miny_x;
                    // double minz_xx = minz_maxx-minz_minx;
                    // double minz_xy = minz_maxx_y-minz_minx_y;
                    // double minz_disty, minz_distx;

                    // int minz_linear_counter=0;

                    // for(int j=0; j < cloud2.points.size(); j++){

                    //     disty = abs(yx*cloud2.points[j].y - yy*cloud2.points[j].x + maxy*miny_x -maxy_x*miny) / sqrt(pow(yy,2)+pow(yx,2));
                    //     distx = abs(xx*cloud2.points[j].y - xy*cloud2.points[j].x + maxx*minx_y -maxx_y*minx) / sqrt(pow(xy,2)+pow(xx,2));
                        // if(disty>linear_dist and distx>linear_dist){
                        // if(disty>linear_dist){
                        //     linear_counter++;
                        // }
                        // if(disty> max_ylinear_dist){
                        //     max_ylinear_dist=disty;
                        //     linear_counter=0;
                        // }
                        // else if(disty == max_ylinear_dist){
                        //     linear_counter++;
                        // }

                        // if(distx> max_xlinear_dist){
                        //     max_xlinear_dist=disty;
                        // }
                        // double slote= abs(yx/yy - (maxy_x-cloud2.points[j].x)/(maxy-cloud2.points[j].y));
                        // if(max_slote< slote) max_slote=slote;

                        // if(cloud2.points[j].x >= minz_minx and cloud2.points[j].x <= minz_maxx and cloud2.points[j].y >= minz_miny and cloud2.points[j].y <= minz_maxy) linear_counter++;


                        // minz_disty = abs(minz_yx*cloud2.points[j].y - minz_yy*cloud2.points[j].x + minz_maxy*minz_miny_x -minz_maxy_x*minz_miny) / sqrt(pow(minz_yy,2)+pow(minz_yx,2));
                        // minz_distx = abs(minz_xx*cloud2.points[j].y - minz_xy*cloud2.points[j].x + minz_maxx*minz_minx_y -minz_maxx_y*minz_minx) / sqrt(pow(minz_xy,2)+pow(minz_xx,2));
                        // if(minz_disty>linear_dist and minz_distx>linear_dist){
                        //     minz_linear_counter++;
                        // }
                        // if(minz_maxy >= cloud2.points[j].y and minz_maxy >= cloud2.points[j].y )
                    // }
                    // if(linear_counter>=0.1*cloud2.points.size() or minz_linear_counter>=0.1*cloud2.points.size()) linear_flag=false;
                    // if(minz_linear_counter>=0.1*cloud2.points.size()) linear_flag=false;
                //}

                // if(linear_flag==true){
                // if(abs(maxy-miny)<0.08 or abs(maxx-minx)<0.08){
                //     msg_.stationary_clusters.push_back(temp_clusters[i]);
                //     temp_clusters.erase(temp_clusters.begin()+i);
                //     linear_flag=true;
                //   //std::cout << "Linear!!" << std::endl;
                // }
            }



            if(vertical_check==true and linear_flag==false){
            // if(vertical_check==true and linear_flag!=false){

                pcl::PointCloud<pcl::PointXYZ> pczmax;
                pcl::PointCloud<pcl::PointXYZ> pczmin;

                pczmax=saveAllZPointsFrom(temp_clusters[i], (3*abs(maxz - minz)/4)+minz);
                pczmin=saveAllZPointsUntil(temp_clusters[i],  (abs(maxz - minz)/4)+minz);
                // pczmin=saveAllZValuePoints(temp_clusters[i], minz);
                bool samepoints = true;

                for(int k=0; k < pczmax.points.size(); k++){

                    if(pczmax.points[k].y > maxy){
                        maxy = pczmax.points[k].y;
                        maxy_x = pczmax.points[k].x;
                    }
                    if(pczmax.points[k].y < miny){
                        miny = pczmax.points[k].y;
                        miny_x = pczmax.points[k].x;
                    }
                    if(pczmax.points[k].x > maxx){
                        maxx = pczmax.points[k].x;
                        maxx_y = pczmax.points[k].y;
                    }
                    if(pczmax.points[k].x < minx){
                        minx = pczmax.points[k].x;
                        minx_y = pczmax.points[k].y;
                    }
                }


                int counter=0;


                // for(int k=0; k < pczmax.points.size(); k++){
                for(int k=0; k < cloud2.points.size(); k++){        


                    // double dist_y= sqrt(pow( maxy- miny, 2)+pow(maxy_x - miny_x, 2));
                    // double dist_x= sqrt(pow( maxx_y-minx_y, 2)+pow(maxx-minx, 2));

                    // double dist_maxy= sqrt(pow( maxy- pczmax.points[k].y, 2)+pow(maxy_x - pczmax.points[k].x, 2));
                    // double dist_miny= sqrt(pow( miny- pczmax.points[k].y, 2)+pow(miny_x - pczmax.points[k].x, 2));
                    // double dist_maxx= sqrt(pow( maxx_y- pczmax.points[k].y, 2)+pow(maxx - pczmax.points[k].x, 2));
                    // double dist_minx= sqrt(pow( minx_y- pczmax.points[k].y, 2)+pow(minx - pczmax.points[k].x, 2));


                    // if(pczmax.points[k].x >= minx && pczmax.points[k].x <= maxx && pczmax.points[k].y >= miny && pczmax.points[k].y <= maxy) counter++;
                    if(cloud2.points[k].x >= minx && cloud2.points[k].x <= maxx && cloud2.points[k].y >= miny && cloud2.points[k].y <= maxy) counter++;

                    // if(dist_maxy > dist_y + vertical_dist or dist_miny > dist_y + vertical_dist or dist_maxx > dist_x + vertical_dist or dist_minx > dist_x + vertical_dist){
                    //     samepoints = false;
                    //     // break;
                    // }
                }

                //if(counter>=0.1*pczmin.points.size() and samepoints==false) samepoints=true;

                if(counter-pczmax.points.size() <= 0.6*(cloud2.points.size()-pczmax.points.size())) samepoints=false;
                if((counter/4) >= 0.8*pczmax.points.size()){ samepoints=true; 
                    // std::cout << "90%!!" << std::endl; 
                }
            
               if(samepoints==true){

                   // std::cout << "Vertical!!" << std::endl;
                    msg_.stationary_clusters.push_back(temp_clusters[i]);
                    temp_clusters.erase(temp_clusters.begin()+i);        
                }
                else{
                    std::cout << "---" << std::endl;
                    std::cout << "disty = " << abs(maxy-miny) << std::endl;
                    std::cout << "distx = " << abs(maxx-minx) << std::endl;
                    std::cout << "distz = " << abs(maxz-minz) << std::endl;
                    std::cout << "total_points = " << cloud2.points.size() << std::endl;
                    std::cout << "pczmax_points = " << pczmax.points.size() << std::endl;
                    std::cout << "pczmin_points = " << pczmin.points.size() << std::endl;
                    //std::cout << "linear_counter_% = " << linear_counter*100/cloud2.points.size() << std::endl;
                    // std::cout << "max_ylinear_dist = " << max_ylinear_dist << std::endl;
                    // std::cout << "max_xlinear_dist = " << max_xlinear_dist << std::endl;
                    // std::cout << "max_slote= " << max_slote<< std::endl;
                }        
            }

            // double mean_maxy=0, mean_miny=0, mean_maxx=0, mean_minx=0, sd_maxy=0, sd_miny=0, sd_maxx=0, sd_minx=0;

            int counterx=0, countery=0, counterz=0;
            bool counterz_flag=true;
            double prev_zmax, prev_zmin;
            int tempj=30, smaller_maxy=0, bigger_maxy=0, smaller_miny=0, bigger_miny=0, smaller_maxx=0, bigger_maxx=0, smaller_minx=0, bigger_minx=0;

            std::cout << "---" << std::endl;

            if((maxy>minz_maxy+0.09 and miny>minz_miny+0.09) or (maxy<minz_maxy-0.09 and miny<minz_miny-0.09)){
                for(int j=0; j < 30; j++){
                    if(min_max_points[0][j]!=std::numeric_limits<double>::lowest() and min_max_points[1][j]!= std::numeric_limits<double>::max()){
                        prev_zmax=min_max_points[0][j];
                        prev_zmin=min_max_points[1][j];
                        tempj=j;
                        counterz_flag=false;
                        counterz++;
                        break;
                    }
                }
                for(int j=tempj+1; j < 30; j++){
                // std::cout << "min_max_points[0] " << min_max_points[0][j] << std::endl;
                // std::cout << "min_max_points[1] " << min_max_points[1][j] << std::endl;
                    if(min_max_points[0][j]!=std::numeric_limits<double>::lowest() and min_max_points[1][j]!= std::numeric_limits<double>::max()){
                        // mean_maxy+=min_max_points[0][j];
                        // mean_miny+=min_max_points[1][j]; 
                        // counterz_flag=false;
                        if(min_max_points[0][j]>prev_zmax) bigger_maxy++;
                        else if(min_max_points[0][j]<prev_zmax) smaller_maxy++;

                        if(min_max_points[1][j]>prev_zmax) bigger_miny++;
                        else if(min_max_points[1][j]<prev_zmax) smaller_miny++;
                        counterz++;
                        prev_zmax=min_max_points[0][j];
                        prev_zmin=min_max_points[1][j];
                        // if(abs(min_max_points[0][j]-min_max_points[1][j])>= std::max(abs(maxy-minz_miny), abs(miny-minz_maxy))) countery++;
                    }
                }
                std::cout << "bigger_maxy = " << bigger_maxy << " smaller_maxy = " << smaller_maxy << " bigger_miny = " << bigger_miny << " smaller_miny = " << smaller_miny << " counterz = " << counterz  << std::endl;

                //std::cout << "maxy = " << maxy << " minz_maxy = " << minz_maxy <<  " dist_maxy = " << abs(maxy-minz_maxy) << std::endl;
                //std::cout << "miny = " << miny << " minz_miny = " << minz_miny << "dist_miny = " << abs(miny-minz_miny) << std::endl;
                


                // mean_maxy/=counterz;
                // mean_miny/=counterz;

                // for(int j=0; j < 30; j++){
                //     if(min_max_points[0][j]!=std::numeric_limits<double>::lowest() and min_max_points[1][j]!= std::numeric_limits<double>::max()){
                //         sd_maxy+=pow(min_max_points[0][j]-mean_maxy,2);
                //         sd_miny+=pow(min_max_points[1][j]-mean_miny,2); 
                //     }
                // }

                // sd_maxy=sqrt((sd_maxy/counterz));
                // sd_miny=sqrt((sd_miny/counterz));
                // sd_maxy=(sd_maxy/abs(mean_maxy))*100;
                // sd_miny=(sd_miny/abs(mean_miny))*100;
            }
            if((maxx>minz_maxx+0.09 and minx>minz_minx+0.09) or (maxx<minz_maxx-0.09 and minx<minz_minx-0.09)){

                tempj=30;

                for(int j=0; j < 30; j++){
                    if(min_max_points[2][j]!=-0.01 and min_max_points[3][j]!= 40.00){
                        prev_zmax=min_max_points[2][j];
                        prev_zmin=min_max_points[3][j];
                        tempj=j;
                        if(counterz_flag==true) counterz++;
                        break;
                    }
                }

                for(int j=tempj+1; j < 30; j++){
                // std::cout << "min_max_points[2] " << min_max_points[2][j] << std::endl;
                // std::cout << "min_max_points[3] " << min_max_points[3][j] << std::endl;
                    if(min_max_points[2][j]!=-0.01 and min_max_points[3][j]!= 40.00){
                        // mean_maxx+=min_max_points[2][j];
                        // mean_minx+=min_max_points[3][j];
                        if(min_max_points[2][j]>prev_zmax) bigger_maxx++;
                        else if(min_max_points[3][j]<prev_zmax) smaller_maxx++;

                        if(min_max_points[2][j]>prev_zmax) bigger_minx++;
                        else if(min_max_points[3][j]<prev_zmax) smaller_minx++;

                        prev_zmax=min_max_points[2][j];
                        prev_zmin=min_max_points[3][j];
                        if(counterz_flag==true) counterz++;
                        // if(abs(min_max_points[2][j]-min_max_points[3][j])>= std::max(abs(maxx-minz_minx), abs(minx-minz_maxx))) counterx++;
                    }
                }
                // std::cout << "maxx = " << maxx << " minz_maxx = " << minz_maxx << "dist_maxx = " << abs(maxx-minz_maxx) << std::endl;
                // std::cout << "minx = " << minx << " minz_minx = " << minz_minx << "dist_minx= " << abs(minx-minz_minx) << std::endl;
                std::cout << "bigger_maxx = " << bigger_maxx << " smaller_maxx = " << smaller_maxx << " bigger_minx = " << bigger_minx << " smaller_minx = " << smaller_minx << " counterz = " << counterz << std::endl;


                // mean_maxx/=counterz;
                // mean_minx/=counterz;
                // for(int j=0; j < 30; j++){
                //     if(min_max_points[2][j]!=-0.01 and min_max_points[3][j]!= 40.00){
                //         sd_maxx+=pow(min_max_points[2][j]-mean_maxx,2);
                //         sd_minx+=pow(min_max_points[3][j]-mean_minx,2); 
                //     }
                // }

                // sd_maxx=sqrt((sd_maxx/counterz));
                // sd_minx=sqrt((sd_minx/counterz));
                // sd_maxx=(sd_maxx/abs(mean_maxx))*100;
                // sd_minx=(sd_minx/abs(mean_minx))*100;

            }
            

            

            if(!((maxy>minz_maxy+0.09 and miny>minz_miny+0.09) or (maxx>minz_maxx+0.09 and minx>minz_minx+0.09) or (maxy<minz_maxy-0.09 and miny<minz_miny-0.09) or (maxx<minz_maxx-0.09 and minx<minz_minx-0.09))){ 
                //or (sd_miny<8 and sd_maxy<8 and sd_minx<8 and sd_maxx<8) ){
                msg_.stationary_clusters.push_back(temp_clusters[i]);
                temp_clusters.erase(temp_clusters.begin()+i);
            }
            else{
                std::cout << "total_disty = " << abs(total_maxy-total_miny) << std::endl;
                std::cout << "total_distx = " << abs(total_maxx-total_minx) << std::endl;
                std::cout << "total_distz = " << abs(maxz-minz) << std::endl;
                // std::cout << "total_points = " << cloud2.points.size() << std::endl;
                // std::cout << "counterz = " << counterz << std::endl;
                // std::cout << "counterx% = " << counterx*100/counterz << std::endl;
                // std::cout << "countery% = " << countery*100/counterz << std::endl;
                // std::cout << "mean_maxy = " << mean_maxy << " sd_maxy = " << sd_maxy << std::endl;
                // std::cout << "mean_miny = " << mean_miny << " sd_miny = " << sd_miny << std::endl;
                // std::cout << "mean_maxx = " << mean_maxx << " sd_maxx = " << sd_maxx << std::endl;
                // std::cout << "mean_minx = " << mean_minx << " sd_minx = " << sd_minx << std::endl;  




                 //                 pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
                 //                   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
                                 
                 //    pcl::fromPCLPointCloud2(pc2, *cloud);
                 //  ne.setInputCloud (cloud);

                 //  // Create an empty kdtree representation, and pass it to the normal estimation object.
                 //  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
                 //  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
                 //  ne.setSearchMethod (tree);

                 //  // Output datasets
                 //  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

                 //  // Use all neighbors in a sphere of radius 3cm
                 //  ne.setRadiusSearch (0.03);

                 //  // Compute the features
                 //  ne.compute (*cloud_normals);

                 //  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
                 // viewer.setBackgroundColor (0.0, 0.0, 0.5);
                 // viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals);

                 // while (!viewer.wasStopped ())  // THE ORGINAL !viewer.wasStopped () 
                 //         {
                 //           viewer.spinOnce ();
                 //         }


  //for(int j=0; j<cloud_normals->points.size(); j++) std::cout << cloud_normals->points[j]<< std::endl;

  // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*



            }

            min_max_points.clear();       
        }   
    }

    

    msg_.clusters=temp_clusters;

    msg_.header.stamp = ros::Time::now();
    msg_.header.frame_id = c_.header.frame_id;
    msg_.factor = c_.factor;
    msg_.overlap = c_.overlap;
    msg_.first_stamp = c_.first_stamp;
    msg_.num_scans = c_.num_scans ;
    msg_.angle_min = c_.angle_min ;
    msg_.angle_max = c_.angle_max ;
    msg_.angle_increment = c_.angle_increment;
    msg_.range_min = c_.range_min;
    msg_.range_max = c_.range_max;
    msg_.scan_time = c_.scan_time;
    msg_.rec_time = c_.rec_time;
    msg_.middle_z = c_.middle_z;
    msg_.idForTracking = c_.idForTracking; 
// if(msg_.clusters.size()>0) pub.publish(msg_);
    if(temp_clusters.size()>0) pub.publish(msg_);
    else if(msg_.stationary_clusters.size()>0) stationary_pub.publish(msg_);    
}

int main (int argc, char** argv){
    ros::init (argc, argv, "pointcloud2_clustering");
    ros::NodeHandle n_;


    n_.param("pointcloud2_clustering/maxIterations", maxIterations, 100);
    n_.param("pointcloud2_clustering/distanceThreshold", distanceThreshold, 0.01);
    n_.param("pointcloud2_clustering/clusterTolerance", clusterTolerance, 0.4);
    n_.param("pointcloud2_clustering/minClusterSize", minClusterSize, 10);
    n_.param("pointcloud2_clustering/maxClusterSize", maxClusterSize, 25000);
    n_.param("pointcloud2_clustering/percentOfpoints", percentOfpoints, 0.20);
    n_.param("pointcloud2_clustering/moving_clusters_only", moving_clusters_only, false);
    n_.param("pointcloud2_clustering/vertical_check", vertical_check, false);
    n_.param("pointcloud2_clustering/linear_check", linear_check, false);
    n_.param("pointcloud2_clustering/linear_dist", linear_dist, 0.2);
    n_.param("pointcloud2_clustering/vertical_dist", vertical_dist, 0.3);


    std::string topic;
    std::string out_topic;
    std::string stationary_topic;
    n_.param("pointcloud2_clustering/topic", topic, std::string("laserscan_stacker/scans"));
    n_.param("pointcloud2_clustering/out_topic", out_topic, std::string("pointcloud2_clustering/clusters"));
    n_.param("pointcloud2_clustering/stationary_topic", stationary_topic, std::string("pointcloud2_clustering/stationary_clusters"));


    ros::Subscriber sub = n_.subscribe (topic, 1, cloud_callback);

    pub = n_.advertise<pointcloud_msgs::PointCloud2_Segments> (out_topic, 1);
    stationary_pub = n_.advertise<pointcloud_msgs::PointCloud2_Segments> (stationary_topic, 1);

    ros::spin ();
}