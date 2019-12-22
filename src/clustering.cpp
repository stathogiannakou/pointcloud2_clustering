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


using namespace pcl;
using namespace std;


typedef pcl::PointXYZI PointTypeIO;


ros::Publisher pub, stationary_pub;

double clusterTolerance, distanceThreshold, percentOfpoints;

int maxIterations;
int minClusterSize, maxClusterSize;

bool moving_clusters_only;
double general_variable;









void cloud_callback (const pointcloud_msgs::PointCloud2_Segments& c_)
{

    pcl::PCLPointCloud2 cloud2;

    pcl_conversions::toPCL( c_.clusters[0] , cloud2);
    pcl::PointCloud<PointTypeIO>::Ptr cloud(new pcl::PointCloud<PointTypeIO> ());
    pcl::fromPCLPointCloud2(cloud2, *cloud);

    //Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<PointTypeIO> vg;
    pcl::PointCloud<PointTypeIO>::Ptr cloud_filtered (new pcl::PointCloud<PointTypeIO>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<PointTypeIO> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<PointTypeIO>::Ptr cloud_plane (new pcl::PointCloud<PointTypeIO> ());
    pcl::PointCloud<PointTypeIO>::Ptr cloud_f (new pcl::PointCloud<PointTypeIO> ());
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
        pcl::ExtractIndices<PointTypeIO> extract;
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



    //Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointTypeIO>::Ptr tree (new pcl::search::KdTree<PointTypeIO>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointTypeIO> ec;
    ec.setClusterTolerance (clusterTolerance);// 2cm
    ec.setMinClusterSize (minClusterSize); //100
    ec.setMaxClusterSize (maxClusterSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);


    pointcloud_msgs::PointCloud2_Segments msg_;


    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {

        pcl::PointCloud<PointTypeIO>::Ptr cloud_cluster (new pcl::PointCloud<PointTypeIO>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]); 
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

        sensor_msgs::PointCloud2 msgout;
        pcl::PCLPointCloud2 cloud2;
        pcl::toPCLPointCloud2(*cloud_cluster, cloud2);

        pcl_conversions::fromPCL(cloud2, msgout);

        msg_.clusters.push_back(msgout);
    }


    if(moving_clusters_only==true){

        int initial_size =  msg_.clusters.size();

        //int numOfPoints;
        float maxz, minz, maxy, miny, maxy_x, miny_x, maxx, minx, maxx_y, minx_y, minz_maxz, minz_minz, minz_maxy, minz_miny, minz_maxy_x, minz_miny_x, minz_maxx, minz_minx, minz_maxx_y, minz_minx_y, total_maxy, total_miny, total_maxx, total_minx;
        float totalx_dist, totaly_dist;

        for(int i=initial_size-1; i>=0; i--){

            pcl::PCLPointCloud2 pc2;
            pcl_conversions::toPCL (  msg_.clusters[i] , pc2 );
            pcl::PointCloud<PointTypeIO> cloud2;
            pcl::fromPCLPointCloud2 ( pc2 , cloud2 );

            

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

             min_max_points.push_back(std::vector<double>(30, 0.00));

             int index= (100*cloud2.points[0].z)/5;

             if(min_max_points[0][index]<cloud2.points[0].y) min_max_points[0][index]= cloud2.points[0].y;
             if(min_max_points[1][index]>cloud2.points[0].y) min_max_points[1][index]= cloud2.points[0].y;
             if(min_max_points[2][index]<cloud2.points[0].x) min_max_points[2][index]= cloud2.points[0].x;
             if(min_max_points[3][index]>cloud2.points[0].x){
                min_max_points[3][index]= cloud2.points[0].x;
                min_max_points[4][index]= cloud2.points[0].y;
            }




            for(int j=1; j < cloud2.points.size(); j++){   //find max and min z of cluster

                index= (100*cloud2.points[j].z)/5;

                if(min_max_points[0][index]<cloud2.points[j].y) min_max_points[0][index]= cloud2.points[j].y;
                if(min_max_points[1][index]>cloud2.points[j].y) min_max_points[1][index]= cloud2.points[j].y;
                if(min_max_points[2][index]<cloud2.points[j].x) min_max_points[2][index]= cloud2.points[j].x;
                if(min_max_points[3][index]>cloud2.points[j].x){
                    min_max_points[3][index]= cloud2.points[j].x;
                    min_max_points[4][index]= cloud2.points[j].y;
                }



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



            int counterx=0, countery=0, counterz=0, countery_dist=0, counterx_dist=0, check_curv=0, count_distx=0, count_disty=0;
            bool counterz_flag=true;
            std::cout << "---" << std::endl;
            bool erase_flag = false;


            if(abs(maxy-miny)<0.02 or abs(minz_maxy-minz_miny)<0.02 or abs(maxx-minx)<0.01 or abs(minz_maxx-minz_minx)<0.01 or (abs(minz_minx-minx)<general_variable and abs(minz_minx_y- minx_y)<general_variable)){
                msg_.stationary_clusters.push_back( msg_.clusters[i]);
                 msg_.clusters.erase( msg_.clusters.begin()+i);
            }
            else{
                if(maxy>minz_maxy+0.09 and miny>minz_miny+0.09){
                    
                    for(int j=0; j < 15; j++){
                    // std::cout << "min_max_points[0] " << min_max_points[0][j] << std::endl;
                    // std::cout << "min_max_points[1] " << min_max_points[1][j] << std::endl;
                        if(min_max_points[0][j]!=std::numeric_limits<double>::lowest() and min_max_points[1][j]!= std::numeric_limits<double>::max()){
                            if(min_max_points[4][j]==min_max_points[0][j] or min_max_points[4][j]==min_max_points[1][j]) check_curv++;
                            if(maxy<min_max_points[0][j]) countery++;
                            if(min_max_points[0][j]+0.01>=maxy or min_max_points[1][j]-0.01<=minz_miny) count_disty++;
                            counterz_flag=false;
                            counterz++;
                            // if(abs(min_max_points[0][j]-min_max_points[1][j])>= std::max(abs(maxy-minz_miny), abs(miny-minz_maxy))) countery++;
                        }
                    }
                    for(int j=15; j < 30; j++){
                    // std::cout << "min_max_points[0] " << min_max_points[0][j] << std::endl;
                    // std::cout << "min_max_points[1] " << min_max_points[1][j] << std::endl;
                        if(min_max_points[0][j]!=std::numeric_limits<double>::lowest() and min_max_points[1][j]!= std::numeric_limits<double>::max()){
                            if(min_max_points[4][j]==min_max_points[0][j] or min_max_points[4][j]==min_max_points[1][j]) check_curv++;
                            if(maxy<min_max_points[0][j]) countery++;
                            if(min_max_points[0][j]+0.01>=maxy or min_max_points[1][j]-0.01<=minz_miny) count_disty++;
                            counterz_flag=false;
                            counterz++;
                            if(abs(maxy-min_max_points[0][j])<=0.02) countery_dist++;
                            // if(abs(min_max_points[0][j]-min_max_points[1][j])>= std::max(abs(maxy-minz_miny), abs(miny-minz_maxy))) countery++;
                        }
                    }

                    //std::cout << "maxy = " << maxy << " minz_maxy = " << minz_maxy <<  " dist_maxy = " << abs(maxy-minz_maxy) << std::endl;
                    //std::cout << "miny = " << miny << " minz_miny = " << minz_miny << "dist_miny = " << abs(miny-minz_miny) << std::endl;             
                }
                else if(maxy<minz_maxy-0.09 and miny<minz_miny-0.09){

                    for(int j=0; j<15; j++){
                    // std::cout << "min_max_points[0] " << min_max_points[0][j] << std::endl;
                    // std::cout << "min_max_points[1] " << min_max_points[1][j] << std::endl;
                        if(min_max_points[0][j]!=std::numeric_limits<double>::lowest() and min_max_points[1][j]!= std::numeric_limits<double>::max()){
                            if(min_max_points[4][j]==min_max_points[0][j] or min_max_points[4][j]==min_max_points[1][j]) check_curv++;
                            if(miny>min_max_points[1][j]) countery++;
                            if(min_max_points[0][j]+0.01>=minz_maxy or min_max_points[1][j]-0.01<=miny) count_disty++;
                            counterz_flag=false;
                            counterz++;
                            // if(abs(min_max_points[0][j]-min_max_points[1][j])>= std::max(abs(maxy-minz_miny), abs(miny-minz_maxy))) countery++;
                        }
                    }
                    for(int j=15; j<30; j++){
                    // std::cout << "min_max_points[0] " << min_max_points[0][j] << std::endl;
                    // std::cout << "min_max_points[1] " << min_max_points[1][j] << std::endl;
                        if(min_max_points[0][j]!=std::numeric_limits<double>::lowest() and min_max_points[1][j]!= std::numeric_limits<double>::max()){
                            if(miny>min_max_points[1][j]) countery++;
                            if(min_max_points[4][j]==min_max_points[0][j] or min_max_points[4][j]==min_max_points[1][j]) check_curv++;
                            if(min_max_points[0][j]+0.01>=minz_maxy or min_max_points[1][j]-0.01<=miny) count_disty++;
                            counterz_flag=false;
                            counterz++;
                            if(abs(miny-min_max_points[1][j])<=0.02) countery_dist++;
                            // if(abs(min_max_points[0][j]-min_max_points[1][j])>= std::max(abs(maxy-minz_miny), abs(miny-minz_maxy))) countery++;
                        }
                    }

                }


                if(maxx>minz_maxx+0.09 and minx>minz_minx+0.09){

                    for(int j=0; j < 15; j++){
                    // std::cout << "min_max_points[2] " << min_max_points[2][j] << std::endl;
                    // std::cout << "min_max_points[3] " << min_max_points[3][j] << std::endl;
                        if(min_max_points[2][j]!=-0.01 and min_max_points[3][j]!= 40.00){
                            if(maxx<min_max_points[2][j]) counterx++;
                            if(counterz_flag==true){
                                counterz++;
                                if(min_max_points[4][j]==min_max_points[0][j] or min_max_points[4][j]==min_max_points[1][j]) check_curv++;
                            }
                            // if(abs(min_max_points[2][j]-min_max_points[3][j])>= std::max(abs(maxx-minz_minx), abs(minx-minz_maxx))) counterx++;
                            if(min_max_points[2][j]+0.01>=maxx or min_max_points[3][j]-0.01<=minz_minx) count_distx++;
                        }
                    }
                    for(int j=15; j < 30; j++){
                    // std::cout << "min_max_points[2] " << min_max_points[2][j] << std::endl;
                    // std::cout << "min_max_points[3] " << min_max_points[3][j] << std::endl;
                        if(min_max_points[2][j]!=-0.01 and min_max_points[3][j]!= 40.00){
                            if(maxx<min_max_points[2][j]) counterx++;
                            if(counterz_flag==true){
                                counterz++;
                                if(min_max_points[4][j]==min_max_points[0][j] or min_max_points[4][j]==min_max_points[1][j]) check_curv++;
                            }
                            if(abs(maxx-min_max_points[2][j])<=0.02) counterx_dist++;
                            // if(abs(min_max_points[2][j]-min_max_points[3][j])>= std::max(abs(maxx-minz_minx), abs(minx-minz_maxx))) counterx++;
                            if(min_max_points[2][j]+0.01>=maxx or min_max_points[3][j]-0.01<=minz_minx) count_distx++;
                        }
                    }
                    // std::cout << "maxx = " << maxx << " minz_maxx = " << minz_maxx << "dist_maxx = " << abs(maxx-minz_maxx) << std::endl;
                    // std::cout << "minx = " << minx << " minz_minx = " << minz_minx << "dist_minx= " << abs(minx-minz_minx) << std::endl;

                } 
                else if (maxx<minz_maxx-0.09 and minx<minz_minx-0.09){

                    for(int j=0; j<15; j++){
                    // std::cout << "min_max_points[2] " << min_max_points[2][j] << std::endl;
                    // std::cout << "min_max_points[3] " << min_max_points[3][j] << std::endl;
                        if(min_max_points[2][j]!=-0.01 and min_max_points[3][j]!= 40.00){
                            if(minx>min_max_points[3][j]) counterx++;
                            if(counterz_flag==true){
                                counterz++;
                                if(min_max_points[4][j]==min_max_points[0][j] or min_max_points[4][j]==min_max_points[1][j]) check_curv++;
                            }
                            // if(abs(min_max_points[2][j]-min_max_points[3][j])>= std::max(abs(maxx-minz_minx), abs(minx-minz_maxx))) counterx++;
                            if(min_max_points[2][j]+0.01>=minz_maxx or min_max_points[3][j]-0.01<=minx) count_distx++;
                        }
                    }
                    for(int j=15; j<30; j++){
                    // std::cout << "min_max_points[2] " << min_max_points[2][j] << std::endl;
                    // std::cout << "min_max_points[3] " << min_max_points[3][j] << std::endl;
                        if(min_max_points[2][j]!=-0.01 and min_max_points[3][j]!= 40.00){
                            if(minx>min_max_points[3][j]) counterx++;
                            if(counterz_flag==true){
                                counterz++;
                                if(min_max_points[4][j]==min_max_points[0][j] or min_max_points[4][j]==min_max_points[1][j]) check_curv++;
                            }
                            if(abs(minx-min_max_points[3][j])<=0.02) counterx_dist++;
                            // if(abs(min_max_points[2][j]-min_max_points[3][j])>= std::max(abs(maxx-minz_minx), abs(minx-minz_maxx))) counterx++;
                            if(min_max_points[2][j]+0.01>=minz_maxx or min_max_points[3][j]-0.01<=minx) count_distx++;
                        }
                    }
                }

                if(!((maxy>minz_maxy+0.09 and miny>minz_miny+0.09) or (maxy<minz_maxy-0.09 and miny<minz_miny-0.09) or (maxx>minz_maxx+0.09 and minx>minz_minx+0.09) or (maxx<minz_maxx-0.09 and minx<minz_minx-0.09)) or counterx>5 or countery>5 or check_curv+2>=counterz){
                    msg_.stationary_clusters.push_back( msg_.clusters[i]);
                     msg_.clusters.erase( msg_.clusters.begin()+i);
                }
                else{
                    // std::cout << "total_disty = " << abs(total_maxy-total_miny) << std::endl;
                    // std::cout << "total_distx = " << abs(total_maxx-total_minx) << std::endl;
                    // std::cout << "total_distz = " << abs(maxz-minz) << std::endl;
                    // std::cout << "total_points = " << cloud2.points.size() << std::endl;
                    // std::cout << "counterz = " << counterz << std::endl;
                    // std::cout << "counterx% = " << counterx*100/counterz << std::endl;
                    // std::cout << "countery% = " << countery*100/counterz << std::endl; 
                    std::cout << "counterx = " << counterx_dist << " countery = " << countery_dist << " counterz = " << counterz  << std::endl;
                    std::cout << "check_curv = " << check_curv << std::endl;
                    std::cout << "count_distx = " << count_distx << " count_disty = " << count_disty << std::endl;
                    
                    // int prev_index=29;
                    // for(int j=0; j<30; j++) if(min_max_points[3][j]!= 40.00){ 
                    //     prev_index=j;
                    //     break;
                    // }
                    // for(int j=prev_index+1; j<30; j++){
                    //     if(min_max_points[3][j]!= 40.00){
                    //         std::cout << (min_max_points[3][prev_index]-min_max_points[3][j])/(min_max_points[4][prev_index]-min_max_points[4][j])  << std::endl;
                    //         prev_index=j;
                    //     }
                    // }

                    //-------------------intensities-------------------------
                    // for(int j=0; j < cloud2.points.size(); j++){   //find max and min z of cluster
                    //     std::cout << "Intensity =  " << cloud2.points[j].intensity << " dist2 = "<< (pow(cloud2.points[j].x,2)+pow(cloud2.points[j].y,2)) << " Energy = " << (cloud2.points[j].intensity*(pow(cloud2.points[j].x,2)+pow(cloud2.points[j].y,2))) << std::endl;

                    // }
                }
            }


            min_max_points.clear();       
        }   
    }

    

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
    if( msg_.clusters.size()>0) pub.publish(msg_);
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
    n_.param("pointcloud2_clustering/general_variable", general_variable, 0.09);


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