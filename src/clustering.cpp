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


ros::Publisher pub;

double clusterTolerance, distanceThreshold, percentOfpoints;

int maxIterations;
int minClusterSize, maxClusterSize;

double minMotionDist=1000.0;
std::vector<Eigen::Vector4f> centroids;

bool moving_clusters_only;



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

    // Creating the KdTree object for the search method of the extraction
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

        float global_maxz=0;
        std::vector<float> maxz(initial_size, 0.0), minz(initial_size, 0.0);
        std::vector<int> numOfPoints(initial_size, 0);


        float maxy, miny, maxy_x, miny_x;
        bool linear_flag;

        for(int i=initial_size-1; i>=0; i--){

            pcl::PCLPointCloud2 pc2;
            pcl_conversions::toPCL ( temp_clusters[i] , pc2 );
            pcl::PointCloud<pcl::PointXYZ> cloud2;
            pcl::fromPCLPointCloud2 ( pc2 , cloud2 );
            
            linear_flag=true;

            maxz[i]=cloud2.points[0].z;
            minz[i]=cloud2.points[0].z;

            maxy=cloud2.points[0].y;
            miny=cloud2.points[0].y;
            maxy_x=cloud2.points[0].x;
            miny_x=cloud2.points[0].x;



            for(int j=1; j < cloud2.points.size(); j++){   //find max and min z of cluster


                if(cloud2.points[j].y > maxy){
                    maxy = cloud2.points[j].y;
                    maxy_x = cloud2.points[j].x;
                }
                if(cloud2.points[j].y < miny){
                    miny = cloud2.points[j].y;
                    miny_x = cloud2.points[j].x;
                }
                if(cloud2.points[j].z > maxz[i]){
                    maxz[i] = cloud2.points[j].z;
                }
                if(cloud2.points[j].z < minz[i]){
                    minz[i] = cloud2.points[j].z;
                }
            }


            numOfPoints[i]= cloud2.points.size();

            if(maxz[i]>global_maxz) global_maxz = maxz[i];        

            if((maxy!=miny or maxy_x!=miny_x)  and numOfPoints[i]>3){ 

                double y = maxy-miny;
                double x = maxy_x-miny_x;
                double dist;

                for(int j=0; j < cloud2.points.size(); j++){

                    dist = abs(x*cloud2.points[j].y - y*cloud2.points[j].x + maxy*miny_x -maxy_x*miny) / sqrt(pow(y,2)+pow(x,2));
                    if(dist>0.15){
                        linear_flag=false;
                        break;
                    }


            
                }
            }

            if(linear_flag==true){

                temp_clusters.erase(temp_clusters.begin()+i);
                maxz.erase(maxz.begin()+i);
                minz.erase(minz.begin()+i);
                numOfPoints.erase(numOfPoints.begin()+i);
                std::cout << "Linear!!" << std::endl;
            }
        }






        initial_size = temp_clusters.size();

        for(int i=initial_size-1; i>=0; i--){

            pcl::PointCloud<pcl::PointXYZ> pczmax;
            pcl::PointCloud<pcl::PointXYZ> pczmin;

            // double max_z, min_z;
            // std::pair<double,double> z_minmax;

            // z_minmax = minmaxz(temp_clusters[i]);
            // max_z = z_minmax.first;
            // min_z = z_minmax.second;

            // pczmax=saveAllZValuePoints(temp_clusters[i], max_z);
            // pczmin=saveAllZValuePoints(temp_clusters[i], min_z);

     //        Eigen::Vector4f max_centroid;
     //        pcl::compute3DCentroid ( pczmax , max_centroid);

     //        Eigen::Vector4f min_centroid;
     //        pcl::compute3DCentroid ( pczmin , min_centroid);

     //        double disttt;

     //        disttt = 1000 * sqrt(pow(max_centroid[0]-min_centroid[0], 2) + pow(max_centroid[1]-min_centroid[1], 2));
     //        if(disttt > minMotionDist){
     // std::cout << "11111!!" << std::endl;
                //                                double check                    //
                pczmax=saveAllZPointsFrom(temp_clusters[i], (3*abs(maxz[i] - minz[i])/4)+minz[i]);
                pczmin=saveAllZPointsUntil(temp_clusters[i],  (abs(maxz[i] - minz[i])/4)+minz[i]);
                bool samepoints = true;
                if(pczmax.size()!=0 and pczmin.size()!=0 ){
                    samepoints=checkforsameXYpoints(pczmax, pczmin);
                }
               if(samepoints==true){


                    std::cout << "444444444!!" << std::endl;

                    temp_clusters.erase(temp_clusters.begin()+i);
                    
                    //std::cout << "cluster in motion id = " << msg.cluster_id[j] << "dist = " << disttt << std::endl;    
                }
            //}
        }






        // initial_size = temp_clusters.size();

        // double total_distz= global_maxz/60.0;

        // for(int i=0; i < initial_size; i++){

        //     double dif_z = maxz[initial_size-1-i] - minz[initial_size-1-i];
        //     double dist_z = dif_z/total_distz;

        //    // if(maxz[initial_size-1-i]==global_maxz and minz[initial_size-1-i]== 0 and numOfPoints[initial_size-1-i]<300 ){

        //     if(numOfPoints[initial_size-1-i]< (300*dist_z)/60.0 ){
        //         temp_clusters.erase (temp_clusters.begin()+initial_size-1-i);
        //         std::cout << "Mpike!!" << std::endl;
        //     }
        // }
    }

    if(temp_clusters.size()>0){

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


        pub.publish(msg_);
    }
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

    std::string topic;
    std::string out_topic;
    n_.param("pointcloud2_clustering/topic", topic, std::string("laserscan_stacker/scans"));
    n_.param("pointcloud2_clustering/out_topic", out_topic, std::string("pointcloud2_clustering/clusters"));

    ros::Subscriber sub = n_.subscribe (topic, 1, cloud_callback);

    pub = n_.advertise<pointcloud_msgs::PointCloud2_Segments> (out_topic, 1);

    ros::spin ();
}