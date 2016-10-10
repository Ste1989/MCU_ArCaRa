/**
*
*  Questo nodo sfrutta la conoscenza di dove si trova l'oggetto nel 2D , e la nuvola
*  di punti ottenuta dalla camera intel r200, per stimare la dimensione dell'oggetto
*/

#include "3Dvision_node.h"


/********************************************************************************
*
*    CALLBACK FEATURES
*
**************************************************************************************/
void Features2DCallback(const obj_detection::Features::ConstPtr& msg)
{

  cout << " ho letto un nuovo dato features" << endl;
  features_packet pkg_features;
  pkg_features.features = msg->features;
  pkg_features.sec = msg->sec;
  pkg_features.nsec = msg->nsec;

  //lo metto nel buffer di ricezione
  buffer_features_packet.push(pkg_features);
  check_syncronization_pkg();

}
/********************************************************************************
*
*    CALLBACK POINT CLOUD
*
**************************************************************************************/
void PointCloudRealSenseCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  cout << " ho letto un nuovo dato point cloud " << endl;
  points_packet pkg_points;
  pkg_points.sec = msg->header.stamp.sec;
  pkg_points.nsec = msg->header.stamp.nsec;
  /*pkg_points.height = msg->height;
  pkg_points.width = msg->width;
  pkg_points.is_bigendian = msg->is_bigendian;
  pkg_points.point_step = msg->point_step;
  pkg_points.row_step = msg->row_step;
  pkg_points.is_dense = msg->is_dense;
  pkg_points.data = msg->data;
  pkg_points.fields = msg->fields;*/
  pkg_points.point_cloud = *msg;

  //lo metto nel buffer di ricezione
  buffer_points_packet.push(pkg_points);
  //controllo se è arrivato il pacchetto features corrispondente
  check_syncronization_pkg();

}
/********************************************************************************
*
*    INIT VARIABILI GLOBALI
*
**************************************************************************************/
void init_global_var(  )
{
  t0_sec = -1;
  
  
}


/********************************************************************************
*
*    MAIN
*
**************************************************************************************/
int main(int argc, char** argv)
{


  ros::init(argc, argv, "obj_3d_detection");
  ros::NodeHandle nh;
  
  //ros::Rate loop_rate(100);
  

  
  //leggo i parametri specificati nel launch file
  
 
 
  //Publisher
  //1
  //2
  

  //Subscriber
  //mi devo sottorscrivere al topic della real sense r200
  point_cloud_sub = nh.subscribe("/camera/depth/points", 1, PointCloudRealSenseCallback);
  features_2D_sub = nh.subscribe("/obj_detection/features", 1, Features2DCallback);



  
  /*inizializzo
  variabili globali ***************************************************************/
  init_global_var();



  /*******************************************************************************************************
                    LOOP
  *********************************************************************************************************/



  while (nh.ok()) 
  {
    //valuta callback
    ros::spinOnce();
  }
}
