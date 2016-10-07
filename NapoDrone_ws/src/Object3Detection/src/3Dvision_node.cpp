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
void Features2DCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  cout << " ho letto un nuovo dato features" << endl;

}
/********************************************************************************
*
*    CALLBACK POINT CLOUD
*
**************************************************************************************/
void PointCloudRealSenseCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  cout << " ho letto un nuovo dato point cloud dovrà essere messo in corrispondeza temporale con la feuatures" << endl;

}
/********************************************************************************
*
*    INIT VARIABILI GLOBALI
*
**************************************************************************************/
void init_global_var(  )
{

 
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
