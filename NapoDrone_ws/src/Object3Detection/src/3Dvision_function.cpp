#include "3Dvision_node.h"
#include <pcl_conversions/pcl_conversions.h>
/*************************************************************************************
*
*    CHECK PACKAGES ARRIVATI
*
**************************************************************************************/
//il pacchetto contenente la posizione delle features e quello i punti 3D devono 
//essere messi in corrispondenza temporale
//buffer_features_packet
//buffer_points_packet
void  check_syncronization_pkg()
{
	cout << "size features " << buffer_features_packet.size() << endl;
	cout << "size points" << buffer_points_packet.size() << endl;

	//primo controllo: se uno dei due vettori è vuoto ritorno
	if(buffer_points_packet.empty() || buffer_features_packet.empty())
	{
		//inizializzo t0_sec con il valore in secondi
		if (t0_sec == -1)
		{
			if(!buffer_points_packet.empty())
				t0_sec = buffer_points_packet.front().sec;
		    if(!buffer_features_packet.empty())
				t0_sec = buffer_features_packet.front().sec;
		}
		return;
	}
	
	//estraggo il tempo del primo pacchetto nel buffer delle features e della point cloud
	features_packet pkg_features = buffer_features_packet.front();
	points_packet pkg_points = buffer_points_packet.front();
	double f_time = (double)pkg_features.sec - t0_sec + (double)pkg_features.nsec/pow(10,9);
	double p_time = (double)pkg_points.sec - t0_sec + (double)pkg_points.nsec/pow(10,9);

	cout << "f time " << f_time << endl;
	cout << "p_time " << p_time << endl;

	double epsilon = 0.001;
	//case A: f_time = p_time
	if(abs(f_time - p_time) < epsilon)
	{
		//i due pacchetti corrispondono alla stessa immagine
		cout << "sincro" << endl;
		compute_3D_data(pkg_features,pkg_points);
		//butto via i pacchetti
		buffer_points_packet.pop();
		buffer_features_packet.pop();

	}
	else
	//case B: f_time > p_time
	{
		if( f_time > p_time)
		{
			cout << "case B" << endl;
			//il tempo a cui ho acquisito l'immagine è maggiore
			//scelgo di buttare via il pacchetto del point cloud
			buffer_points_packet.pop();
			//richiamo la funzione
			check_syncronization_pkg();
			return;
		}
		else
		//case C: f_time < p_time
		{
			cout << "case C" << endl;
			//il tempo a cui ho acquisito la point cloud è maggiore
			//scelgo di buttare via il pacchetto del immagine
			buffer_features_packet.pop();
			check_syncronization_pkg();
			return;
		}	
	}
}
/********************************************************************************
*
*    COMPUTE 3D DATA
*
**************************************************************************************/
void compute_3D_data(features_packet pkg_features, points_packet pkg_points)
{
	//controllo che il la point cloud sia valida
	if(!pkg_points.point_cloud.is_dense)
		return;

		
      pcl::PointCloud<pcl::PointXYZ> input_;
	  pcl::fromROSMsg(pkg_points.point_cloud, input_);
	  FILE *fd1 ;
	  fd1 = fopen("/home/ste/point.txt","w");
	  fprintf(fd1, "%s", "[");
	  for(int i = 0 ; i < input_.width * input_.height ; i++)
	  {
	  	fprintf(fd1, "%f", input_.points[i].x,"a");
	  	fprintf(fd1, "%s", ",");
	  	fprintf(fd1, "%f", input_.points[i].y,"a");
	  	fprintf(fd1, "%s", ",");
	  	fprintf(fd1, "%f", input_.points[i].z,"a");
	  	fprintf(fd1, "%s\n", ";");
	  }
	  fprintf(fd1, "%s", "]");
	  fclose(fd1);

	
	return;
}