#include "vision_node.h"

/********************************************************************************
*
*    FINESTRE DI CONTROLLO
*
**************************************************************************************/
void make_control_window(  )
{
  /**********************Creo Una Finestra  DI Controllo***********************/
  namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
  //Create trackbars in "Control" window
  cvCreateTrackbar("Image num", "Control", &idx, 3000);
  cvCreateTrackbar("Test num", "Control", &id_test, 3);
  /*****************************************************************************/
  namedWindow("Test Color", CV_WINDOW_AUTOSIZE);
  cvCreateTrackbar("LowH", "Test Color", &iLowH, 179); //Hue (0 - 179) valore *2 = angolo
  cvCreateTrackbar("HighH", "Test Color", &iHighH, 179);
  cvCreateTrackbar("LowS", "Test Color", &iLowS, 255); //Saturation (0 - 255)
  cvCreateTrackbar("HighS", "Test Color", &iHighS, 255);
  cvCreateTrackbar("LowV", "Test Color", &iLowV, 255); //Value (0 - 255)
  cvCreateTrackbar("HighV", "Test Color", &iHighV, 255);
  cvCreateTrackbar("Clos Erod", "Test Color", &size_erode_clos, 255); //Value (0 - 255)
  cvCreateTrackbar("Clos Dil", "Test Color", &size_dil_clos, 255);
  cvCreateTrackbar("Fill Erod", "Test Color", &size_erode_fill, 255); //Value (0 - 255)
  cvCreateTrackbar("Fill Dil", "Test Color", &size_dil_fill , 255);

  /**********************Creo Una Finestra  DI Controllo***********************/
  /*namedWindow("Blob", CV_WINDOW_AUTOSIZE); //create a window called "Blob"
  cvCreateTrackbar("Min Tresh", "Blob", &min_threshold_blob , 255);
  cvCreateTrackbar("Max Tresh", "Blob", &max_threshold_blob , 255);
  cvCreateTrackbar("Blob Threshold Step", "Blob", &thresholdStep_blob , 50);
  cvCreateTrackbar("Blob min Area", "Blob", &min_area_blob  ,10000);
  cvCreateTrackbar("Blob max Area", "Blob", &max_area_blob  ,1000000);
  //cvCreateTrackbar("Blob Color", "Blob", &blob_color  , 255);
  cvCreateTrackbar("Min Dist", "Blob", &min_dist_blob , 500);*/
  /**********************Creo Una Finestra  DI Controllo***********************/
  //namedWindow("Bin e Contour", CV_WINDOW_AUTOSIZE); //create a window called "Bin e Contour"
  //cvCreateTrackbar("Canny Tresh", "Bin e Contour", &thresh  , max_thresh);
  //cvCreateTrackbar("Bin Image", "Bin e Contour", &bin_threshold , 255);
  //cvCreateTrackbar("Bin Type", "Bin e Contour", &bin_type;  , 4);


}

/********************************************************************************
*
*    BLOB
*
**************************************************************************************/
/*void find_and_extract_blob(Mat src_image)
{

  // Set up the detector with parameters.
  // Setup SimpleBlobDetector parameters.
  SimpleBlobDetector::Params params;
  // Change thresholds
  params.minThreshold = min_threshold_blob;
  params.maxThreshold = max_threshold_blob;
  params.thresholdStep = thresholdStep_blob+1;

  //min dist blob
  params.minDistBetweenBlobs = min_dist_blob;

  // Filter by Area.
  params.filterByArea = true;
  params.minArea = min_area_blob+1;
  params.maxArea = max_area_blob + 1;
  
  //Colorros::Publisher info_pub
  params.filterByColor = false;
  params.blobColor = blob_color;

  //other parameter
  params.filterByInertia = false;
  params.filterByConvexity = false;
  params.filterByCircularity = false;

  //detect blob 
  std::vector<KeyPoint> keypoints;
  
  SimpleBlobDetector detector(params);
  detector.detect( src_image, keypoints);

  //Ptr<SimpleBlobDetector> sbd = SimpleBlobDetector::create(params);
  //sbd->detect(src_image, keypoints, Mat());

  
  
    Mat im_with_keypoints;
    for(std::vector<cv::KeyPoint>::iterator blobIterator = keypoints.begin(); blobIterator != keypoints.end(); blobIterator++)
    {
      std::cout << "size of blob is: " << blobIterator->size << std::endl;
      std::cout << "point is at: " << blobIterator->pt.x << " " << blobIterator->pt.y << std::endl;
      
      if(!real_time)
      {
        stringstream ssx;
        stringstream ssy;
        ssx << blobIterator->pt.x;
        ssy << blobIterator->pt.y;

        string text = "(" + ssx.str() + "," + ssy.str() + ")";
        //center the text
        Point textOrg(blobIterator->pt.x,blobIterator->pt.y);
        // then put the text itself
        putText(src_image, text, textOrg, FONT_HERSHEY_SCRIPT_SIMPLEX, 0.1, Scalar(120,0,255), 1, 8);
      }
     }
    
    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
    if(!real_time)
    {
      drawKeypoints( src_image, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
      // Show blobs
      imshow("blobs", im_with_keypoints );
      waitKey(30);
    }
    //sensor_msgs::CameraInfo info_image;
    //info_image.height = im_with_keypoints.size().height;
    //info_image.width = im_with_keypoints.size().width;
    //sensor_msgs::ImagePtr msg_image = cv_bridge::CvImage(std_msgs::Header(), "mono8", im_with_keypoints).toImageMsg();
    //pub.publish(msg_image);
    //info_pub.publish(info_image);



  

  
}
*/
/********************************************************************************
*
*    MORPHOLOGICAL FILTER
*
**************************************************************************************/
void morphological_filter(Mat& image)
{
  //morphological opening (remove small objects from the foreground)
  erode(image, image, getStructuringElement(MORPH_ELLIPSE, Size(size_erode_clos+1, size_erode_clos+1)) );
  dilate( image, image, getStructuringElement(MORPH_ELLIPSE, Size(size_dil_clos+1, size_dil_clos+1)) ); 

  //morphological closing (fill small holes in the foreground)
  dilate(image, image, getStructuringElement(MORPH_ELLIPSE, Size(size_erode_fill+1, size_erode_fill+1)) ); 
  erode(image, image, getStructuringElement(MORPH_ELLIPSE, Size(size_dil_fill+1, size_dil_fill+1)) );

}
/********************************************************************************
*
*    FILTER COLOR
*
**************************************************************************************/
Mat color_filter_image(Mat hsv_image, string color )
{ // http://www.rapidtables.com/convert/color/rgb-to-hsv.htm
  Mat imgThresholded;
  if (color == "red")
  { 
    Mat upper_red, lower_red;
    inRange(hsv_image, Scalar(0, iLowS, iLowV), Scalar(10, iHighS, iHighV), lower_red);
    inRange(hsv_image, Scalar(165, iLowS, iLowV), Scalar(179, iHighS, iHighV), upper_red);  
    addWeighted(lower_red, 1.0, upper_red, 1.0, 0.0, imgThresholded);
  }
  if (color == "blue")
  { 

    inRange(hsv_image, Scalar(100, iLowS, iLowV), Scalar(120, iHighS, iHighV), imgThresholded);   

  }
  
  //GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);
  return imgThresholded;

}
/********************************************************************************
*
*    Binarizazione
*
**************************************************************************************/
Mat Binarize_Image( Mat src_gray)
{
  /* 0: Binary
     1: Binary Inverted
     2: Threshold Truncated
     3: Threshold to Zero
     4: Threshold to Zero Inverted
   */
  Mat dst;
  threshold( src_gray, dst, bin_threshold, max_BINARY_value,bin_type);
  return dst;
}
/********************************************************************************
*
*    CALCOLA BARICENTRO
*
**************************************************************************************/
void calcola_baricentro(Point2f* rect_points, double* cx, double* cy)
{
  //calcolo baricentgro dall'intersezione delle diagonali
  //double m1 = (rect_points[0].y - rect_points[2].y)/(rect_points[0].x - rect_points[2].x);
  //double m2 = (rect_points[1].y - rect_points[3].y)/(rect_points[1].x - rect_points[3].x);
  //double A1 = m1-m2;
  //double A2 = rect_points[1].y-rect_points[0].y + m1 * rect_points[0].x - m2 * rect_points[1].x;
  //coordinata del baricentro
  //*cx = A2/A1;
  //*cy = m1 * (*cx - rect_points[0].x) + rect_points[0].y;

  *cx = (rect_points[0].x+rect_points[1].x+rect_points[2].x+rect_points[3].x)/4;
  *cy = (rect_points[0].y+rect_points[1].y+rect_points[2].y+rect_points[3].y)/4;

  return;


}
/********************************************************************************
*
*    SORT POINT
*
**************************************************************************************/
void sort_point(Point2f* rect_points)
{

  
  //ordino i puti secondo l'ascissa, da minore a maggiore
  sort(rect_points,rect_points+4,comp);

  //order coordinates clockwise
  //now, sort the left-most coordinates according to their
  //y-coordinates so we can grab the top-left and bottom-left
  //points, respectively
  Point2f bottom_left, top_left;
  if(rect_points[0].y > rect_points[1].y)
  {
    Point2f app;
    app.x = rect_points[0].x;
    app.y = rect_points[0].y;
    rect_points[0].x = rect_points[1].x;
    rect_points[0].y = rect_points[1].y;
    rect_points[1].x = app.x;
    rect_points[1].y = app.y;

  }
  else
  {
    //..già ordinati
  }

  //adesso ho il primo punto bottom_left, il secondo top_left
  //adesso calcolo la distanza dal bottom left agli altri due punti a destra.
  //qeullo con distanza maggiore sarà il top_right
  double dist_1 = (rect_points[0].x - rect_points[2].x )*(rect_points[0].x - rect_points[2].x ) + (rect_points[0].y - rect_points[2].y )*(rect_points[0].y - rect_points[2].y ) ;
  double dist_2 = (rect_points[0].x - rect_points[3].x )*(rect_points[0].x - rect_points[3].x ) + (rect_points[0].y - rect_points[3].y )*(rect_points[0].y - rect_points[3].y ) ;
  if (dist_1 < dist_2)
  {
    //scambio di posizione il 3 con il 4
    Point2f app;
    app.x = rect_points[2].x;
    app.y = rect_points[2].y;
    rect_points[2].x = rect_points[3].x;
    rect_points[2].y = rect_points[3].y;
    rect_points[3].x = app.x;
    rect_points[3].y = app.y;
  }
  else
  {
    //sono già tutti ordinati
  }

}
/********************************************************************************
*
*    CALCOLA RETTANGOLI ESTERNI
*
**************************************************************************************/
void find_object(vector<vector<Point> > contours, vector<Point2f>*  boxObj)
{


  cout << contours.size() << endl;
  vector<RotatedRect> minRect( contours.size() );
  Point2f rect_points_i[4]; 
  Point2f rect_points_j[4];
  
  
  bool inside = false;
  for( int i = 0; i< contours.size(); i++ )
  {
      //considero il contorno i-esimo e ne calcolo il rettangolo minimo che lo contiene
      RotatedRect minRect_i = minAreaRect( Mat(contours[i]) );
      //prendo i 4 punti
      minRect_i.points( rect_points_i );
      //li ordino in senso orario
      sort_point(rect_points_i);
      
      //calcolo l'area
      double l = sqrt((rect_points_i[0].x- rect_points_i[1].x)*(rect_points_i[0].x- rect_points_i[1].x)+(rect_points_i[0].y- rect_points_i[1].y)*(rect_points_i[0].y- rect_points_i[1].y));
      double h = sqrt((rect_points_i[1].x- rect_points_i[2].x)*(rect_points_i[1].x- rect_points_i[2].x)+(rect_points_i[1].y- rect_points_i[2].y)*(rect_points_i[1].y- rect_points_i[2].y));
      double area_i = l*h;
      
      //calcolo intersezione delle diagonali per calcolare il baricentro
      double cx_i;
      double cy_i;
      calcola_baricentro(rect_points_i, &cx_i, &cy_i);

      //primo controllo sull'area che deve essre maggiore di area_min_contour
      if(area_i >= area_min_contour)
      {
        //devo vedere se questo rettangolo è contenuto in altri rettangoli
        inside = false;
        for(int j = 0 ; j< contours.size(); j++ )
        {
          
          if(j != i)
          {
            //prendo il contorno j-esimo
            RotatedRect minRect_j;
            //calcolo il rettangolo di area minimia che lo contiene
            minRect_j = minAreaRect( Mat(contours[j]) );
            minRect_j.points( rect_points_j );
            //ordino i punti dal bottom_left in senso orario
            sort_point(rect_points_j);
          
            //controllo che il centroide non sia interno a questo rettangolo
            //(Bx-Px)(Ay-Py)-(By-Py)(Ax-Px) 
            //Segno > 0 significa che P è a destra di A->B
            //Segno < 0 significa che P è a sinistra di A->B
            //Segno = 0 significa che P appartiene ad A->B
            double l1 = (rect_points_j[1].x-cx_i)*(rect_points_j[0].y-cy_i) - (rect_points_j[1].y-cy_i)*(rect_points_j[0].x-cx_i);
            double l2 = (rect_points_j[2].x-cx_i)*(rect_points_j[1].y-cy_i) - (rect_points_j[2].y-cy_i)*(rect_points_j[1].x-cx_i);
            double l3 = (rect_points_j[3].x-cx_i)*(rect_points_j[2].y-cy_i) - (rect_points_j[3].y-cy_i)*(rect_points_j[2].x-cx_i);
            double l4 = (rect_points_j[0].x-cx_i)*(rect_points_j[3].y-cy_i) - (rect_points_j[0].y-cy_i)*(rect_points_j[3].x-cx_i);
            
            
            if(l1 > 0 && l2 > 0 && l3 > 0 && l4 > 0)
            {

              //calolo l'area del rettangolo j-esimo
              double l = sqrt((rect_points_j[0].x- rect_points_j[1].x)*(rect_points_j[0].x- rect_points_j[1].x)+(rect_points_j[0].y- rect_points_j[1].y)*(rect_points_j[0].y- rect_points_j[1].y));
              double h = sqrt((rect_points_j[1].x- rect_points_j[2].x)*(rect_points_j[1].x- rect_points_j[2].x)+(rect_points_j[1].y- rect_points_j[2].y)*(rect_points_j[1].y- rect_points_j[2].y));
              double area_j = l*h;
              
              if(area_i >= area_j)
              {
                //se area_i è maggiore di area_j vado avanti nell'analisi, 
                //non so se esistono altri j che potrebbero contenere i
                cout << "non contenuto" << endl;
                inside = false;
              }
              else
              {
                //se area_i è minore di area_j, sicuramente lo posso scartare
                cout << "contenuto" << endl;
                inside = true;
                break;
              }
            }
          }
        }
        
        //ho finito l'analisi per i. Se inside = false, allora è il piu grande e lo devo prendere
        if(inside == false)
        {
         
          (*boxObj).push_back(rect_points_i[0]);
          (*boxObj).push_back(rect_points_i[1]);
          (*boxObj).push_back(rect_points_i[2]);
          (*boxObj).push_back(rect_points_i[3]);
          
          
        }
      }
      else
      {
        //..area troppo piccola
      } 
    }

}

