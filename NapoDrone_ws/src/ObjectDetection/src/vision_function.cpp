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
/************************************************************************************
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

/************************************************************************************
*
*   PIANO IMMAGINE VIRTUALE: ROLL
*
**************************************************************************************/
void virtual_image_plane_roll(double phi, double u1, double v1, double fv, double fu, double u0, double v0, double* u, double* v)
{
  //dato un angolo di rollio (phi)
  //coordinate sul piano immagine [u1,v1]
  //resituisce coordinate [u,v] sul piano immagine virtuale

  //coordinata u
  double num = fv*(u1-u0);
  double den = fu*fv*cos(phi) + fu*(v1-v0)*sin(phi);
 
  *u = num/den;
  //coordinata v

  num = -fv*sin(phi) + (v1-v0)*cos(phi);
  den = fv*cos(phi) + (v1-v0)*sin(phi);
 
  *v = num/den;

  return;
}
  




/************************************************************************************
*
*   PIANO IMMAGINE VIRTUALE: PITCH
*
**************************************************************************************/
void virtual_image_plane_pitch(double theta, double u1, double v1, double fv, double fu, double u0, double v0, double* u, double* v)
{
  //dato un angolo di pitch (theta)
  //coordinate sul piano immagine [u1,v1]
  //resituisce coordinate [u,v] sul piano immagine virtuale

  //coordinata u
  double num = fu*sin(theta) + (u1-u0)*cos(theta);
  double den = (u1-u0)*sin(theta) + fu*cos(theta);
 
  *u = num/den;
  //coordinata v

  num = fu*(v1-v0);
  den = fu*fv*cos(theta) + fv*(u1-u0)*sin(theta);
 
  *v = num/den;

  return;
}

/********************************************************************************
*
*    Elabora Immagine
*
**************************************************************************************/
void obj_detection(Mat bgr_image)
{
  
  double tempo;
  start=clock();
  //prealloco tutte le immagini
  Mat bgr_image_rs = Mat::zeros( cv::Size(320,180), CV_8UC3 );
  Mat hsv_image = Mat::zeros( cv::Size(320,180), CV_8UC3 );
  Mat imgThresholded = Mat::zeros( cv::Size(320,180), CV_8UC3 );
  Mat img_red = Mat::zeros( cv::Size(320,180), CV_8UC3 );
  Mat img_edges = Mat::zeros( cv::Size(320,180), CV_8UC3 );
  //salvo l'immagine se richiesto
  if(params.save_img)
  {
    stringstream s_idx;
    s_idx << id_img;
    string str_path  = s_idx.str();

    if(id_img < 10)
      str_path  = params.img_path_save + "/left000"+ str_path + ".jpg";
    if(id_img>=10 && id_img < 100)
      str_path  = params.img_path_save +  "/left00"+ str_path + ".jpg";
    if(id_img>=100 && id_img < 1000)
      str_path  = params.img_path_save + "/left0"+ str_path + ".jpg";
    if(id_img>=1000 && id_img < 10000)
      str_path  = params.img_path_save +  "/left"+ str_path + ".jpg";

    //salvo su disco l'immagine

    imwrite( str_path, bgr_image );
        
    //currentTime = clock();
    //cout << double(currentTime)/CLOCKS_PER_SEC << endl;
    //salvo i tempi a cui ho acquisito la nuova immagine
    //fd1=fopen("/home/odroid/time.txt", "a");
    //fprintf(fd1, "%i", id_img);
    //fprintf(fd1, "%s", "  ");
    //fprintf(fd1, "%f\n",double(currentTime)/CLOCKS_PER_SEC) ;
    //fclose(fd1);

    id_img++;
  }
  //calcolo il tempo che ci metto per elaborare l'immagine
  start_cv=clock();

  



  //1B-COPRO  LA PART CHE NON MI INTERESSA///////////////////////////////////////////////////////////////////
  //questo serve per eliminare la pinza dall'immagine
  //cout <<bin_image.size().width << endl << bin_image.size().height<< endl;
  /*if(bgr_image.size().width == 960 && bgr_image.size().height == 540)
  {
    //levo immagine pinza più in alto
    Rect rec1;
    rec1.x = 640;
    rec1.y = 0;
    rec1.height = 100;
    rec1.width = bgr_image.size().width - rec1.x;
    rectangle(bgr_image, rec1, Scalar(0,0,0), CV_FILLED, 8, 0 );
    //pinza piu bassa
    Rect rec2;
    rec2.x = 640;
    rec2.y = 440;
    rec2.height = bgr_image.size().height- rec2.y;
    rec2.width = bgr_image.size().width- rec2.x;
    rectangle(bgr_image, rec2, Scalar(0,0,0), CV_FILLED, 8, 0 );
    //cebtro pinza
    Rect rec3;
    rec3.x = 845;
    rec3.y = rec1.height;
    rec3.height = 340;
    rec3.width = bgr_image.size().width- rec3.x;
    rectangle(bgr_image, rec3, Scalar(0,0,0), CV_FILLED, 8, 0 );
  }
  
  if(!params.real_time && false)
  {
    namedWindow("Rect", CV_WINDOW_AUTOSIZE); 
    cvCreateTrackbar("x", "Rect", &x, bgr_image.size().width);
    cvCreateTrackbar("y", "Rect", &y, bgr_image.size().height);
    cvCreateTrackbar("h", "Rect", &h, bgr_image.size().width); 
    cvCreateTrackbar("w", "Rect", &w, bgr_image.size().height); 
    Rect rec;
    rec.x = x;
    rec.y = y;
    rec.height = h;
    rec.width = w;
    rectangle(bgr_image, rec, Scalar(120,0,255), CV_FILLED, 8, 0 );
    imshow( "copro pinza", bgr_image );
    waitKey(30);
  }*/

  //1-C RESIZE IMMAGINE
  Size size(320,180);//the dst image size,e.g.100x100
  resize(bgr_image,bgr_image_rs,size);//resize image
  bgr_image = bgr_image_rs;
    


  //2- ottengo l'immagine in bianco e nero e l'iimagine in HSV e filtro per i vari colori//////////////////
  Mat gray_image, gray_image_b;
  cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);
  cv::cvtColor(bgr_image, gray_image, cv::COLOR_BGR2GRAY);
  //Mat gray_image;
  //cv::cvtColor(bgr_image, gray_image, cv::COLOR_BGR2GRAY);
    
  //gaussian Blur
  //GaussianBlur(hsv_image, hsv_image,cv::Size(7,7),0 );


  //2A- filtro rosso
  img_red = color_filter_image(hsv_image, "red");
  //2B- filtro blu
  //Mat img_blue = color_filter_image(hsv_image, "blue");
  //.. altri filtri per colori


  //...combinazione di piu filtri..
  //addWeighted(img_red, 1.0, img_blue, 1.0, 0.0, imgThresholded);
  imgThresholded = img_red;

  if(!params.real_time)
  {
    imshow( "imgThresholded", imgThresholded );
    waitKey(30);
  }



  //3-Remove small object and fill small holes ////////////////////////////////////////////////////////////
  morphological_filter(imgThresholded);
  if(!params.real_time && false)
  {
    imshow( "imgThresholded_foreground", imgThresholded );
    waitKey(30);
  }

  //4-Canny///////////////////////////////////////////////////////////////////////////////////////////////////
  Canny(imgThresholded, img_edges, 50,100);
  if(!params.real_time )
  {
    imshow( "canny", img_edges );
    waitKey(30);
  }

  //5--Find contours//////////////////////////////////////////////////////////////////////////////////////////
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  findContours( img_edges, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0, 0) );
    
  //6-Find object///////////////////////////////////////////////////////////////////////////////////////////////
  vector<Point2f>  boxObj;
  find_object(contours, &boxObj);
  int num_object = boxObj.size() / 4 ;
  cout<< "oggetti num : " <<  boxObj.size() / 4 << endl;
  if(!params.real_time )
  {
    RNG rng(12345);
    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    for(int i = 0 ; i < boxObj.size() / 4  ; i ++)
    {
      for( int j = 0; j < 4; j++ )
      {
        
        line( bgr_image_rs, boxObj[j+i*4], boxObj[(j+1)%4 + i*4], color, 2, 8 );
      }

    }    

    imshow( "drawing", bgr_image_rs );
    waitKey(30);
  }
  
  if(num_object > 0)
  {


    //pubblico le coordinate dell'ogetto su topic
    std_msgs::Float64MultiArray rect;
    //inserisco il primo punto (in realtà poi dovra essere messo un controllo che sia sempre lo stesso oggetto)
    rect.data.push_back(boxObj[0].x);
    rect.data.push_back(boxObj[0].y);
    rect.data.push_back(boxObj[1].x);
    rect.data.push_back(boxObj[1].y);
    rect.data.push_back(boxObj[2].x);
    rect.data.push_back(boxObj[2].y);
    rect.data.push_back(boxObj[3].x);
    rect.data.push_back(boxObj[3].y);
    //layout
    rect.layout.dim.push_back(std_msgs::MultiArrayDimension());
    rect.layout.dim.push_back(std_msgs::MultiArrayDimension());
    rect.layout.dim[0].label = "righe";
    rect.layout.dim[0].size = 4;
    //rect.layout.dim[0].stride = 1;
    rect.layout.dim[1].label = "colonne";
    rect.layout.dim[1].size = 2;
    //pubblico il messaggio su topic
    features_rect_pub.publish(rect);


  }
  
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  end=clock();
  tempo=((double)(end-start))/CLOCKS_PER_SEC;
  cout << "FREQ: " << 1/tempo << endl;

  tempo=((double)(end-start_cv))/CLOCKS_PER_SEC;
  cout << "FREQ COMPUTER VISION: " << tempo << "meno di: " << 0.05 << endl;

}