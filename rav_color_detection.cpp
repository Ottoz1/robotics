#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <stdio.h>

using namespace std;
using namespace cv;

vector<vector<Point>> contour_color(Mat image, int color);
vector <Point> get_contour_center(vector<vector<Point> > contours);
int distance_to_center(Point center, Point centroid);
vector<vector<Point>> contour_polygons(vector<vector<Point>> contours);
vector<vector<Point>> filter_contours(vector<vector<Point>> contours);
void contour_distance_to_image_center(vector<vector<Point>> contours, vector<Point> centroid, Point center);

int SCALER = 2;

int main() {
   Mat img;//Declaring a matrix to load the frames//

   //Declaring the video to show the video//
   namedWindow("Video Player");

   //Declaring an object to capture stream of frames from default camera//
   VideoCapture cap(0);

   //prompt an error message if no video stream is found//
   if (!cap.isOpened()){ 
      cout << "No video stream detected" << endl;
      system("pause");
      return-1;
   }

   //Taking an everlasting loop to show the video//
   while (true){ 
      cap >> img;

      //Breaking the loop if no video frame is detected//
      if (img.empty()){ 
         break;
      }
    //---------------------------Image Processing---------------------------------//

        vector<vector<Point>> contours;
        vector<vector<Point>> contours_poly;
        vector<Point> centroid;
        Point img_center;
        Mat img_scaled;
        Mat drawing = img;

        resize(img, img_scaled,  Size(img.cols / SCALER, img.rows / SCALER), 0, 0, INTER_LINEAR);

        contours = contour_color(img_scaled, 0);
        img_center = Point(img.cols / 2, img.rows / 2);

        // Filter contours
        contours = filter_contours(contours);
        contours_poly = contour_polygons(contours);
        
        for (int i = 0; i < contours.size(); i++){

            // Draw contours
            Scalar color = Scalar(255, 255, 255);
            drawContours(drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point());
            // print contours distance to center
        }

        // Draw number of contours
        putText(drawing, "Contours: " + to_string(contours.size()), Point(10, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0), 2);

        // Draw the center of the contours
        centroid = get_contour_center(contours);
        for (int i = 0; i < centroid.size(); i++){
            circle(drawing, centroid[i], 5, Scalar(0, 0, 255), -1);
        }

        // Draw the center of the image
        Point center = Point(drawing.cols / 2, drawing.rows / 2);
        circle(drawing, center, 5, Scalar(0, 255, 0), -1);
        
        contour_distance_to_image_center(contours, centroid, img_center);

    //----------------------------------------------------------------------------//

      //Showing the video//
      imshow("Video Player", drawing);
      //Allowing 25 milliseconds frame processing time and initiating break condition//
      char c = (char)waitKey(25);
      if (c == 27){ //If 'Esc' is entered break the loop//
         break;
      }
   }
   cap.release();//Releasing the buffer memory//
}

vector<vector<Point>> contour_polygons(vector<vector<Point>> contours){
    vector<vector<Point>> contours_poly(contours.size());

    // Approximate contours to polygons + get bounding rects and circles
    for (int i = 0; i < contours.size(); i++){
        approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
    }

    return contours_poly;
}

vector<vector<Point>> contour_color(Mat image, int color){
    Mat imgHSV;
    cvtColor(image, imgHSV, COLOR_BGR2HSV);
    inRange(imgHSV, Scalar(99, 130, 177), Scalar(117, 255, 255), imgHSV);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(imgHSV, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

    if (contours.size() == 0){
    }

    // Scale contours back to original size
    for (int i = 0; i < contours.size(); i++){
        for (int j = 0; j < contours[i].size(); j++){
            contours[i][j].x *= SCALER;
            contours[i][j].y *= SCALER;
        }
    }
    return contours;
}

vector<vector<Point>> filter_contours(vector<vector<Point>> contours){

    vector<vector<Point>> contours_new;

    for (int i = 0; i < contours.size(); i++){
        int area = contourArea(contours[i]);
        if (area > 100){
            contours_new.push_back(contours[i]);
        }
    }
    return contours_new;
}

void contour_distance_to_image_center(vector<vector<Point>> contours, vector<Point> centroid, Point center){

    for (int i = 0; i < contours.size(); i++){
        int distance = distance_to_center(center, centroid[i]);
        if(distance > 0 && distance < 5000){
            cout << "Distance: " << distance << endl;
        }
    }
    return;
}

vector<Point> get_contour_center(vector<vector<Point>> contours){
    vector <Point> centers;
    for (int i = 0; i < contours.size(); i++){
        Moments M = moments(contours[i]);
        centers.push_back(Point(M.m10 / M.m00, M.m01 / M.m00));
    }
    return centers;
}

int distance_to_center(Point center, Point centroid){
    int distance = sqrt(pow(center.x - centroid.x, 2) + pow(center.y - centroid.y, 2));
    return distance;
}

