#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <stdio.h>

using namespace std;
using namespace cv;

Mat contour_color(Mat image, int color);
vector <Point> get_contour_center(vector<vector<Point> > contours);
int distance_to_center(Point center, Point centroid);

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

        Mat imgHSV = contour_color(img, 0);

    //----------------------------------------------------------------------------//

      //Showing the video//
      imshow("Video Player", imgHSV);
      //Allowing 25 milliseconds frame processing time and initiating break condition//
      char c = (char)waitKey(25);
      if (c == 27){ //If 'Esc' is entered break the loop//
         break;
      }
   }
   cap.release();//Releasing the buffer memory//
}

Mat contour_color(Mat image, int color){
    Mat imgHSV;
    cvtColor(image, imgHSV, COLOR_BGR2HSV);
    inRange(imgHSV, Scalar(0, 119, 114), Scalar(179, 175, 255), imgHSV);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(imgHSV, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

    vector<vector<Point> > contours_poly(contours.size());
    vector<Rect> boundRect(contours.size());

    if (contours.size() == 0){
        return image;
    }

    // Approximate contours to polygons + get bounding rects and circles
    for (int i = 0; i < contours.size(); i++){
        approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
        boundRect[i] = boundingRect(Mat(contours_poly[i]));
    }

    // Draw contours
    Mat drawing = image;
    for (int i = 0; i < contours.size(); i++){
        Scalar color = Scalar(255, 255, 255);
        drawContours(drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point());
    }

    // Draw number of contours
    putText(drawing, "Contours: " + to_string(contours.size()), Point(10, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0), 2);

    // Get the center of the contour
    vector <Point> centroid = get_contour_center(contours);
    for (int i = 0; i < centroid.size(); i++){
        circle(drawing, centroid[i], 5, Scalar(0, 0, 255), -1);
    }

    // Get the center of the image
    Point center = Point(drawing.cols / 2, drawing.rows / 2);
    circle(drawing, center, 5, Scalar(0, 255, 0), -1);

    // calculate the average point location
    Point avg;
    for (int i = 0; i < centroid.size(); i++){
        avg.x += centroid[i].x;
        avg.y += centroid[i].y;
    }
    avg.x /= centroid.size();
    avg.y /= centroid.size();

    // Get all distances between the center of the image and the center of the contours
    if(centroid.size() > 0){
        int distance = distance_to_center(center, avg);
        if(distance > 0 && distance < 5000){
            cout << "Distance: " << distance << endl;
        }

    }

    return drawing;
}

vector <Point> get_contour_center(vector<vector<Point> > contours){
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

