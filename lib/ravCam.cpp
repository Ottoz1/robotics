#include "ravCam.hpp"

void process_image(Mat image, Scalar lower, Scalar upper, int* predicted_number, vector<Point>* box_contour_ptr, vector<Point>* number_contour_ptr, vector<Point>* inner_number_contour_ptr, float* D)
{
    // Find the 3 most important contours
    vector<Point> box_contour;  // Biggest contour in the image (supposed to be the box)
    vector<Point> number_contour;   // Biggest contour in the image within largest_contour (suppose to be the number)
    vector<Point> inner_number_contour;    // Biggest contour in the image within number_contour (zero will have a large contour here, 1 will not)
    find_important_contours(lower, upper, image, &box_contour, &number_contour, &inner_number_contour);

    if (box_contour.empty() || number_contour.empty() || inner_number_contour.empty()) {
        //printf("No contours found between the given HSV color bounds");
        *D = 0;
        *predicted_number = -2;
        box_contour = vector<Point>();
        number_contour = vector<Point>();
        inner_number_contour = vector<Point>();
        box_contour.push_back(Point(0, 0));
        number_contour.push_back(Point(0, 0));
        inner_number_contour.push_back(Point(0, 0));
        *box_contour_ptr = box_contour;
        *number_contour_ptr = number_contour;
        *inner_number_contour_ptr = inner_number_contour;
        return;
    }

    // Predict the number based on the ratio of the inner number to the outer number areas
    int pred = predict_number(number_contour, inner_number_contour, box_contour);

    // Find the D value, a number between -1 and 1 in the X direction
    float d = find_d(image, box_contour);

    // Output the results
    *predicted_number = pred;
    *box_contour_ptr = box_contour;
    *number_contour_ptr = number_contour;
    *inner_number_contour_ptr = inner_number_contour;
    *D = d;
}

// This function runs the images through the filter and returns the filtered image
void find_important_contours(Scalar lower, Scalar upper, Mat image, vector<Point>* box_countour_ptr, vector<Point>* number_contour_ptr, vector<Point>* inner_number_contour_ptr) {
    Mat imgHSV;
    cvtColor(image, imgHSV, COLOR_BGR2HSV);
    inRange(imgHSV, lower, upper, imgHSV);

    // Get contours and only keep the one with the largest area
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(imgHSV, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

    if (contours.size() < 5){   // We need at least a box and a number on it, two contours
        //printf("No contours found between the given HSV color bounds");
        return;
    }

    // Find the largest contour (the box)
    int box_index = biggest_within(contours, vector<Point>());
    vector<Point> largest_contour = contours[box_index];

    // Find the second largest contour (the number) and make sure it's inside the box
    int second_largest_contour_index = biggest_within(contours, largest_contour);
    vector<Point> second_largest_contour = contours[second_largest_contour_index];

    // Find the third largest contour (the number) and make sure it's inside number (second largest contour)
    int third_largest_contour_index = biggest_within(contours, second_largest_contour);
    vector<Point> third_largest_contour = contours[third_largest_contour_index];

    *box_countour_ptr = largest_contour;
    *number_contour_ptr = second_largest_contour;
    *inner_number_contour_ptr = third_largest_contour;
}

// This function finds the biggest contour within another contour
// If "within" is empty, it will find the biggest contour in "contours" list
// This function is used by "find_important_contours" to find the box, nummer within the box, and the inner_number within the number
int biggest_within(vector<vector<Point> > contours, vector<Point> within)
{
    int largest_contour_index = 0;
    int largest_area = 0;
    for (int i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        if (area > largest_area) {
            if (within.empty()) {
                largest_area = area;
                largest_contour_index = i;
            }
            else {
                vector<Point> contour = contours[i];
                Point point_on_contour = contour[0];
                if (within == contour)  // Don't count the contour we're checking against
                    continue;
                if (pointPolygonTest(within, point_on_contour, false) > 0) {
                    largest_area = area;
                    largest_contour_index = i;
                }
            }
        }
    }
    return largest_contour_index;
}

int predict_number(vector<Point> number_contour, vector<Point> inner_number_contour, vector<Point> box_contour)
{
    float inner_area = (float)contourArea(inner_number_contour);    // if it's 1, this will be 0 or close to 0
    float number_area = (float)contourArea(number_contour);
    float box_area = (float)contourArea(box_contour);

    if(box_area == inner_area)
        return 1;

    float ratio = inner_area / number_area;
    if (ratio < 0.1)
        return 1;
    else return 0;
}

// This function finds the normalized distance between the center of the contour and the center of the image in the X direction
float find_d(Mat img, vector<Point> contour) {
    // Find the center of the contour and the center of the image
    Rect box = boundingRect(contour);
    Point cont_center = Point(box.x + box.width / 2, box.y + box.height / 2);
    Point img_center = Point(img.cols / 2, img.rows / 2);

    // We only care about the X direction
    float x_img = (float)img_center.x;
    float x_cont = (float)cont_center.x;

    // If the center of the image was (0, 0), then relative x = x_cont - x_img
    float relative_x = x_cont - x_img;

    // Normalize the distance so that the distance is in the range [-1, 1]
    float d = relative_x / x_img;

    return d;
}

Mat visualize_results(Mat image, vector<Point> largest_contour, vector<Point> number_contour, vector<Point> third_contour, int predicted_number)
{
    Mat img = image.clone();    // Make a copy of the image so we don't modify the original

    // Draw the contours on the image
    drawContours(img, vector<vector<Point> >(1, largest_contour), 0, Scalar(0, 255, 0), 2);
    drawContours(img, vector<vector<Point> >(1, number_contour), 0, Scalar(255, 0, 0), 2);
    drawContours(img, vector<vector<Point> >(1, third_contour), 0, Scalar(0, 0, 255), 2);

    // Draw predicted number on the image above the box
    char predicted = (char)(predicted_number + 48);
    Rect box = boundingRect(largest_contour);
    Point above_box = Point(box.x + box.width / 2, box.y - 10);
    putText(img, string(1, predicted), above_box, FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0), 2);

    // Draw the center of the contour and the center of the image as filled circles
    Point contcent = Point(box.x + box.width / 2, box.y + box.height / 2);
    Point imgcent = Point(img.cols / 2, img.rows / 2);
    circle(img, contcent, 5, Scalar(0, 0, 0), FILLED);
    circle(img, imgcent, 5, Scalar(0, 255, 0), FILLED);

    return img;
}