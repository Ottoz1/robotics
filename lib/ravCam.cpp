#include "ravCam.hpp"

void process_frame(Mat frame, Scalar lower, Scalar upper, vector<Rect>& boxes, vector<int>& identity_out) {
    // Generate mask
    Mat hsv;
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    Mat mask;
    Mat number_mask;
    generate_mask(hsv, lower, upper, &mask, &number_mask);

    // Find contours based on mask
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    // Extract the interesting bounding boxes
    vector<Rect> interesting_boxes;
    vector<vector<Point>> interesting_conts;
    vector<int> identity;
    extract_interesting_areas(contours, hierarchy, frame, mask, interesting_boxes, interesting_conts, identity);

    // Identify the numbers
    identity = identify_numbers(interesting_boxes, mask, number_mask, identity, frame);

    // Return the results
    boxes = interesting_boxes;
    identity_out = identity;
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

    if (contours.size() < 2){   // We need at least a box and a number on it, two contours
        //printf("No contours found between the given HSV color bounds");
        return;
    }

    for (size_t i = 0; i < conts.size(); i++) {
        // Check if the contour has a parent
        if (hierarchy[i][3] != -1) {
            continue;
        }

        vector<Point>& cont = conts[i];
        Rect bb = boundingRect(cont);

        interesting_boxes.push_back(bb);
        interesting_conts.push_back(cont);

        int x = bb.x;
        int y = bb.y;
        int w = bb.width;
        int h = bb.height;

        // Check if the bounding rectangle is touching any of the edges of the frame
        if (x == 0 || y == 0 || x + w == frame.cols || y + h == frame.rows) {
            // Check if the bounding rectangle is touching the underside of the frame
            if (y + h == frame.rows) {
                identity.push_back(-4);  // -4 means taken
                continue;
            }
            else
            {
                identity.push_back(-3);  // -3 means cropped
                continue;
            }
        }

        // Check if the bounding rectangle is too small
        if (w < 40 || h < 40) {
            identity.push_back(-2);  // -2 means too far
            continue;
        }

        // Calculate the ratio of white pixels to black pixels
        double ratio = countNonZero(mask(Rect(x, y, w, h))) / (double)(w * h);
        if (ratio < 0.78) {
            identity.push_back(-1);  // -1 means unsure
            continue;
        }

        identity.push_back(-10);  // -10 means identify the number later
    }
}

void generate_mask(Mat hsv, Scalar lower, Scalar upper, Mat* mask, Mat* number_mask) {
    inRange(hsv, lower, upper, *mask);

    // Erode and dilate to remove accidental line detections
    Mat kernel3 = getStructuringElement(MORPH_RECT, Size(3, 3));
    erode(*mask, *mask, kernel3, Point(-1, -1), 2);
    dilate(*mask, *mask, kernel3, Point(-1, -1), 2);

    *number_mask = mask->clone();

    // Dilate and erode to blend the numbers together with the boxes
    Mat kernel32 = getStructuringElement(MORPH_RECT, Size(40, 40));
    dilate(*mask, *mask, kernel32, Point(-1, -1), 1);
    erode(*mask, *mask, kernel32, Point(-1, -1), 1);
}

vector<int> identify_numbers(const vector<Rect>& boxes, const Mat& mask, const Mat& number_mask, const vector<int>& identity, const Mat& frame) {
    vector<int> updated_identity = identity;

    // Check if identity contains any -10
    if (find(identity.begin(), identity.end(), -10) == identity.end()) {
        return updated_identity;
    }

    Mat masked; // The frame with the mask applied bitwise
    bitwise_and(frame, frame, masked, mask);

    // Invert the number mask
    Mat inverted_number_mask;
    bitwise_not(number_mask, inverted_number_mask);

    Mat masked2;
    bitwise_and(masked, masked, masked2, inverted_number_mask);

    masked = masked2;

    // Make all pixels that are not black white
    threshold(masked, masked, 1, 255, THRESH_BINARY);

    // Erode and dilate to remove accidental line detections
    Mat kernel3 = getStructuringElement(MORPH_RECT, Size(3, 3));
    erode(masked, masked, kernel3, Point(-1, -1), 2);
    dilate(masked, masked, kernel3, Point(-1, -1), 2);

    // Find contours
    Mat masked_gray;
    cvtColor(masked, masked_gray, COLOR_BGR2GRAY);
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(masked_gray, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    // Loop through all boxes
    for (size_t i = 0; i < boxes.size(); i++) {
        if (updated_identity[i] != -10) {   // Check if the box has already been identified
            continue;
        }

        // Find the biggest contour that is inside the box
        Rect expanded_box = boxes[i] + Size(-20, -20) + Size(40, 40); // Add a 10 pixel border to the box
        int biggest_cont_index = -1;
        double biggest_cont_area = 0.0;
        for (size_t j = 0; j < contours.size(); j++) {
            const vector<Point>& cont = contours[j];
            double area = contourArea(cont);
            if (area > biggest_cont_area && expanded_box.contains(cont[0])) {
                biggest_cont_index = j;
                biggest_cont_area = area;
            }
        }

        // Check if a contour was found
        if (biggest_cont_index == -1) {
            updated_identity[i] = -1;
            continue;
        }

        // Draw the contour
        drawContours(masked, contours, biggest_cont_index, Scalar(0, 255, 0), 3);

        // Get the biggest child contour of the biggest contour
        const vector<Point>& biggest_cont = contours[biggest_cont_index];
        int biggest_child_cont_index = -1;
        double biggest_child_cont_area = 0.0;
        for (size_t j = 0; j < contours.size(); j++) {
            const vector<Point>& cont = contours[j];
            double area = contourArea(cont);
            if (hierarchy[j][3] == biggest_cont_index && area > biggest_child_cont_area) {
                biggest_child_cont_index = j;
                biggest_child_cont_area = area;
            }
        }

        // Draw the child contour if it exists
        if (biggest_child_cont_index != -1 && biggest_child_cont_area > 100) {
            drawContours(masked, contours, biggest_child_cont_index, Scalar(0, 0, 255), 3);
            updated_identity[i] = 0;
        } else {
            updated_identity[i] = 1;
        }
    }

    return updated_identity;
}

Mat visualize_results(Mat frame, vector<Rect> boxes, vector<int> identity){
    Mat frame_copy = frame.clone();

    for(int i = 0; i < boxes.size(); i++){
        Rect box = boxes[i];
        int id = identity[i];
        rectangle(frame_copy, box, Scalar(0, 255, 0), 2);
        
        if(id == 0){
            putText(frame_copy, "0", Point(box.x, box.y-4), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
        }
        else if (id == 1){
            putText(frame_copy, "1", Point(box.x, box.y-4), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
        }
        else if (id == -1){
            putText(frame_copy, "-1 Unsure", Point(box.x, box.y-4), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
        }
        else if (id == -2){
            putText(frame_copy, "-2 Too far", Point(box.x, box.y-4), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
        }
        else if (id == -3){
            putText(frame_copy, "-3 Cropped", Point(box.x, box.y-4), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
        }
        else if(id == -4){
            putText(frame_copy, "-4 Taken", Point(box.x, box.y-4), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
        }
    }

    return frame_copy;
}

// This function finds the normalized distance between the center of the contour and the center of the image in the X direction
float find_d(Mat img, Rect box) {
    // Find the center of the box
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
