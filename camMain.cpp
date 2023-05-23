#include "lib/ravCam.hpp"
#include <chrono>

using namespace std;
using namespace cv;

int main() {
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        // Check if video file opened successfully
        cout << "Error opening video file" << std::endl;
        return -1;
    }

    // Define the color range
    Scalar lower = Scalar(88, 73, 0);
    Scalar upper = Scalar(107, 255, 255);

    // Define obstacle color range
    Scalar lower_purple = Scalar(132, 72, 0);
    Scalar upper_purple = Scalar(161, 255, 255);
    Scalar lower_yellow = Scalar(20, 93, 0);
    Scalar upper_yellow = Scalar(30, 255, 255);
    Scalar lower_darkBlue = Scalar(108, 118, 0);
    Scalar upper_darkBlue = Scalar(116, 230, 255);

    Mat image;
    while (true) {
        bool ret = cap.read(image);
        if (!ret) {
            // Break the loop if no image is captured
            break;
        }
        rotate(image, image, ROTATE_180);   // Rotate the image 180 degrees

        // Process the image
        vector<Rect> boxes;
        vector<int> identity;

        // Check execution time
        chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
        process_frame(image, lower, upper, boxes, identity, lower_purple, upper_purple, lower_yellow, upper_yellow, lower_darkBlue, upper_darkBlue);
        chrono::high_resolution_clock::time_point end = chrono::high_resolution_clock::now();
        cout << "Time process frame in ms: " << chrono::duration_cast<chrono::duration<double>>(end - start).count() * 1000 << endl;

        // Visualize the results
        Mat result = visualize_results(image, boxes, identity);

        // Display result image
        imshow("Result", result);
        imshow("Mask", numMask);

        // Press 'q' on the keyboard to exit
        if (waitKey(25) & 0xFF == 'q') {
            break;
        }
    }

    // Release everything if the job is finished
    cap.release();
    destroyAllWindows();
    cout << "Done" << endl;

    return 0;
}