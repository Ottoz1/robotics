#include "lib/vision.hpp"

int main()
{
    // Load the image
    Mat image = imread("../img/1.jpg");
    resize(image, image, Size(1280, 720));

    // Set the HSV color bounds for the filter
    Scalar lower = Scalar(41, 50, 0);
    Scalar upper = Scalar(115, 255, 240);

    maien(image, lower, upper);
}