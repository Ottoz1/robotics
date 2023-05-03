#include "lib/matplotlibcpp.h"
#include "speedProfile.hpp"

using namespace std;

int main() {

    speedProfile sp(70, 0.01, 10);

    sp.run();


    std::vector<float> timeStamps;
    std::vector<float> speeds = sp.getSpeeds();
    std::vector<float> positions = sp.getPositions();

    for (int i = 0; i < speeds.size(); i++){
        timeStamps.push_back(i * 0.01);
    }


    cout << "speeds: " << endl;
    for (int i = 0; i < speeds.size(); i++){
        cout << speeds[i] << endl;
    }

    matplotlibcpp::plot(timeStamps, speeds);
    matplotlibcpp::plot(timeStamps, positions);
    matplotlibcpp::title("Speed and Position over time");
    matplotlibcpp::xlabel("Time (s)");
    matplotlibcpp::ylabel("Speed (m/s)(blue) Position (m)(orange)");

    matplotlibcpp::show();
}