#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

int main() {
   // Sample data for odometry, scan-match, and Kalman calculations
   vector<double> odom_data {1.0, 2.0, 0.0, 0.0};
   vector<double> scanmatch_data {3.0, 4.0, 0.5};
   vector<double> kalman_data {5.0, 6.0, 0.8};

   // Open files for writing
   ofstream odom_file("odom.txt");
   ofstream scanmatch_file("scanmatch.txt");
   ofstream kalman_file("kalman.txt");

   // Write values to the files
   for (int i = 0; i < odom_data.size(); i++) {
       odom_file << odom_data[i] << " ";
   }
   odom_file << endl;

   for (int i = 0; i < scanmatch_data.size(); i++) {
       scanmatch_file << scanmatch_data[i] << " ";
   }
   scanmatch_file << endl;

   for (int i = 0; i < kalman_data.size(); i++) {
       kalman_file << kalman_data[i] << " ";
   }
   kalman_file << endl;

   // Close the files
   odom_file.close();
   scanmatch_file.close();
   kalman_file.close();

   cout << "Files written successfully." << endl;
   return 0;
}
