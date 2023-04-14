#include "cox.hpp"

void cox_linefit(MatrixXf points, MatrixXf line_segments)
{
    char* title = "Example_points_with_lines";
    plot(points, line_segments, title);
}

void plot(MatrixXf points, MatrixXf lines, char* title)
{
    // Assign value to title if it is empty
    if (title == NULL) {
        title = "NO_TITLE_PROVIDED";
    }

    // Convert the MatrixXf to arrays
    float* x = points.col(0).data();
    float* y = points.col(1).data();
    int n = points.rows();

    // Construct the command to run the Python script
    std::string cmd = "python3 ../lib/plotter.py ";
    for (int i = 0; i < n; i++) {
        if (i == n-1)
            cmd += std::to_string(x[i]) + "," + std::to_string(y[i]) + " ";
        else
            cmd += std::to_string(x[i]) + "," + std::to_string(y[i]) + ",";
    }

    float* x1 = lines.col(0).data();
    float* y1 = lines.col(1).data();
    float* x2 = lines.col(2).data();
    float* y2 = lines.col(3).data();
    int m = lines.rows();
    for (int i = 0; i < m; i++) {
        if (i == m-1)
            cmd += std::to_string(x1[i]) + "," + std::to_string(y1[i]) + "," + std::to_string(x2[i]) + "," + std::to_string(y2[i]) + " ";
        else
            cmd += std::to_string(x1[i]) + "," + std::to_string(y1[i]) + "," + std::to_string(x2[i]) + "," + std::to_string(y2[i]) + ",";
    }

    // Make title 
    cmd += std::string(title);

    // Call the Python script using the system() function
    system(cmd.c_str());
}

MatrixXf generate_lines() {
    // Corner points of the box
    float data[4][2] = {
        {0, 0},
        {0, 100},
        {200, 100},
        {200, 0}
    };

    // Connect the corner points to form a line
    MatrixXf lines(4, 4);
    for (int i = 0; i < 4; i++) {
        lines(i, 0) = data[i][0];
        lines(i, 1) = data[i][1];
        lines(i, 2) = data[(i+1)%4][0];
        lines(i, 3) = data[(i+1)%4][1];
    }

    return lines;
}

MatrixXf generate_data() {
    // Create data points as a double array
    float data[100][2] = {
		{194.0711f, -3.8231f},
		{24.2419f, 77.6648f},
		{29.8258f, 123.4154f},
		{140.6529f, 88.5075f},
		{61.2384f, 27.6846f},
		{11.6752f, 94.618f},
		{67.8886f, 151.5974f},
		{87.9694f, -8.3799f},
		{66.4132f, 150.5174f},
		{106.865f, 134.1155f},
		{135.294f, 95.7334f},
		{112.1564f, -41.0612f},
		{33.3155f, 125.9674f},
		{193.5289f, -4.2452f},
		{46.3338f, 135.6171f},
		{61.7889f, 147.0928f},
		{122.9766f, 112.3226f},
		{121.6199f, -53.8036f},
		{7.7085f, 99.9903f},
		{34.9684f, 127.2227f},
		{65.7349f, 150.0217f},
		{108.4665f, 131.9524f},
		{28.8465f, 122.6865f},
		{53.5039f, 140.9519f},
		{168.8306f, 50.4685f},
		{90.5457f, 156.1166f},
		{147.5697f, -38.2658f},
		{86.9704f, 160.9776f},
		{49.8278f, 138.2435f},
		{81.3284f, 0.5853f},
		{190.4944f, 21.2119f},
		{137.1321f, -45.997f},
		{166.5647f, -24.1997f},
		{138.4987f, -44.9998f},
		{198.1996f, -0.7666f},
		{193.9433f, -3.9317f},
		{87.7513f, 159.9194f},
		{61.2489f, 146.6749f},
		{127.741f, -52.9408f},
		{26.7069f, 121.0885f},
		{155.0325f, -32.724f},
		{155.4866f, -32.3717f},
		{169.8913f, 49.0563f},
		{186.2885f, -9.5799f},
		{144.063f, -40.8735f},
		{34.0433f, 126.5335f},
		{121.8665f, -54.1569f},
		{133.0279f, -49.048f},
		{92.1414f, -14.0213f},
		{130.4579f, -50.9279f},
		{23.284f, 118.5557f},
		{185.7803f, -9.971f},
		{130.6387f, -50.8254f},
		{170.2131f, -21.4941f},
		{151.787f, -35.1426f},
		{15.6428f, 112.8798f},
		{94.2393f, -16.8581f},
		{110.443f, 129.2987f},
		{23.161f, 118.4566f},
		{109.9794f, 129.9064f},
		{119.3529f, 117.2838f},
		{102.7101f, 139.7035f},
		{48.7943f, 137.4509f},
		{139.6879f, -44.1116f},
		{167.9549f, -23.1578f},
		{177.3486f, -16.2215f},
		{97.9608f, -21.895f},
		{122.4145f, -54.8874f},
		{185.7112f, -10.0183f},
		{44.3614f, 50.4707f},
		{169.2841f, -22.1712f},
		{142.4399f, -42.0612f},
		{37.5896f, 129.1603f},
		{104.3641f, -30.5095f},
		{19.8474f, 116.002f},
		{104.1334f, -30.2054f},
		{31.0208f, 68.5178f},
		{196.5526f, -2.0036f},
		{149.2828f, 76.8406f},
		{174.8298f, -18.0721f},
		{138.768f, 91.0572f},
		{51.0138f, 139.1224f},
		{186.0231f, 27.2252f},
		{180.2021f, -14.0995f},
		{159.1666f, -29.6763f},
		{168.5597f, 50.8287f},
		{116.6353f, -47.0697f},
		{26.5984f, 120.9997f},
		{58.9743f, 144.9737f},
		{34.2813f, 126.7135f},
		{47.8399f, 45.8109f},
		{107.2327f, -34.3996f},
		{31.4628f, 67.8986f},
		{127.4673f, -53.1474f},
		{6.5855f, 106.1794f},
		{39.8534f, 130.8491f},
		{101.698f, 141.0872f},
		{145.3634f, 82.1386f},
		{154.8574f, -32.8642f},
		{165.7778f, -24.7728f}
    };

    // Convert to matrix
    MatrixXf myMatrix = arrayToMatrix(&data[0][0], 100, 2);

    return myMatrix;
}

MatrixXf arrayToMatrix(float* data, int numRows, int numCols)
{
    MatrixXf myMatrix(numRows, numCols);

    for (int i = 0; i < numRows * 2; i++)
    {
        float x = data[i];
        i++;
        float y = data[i];
        myMatrix(i / 2, 0) = x;
        myMatrix(i / 2, 1) = y;
    }
    
    return myMatrix;
}

