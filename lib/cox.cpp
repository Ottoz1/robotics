#include "cox.hpp"

VectorXf cox_linefit(MatrixXf points, MatrixXf line_segments, int max_iter)
{
    // The translation and rotation to be returned
    float ddx = 0.0f;
    float ddy = 0.0f;
    float dda = 0.0f;

    // Create a copy of the points (we don't want to modify the original)
    MatrixXf pts = points;

    // Find the normals of the line segments
    MatrixXf normals = find_normals(line_segments);

    // main loop
    for (int i = 0; i < max_iter; i++)
    {
        // Find the closest line segment for each point
        MatrixXf distances = MatrixXf::Zero(pts.rows(), 1);
        MatrixXf new_normals = MatrixXf::Zero(pts.rows(), 2);
        MatrixXf targets = assign_points_to_lines(pts, line_segments, normals, &distances, &new_normals);

        float mean = distances.mean();
        printf("DD: %f", mean);
 
        // Setup the variables of the linear system of equations (least squares problem)
        VectorXf y = get_signed_distance(pts, targets, new_normals); // The signed distance from the points to the line segments
        VectorXf xi1 = new_normals.col(0);  // The x component of the normal vector of the line segments
        VectorXf xi2 = new_normals.col(1);  // The y component of the normal vector of the line segments

        // Figure out xi3 according to the Cox paper
        VectorXf vm = VectorXf::Zero(2);   // The mean of the points
        vm(0) = pts.col(0).mean();
        vm(1) = pts.col(1).mean();
        MatrixXf diff = pts.rowwise() - vm.transpose(); // Subtract the mean from the points
        MatrixXf m(2, 2);
	    m << 0, -1, 1, 0;
        diff = m * diff.transpose();    // Multiply the difference matrix with the rotation matrix
        MatrixXf temp = diff.transpose();    // The z component of the normal vector of the line segments
        VectorXf xi3 = (new_normals.array() * temp.array()).rowwise().sum();

        // Solve the linear system of equations
        MatrixXf A = MatrixXf::Zero(pts.rows(), 3);
        A.col(0) = xi1;
        A.col(1) = xi2;
        A.col(2) = xi3;

        // Ab = y => A^T * A * b = A^T * y => b = (A^T * A)^-1 * A^T * y
        VectorXf b = (A.transpose() * A).inverse() * A.transpose() * y;

        // Update the translation and rotation
        ddx += b(0);
        ddy += b(1);
        dda += b(2);

        // Update the points with the new translation and rotation
        VectorXf transformation(3);
        transformation << ddx, ddy, dda;
        pts = transform_points(points, transformation);

        // Check if the algorithm has converged
        if (b.norm() < 0.0001f)
        {
            VectorXf result(3);
            result << ddx, ddy, dda;
            return result;
        }
    }
    VectorXf result(3);
    result << ddx, ddy, dda;
    return result;
}

MatrixXf assign_points_to_lines(MatrixXf points, MatrixXf line_segments, MatrixXf normals, MatrixXf *distances, MatrixXf *new_normals_ptr)
{
    // points: Matrix with 2 columns (x, y) and n rows (n points to be assigned)
    // line_segments: Matrix with 4 columns (x1, y1, x2, y2) and m rows (m line segments)
    // normals: Matrix with 2 columns (dx, dy) and m rows (m normal vectors to the line_segments)
    // output: distances: Matrix with 1 column (distance) and n rows (n distances) (distances from the points to the closest line segment)
    // output: Matrix with 2 columns (x, y) and n rows (n points) (the closest point on the line segment for each point)

    int n = points.rows();  // Number of points
    int m = line_segments.rows();   // Number of line segments
    MatrixXf targets = MatrixXf::Zero(n, 4);    // The closest line segment for each point
    MatrixXf new_normals = MatrixXf::Zero(n, 2);    // The normal vector of the closest line segment for each point
    MatrixXf dists = MatrixXf::Zero(n, 1);   // The distance from the point to the closest line segment

    for (int i = 0; i < n; i++){    // Loop through all points
        int min_index = 0;  // The index of the line segment with the smallest distance
        float min_dist = 1000000.0f;    // The smallest distance
        for (int j = 0; j < m; j++){    // Loop through all lines
            float dist = point_segment_distance(points.row(i), line_segments.row(j)); // The distance from the point to the line segment
            if (dist < min_dist){   // If the distance is smaller than the smallest distance
                min_dist = dist;    // Update the smallest distance
                min_index = j;  // Update the index of the line segment with the smallest distance
            }
        }
        dists(i) = min_dist;    // Update the distance matrix
        targets.row(i) = line_segments.row(min_index);   // Update the target matrix
        new_normals.row(i) = normals.row(min_index);    // Update the normal matrix
    }

    *distances = dists; // Update the distances pointer
    *new_normals_ptr = new_normals; // Update the new_normals pointer
    return targets;
}

float point_segment_distance(VectorXf point, VectorXf line_segment)
{
    // point: Vector with 2 elements (x, y) (the point)
    // line_segment: Vector with 4 elements (x1, y1, x2, y2) (the line segment)
    // output: The distance from the point to the line segment

    // The following variable names are consistent with Björn's instructions from lab3
    VectorXf vi = point; // The current point
    VectorXf ui = line_segment.tail(2) - line_segment.head(2); // The direction vector of the line segment
    VectorXf z = line_segment.head(2); // The first endpoint of the current line segment

    // Find the closest point on the line segment to the point
    float ri = ui.dot(z);   // The distance from the origin to the line
    float ui_norm = ui.squaredNorm(); // The squared norm of the direction vector of the line segment
    float t = ui.dot(vi - z) / ui_norm; // The parameter t of the closest point on the line segment
    t = std::max(0.0f, std::min(1.0f, t)); // Clamp t to the range [0, 1]
    VectorXf closest_point = z + t * ui; // The closest point on the line segment

    // Calculate the distance between the closest point and the point
    float distance = (closest_point - vi).norm();

    return distance;
}

VectorXf get_signed_distance(MatrixXf points, MatrixXf targets, MatrixXf normals){
    // points: Matrix with 2 columns (x, y) and n rows (n points)
    // targets: Matrix with 4 columns (x1, y1, x2, y2) and n rows (n line segments)
    // normals: Matrix with 2 columns (dx, dy) and n rows (n normal vectors)
    // output: The signed distance from the points to the line segments

    VectorXf y = VectorXf::Zero(points.rows()); // The signed distance from the points to the line segments
    for (int i = 0; i < points.rows(); i++){
        VectorXf vi = points.row(i); // The current point
        VectorXf target = targets.row(i);   // The closest line segment for the current point
        VectorXf ui = normals.row(i);   // The normal vector of the closest line segment for the current point
        VectorXf z = target.head(2);    // The first endpoint of the closest line segment for the current point

        // Find the closest point on the line segment to the point
        float ri = ui.dot(z);   // The distance from the origin to the line
        float yi = ri - ui.dot(vi); // The signed distance from the point to the line segment
        y(i) = yi;  // Update the signed distance matrix
    }

    return y;
}


// This function is used to find the normals of the line segments
MatrixXf find_normals(MatrixXf lines){
    // lines: Matrix with 4 columns (x1, y1, x2, y2) and n rows (n line segments)
    // output: Matrix with 2 columns (dx, dy) and n rows (n normal vectors)

    VectorXf x1 = lines.col(0); // Endpoints 1 of the line segments (x1, y1)
    VectorXf y1 = lines.col(1);
    VectorXf x2 = lines.col(2); // Endpoints 2 of the line segments (x2, y2)
    VectorXf y2 = lines.col(3);

    // Find the normals (by rotating the vectors 90 degrees)
    VectorXf dx = y1 - y2;
    VectorXf dy = x2 - x1;

    // Normalize the normals
    VectorXf norm = (dx.cwiseProduct(dx) + dy.cwiseProduct(dy));    // norm^2 = dx^2 + dy^2   (element-wise multiplication)
    norm = norm.cwiseSqrt();    // norm (element-wise square root)
    dx = dx.cwiseQuotient(norm);    // dx = dx / norm
    dy = dy.cwiseQuotient(norm);    // dy = dy / norm

    // Store the normals in a matrix
    MatrixXf normals = MatrixXf::Zero(lines.rows(), 2);
    normals.col(0) = dx;
    normals.col(1) = dy;

    return normals;
}

// This function is only used for plotting
// It uses python and string which may not be available in the Pi
// It should not be used in the final version


MatrixXf generate_lines() {
    // Corner points of the box
    float data[4][2] = {
        {0, 0},
        {0, 2430},
        {3630, 2430},
        {3630, 0}
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

MatrixXf polar_to_cart(MatrixXf polar){
    MatrixXf cartesian(200,2);
    cartesian.col(0) = polar.col(0).array() * polar.col(1).array().cos();
    cartesian.col(1) = polar.col(0).array() * polar.col(1).array().sin();
    return cartesian;
}

MatrixXf transform_points(MatrixXf points, VectorXf transformation)
{
    // points: Matrix with 2 columns (x, y) and n rows (n points)
    // transformation: Vector with 3 elements (x, y, theta)
    // output: Matrix with 2 columns (x, y) and n rows (n transformed points)
    float ddx = transformation(0);
    float ddy = transformation(1);
    float dda = transformation(2);

    MatrixXf T(3, 3);
    T << cos(dda), -sin(dda), ddx,
         sin(dda), cos(dda), ddy,
         0, 0, 1;
    MatrixXf temp_pts = points.rowwise().homogeneous();
    MatrixXf temp_pts2 = temp_pts * T.transpose();
    MatrixXf pts = temp_pts2.leftCols(2);
    return pts;
}

MatrixXf generate_data() {
    // Create data points as a double array
    float data[100][2] = {
		{7.080281247585631f, 93.08368110067224f},
		{7.425491943899104f, 94.03937863873513f},
		{15.614526914572693f, 26.9526367687294f},
		{53.48953585441349f, 13.259982472516562f},
		{-2.9657278388578447f, 65.3299789332093f},
		{19.69970045499521f, 127.9695935509652f},
		{192.72602590036922f, 18.377514016919257f},
		{75.48562317661177f, 5.283090215318062f},
		{0.2740982448011522f, 74.28052356796277f},
		{34.30071260788145f, 126.53003746847533f},
		{142.81990135575688f, 87.25333523263957f},
		{15.247436144712537f, 115.6458447879665f},
		{204.2418528327049f, 50.09205431237615f},
		{20.209491220899807f, 129.3817283653931f},
		{14.150216591572047f, 112.65909724857306f},
		{167.727713995951f, 78.24817782755015f},
		{184.45053636820882f, -4.475618209185697f},
		{47.896947144551184f, 121.6116349564521f},
		{32.29670887455062f, 20.90240103279305f},
		{194.86494101147218f, 24.29865505288314f},
		{139.91086527824152f, -18.030895478798925f},
		{193.47396068197435f, 20.3954622195699f},
		{204.38232309373996f, 50.58599775323272f},
		{7.9098341270617425f, 95.41517115405122f},
		{201.43776432047366f, 42.38284199989922f},
		{198.4259559351705f, 67.11802079829401f},
		{105.56218649887504f, 100.73213957157054f},
		{92.75693672567299f, -0.9705113935825551f},
		{39.6022128028465f, 124.61474356260219f},
		{146.32500651716376f, 85.9797075401516f},
		{142.80125453692636f, -19.11260184544132f},
		{204.5915582576154f, 51.14677276533557f},
		{188.30115589830115f, 6.118761187676988f},
		{6.299225291510126f, 90.9700603661983f},
		{8.489262383116682f, 97.00777682860299f},
		{205.50912112820149f, 53.69996260615691f},
		{55.52057708020884f, 12.492485475373236f},
		{9.136517597086609f, 98.75826974879908f},
		{1.0142733534927402f, 76.31900151601725f},
		{-4.756658975886268f, 60.340002484859724f},
		{111.38180916209701f, -7.724064010099106f},
		{144.3790752105497f, 86.6910800831037f},
		{167.3123877238187f, 78.36314173270799f},
		{105.60684703746217f, -5.638349229575567f},
		{23.496691555122126f, 130.45257571749846f},
		{178.52932081646964f, 74.32224669153301f},
		{175.91015120729546f, -28.10737809835284f},
		{86.80799225831433f, 1.1718928346520414f},
		{31.42062344988615f, 127.58852752513486f},
		{159.98353169506944f, 81.03354653967111f},
		{1.9249248015799623f, 78.88478171323925f},
		{152.34324501656988f, -22.547854722560217f},
		{84.50179825599581f, 2.011961306280881f},
		{80.74194640423391f, 109.72853051218982f},
		{2.266487590140457f, 31.784801705379888f},
		{44.713160968124285f, 16.412848248158348f},
		{12.260991026265813f, 28.172912802961758f},
		{191.17216125987272f, 14.100212533196952f},
		{194.17502757323643f, 68.63180676279582f},
		{18.541395966860236f, 124.73545990480888f},
		{173.18230853198185f, -30.102680217657777f},
		{204.50515302062135f, 50.893137556708595f},
		{15.53354596946609f, 116.44999345990705f},
		{202.77221913113536f, 46.0693803835823f},
		{190.16089604660542f, 11.297236454577423f},
		{15.121365586656594f, 115.28725442222849f},
		{204.0091077103169f, 49.51508980565449f},
		{199.81087764524582f, 37.904298721499984f},
		{199.16171326336945f, 36.153065163791055f},
		{-4.64702216886478f, 60.75283176275594f},
		{100.44910927412386f, -3.768358789076103f},
		{189.50623551359132f, 9.482823321902558f},
		{17.23259010569663f, 121.19020114474893f},
		{105.03039882095068f, 100.93255535650648f},
		{20.21006546106554f, 129.36563385940036f},
		{119.0098147492824f, -10.512980045002763f},
		{98.40565912813594f, 103.33447222238134f},
		{9.24825717856514f, 29.255191061729924f},
		{-2.4352318906777555f, 66.80034424502303f},
		{185.40499701836376f, -1.8018229999048216f},
		{9.357955490116339f, 99.38845255892473f},
		{200.7346656068244f, 66.29502538933497f},
		{95.20306008962963f, 104.49640676230183f},
		{7.536046235555688f, 94.38416873031886f},
		{109.36257374544888f, 99.37685176541524f},
		{50.794509629552756f, 14.211023385988383f},
		{-0.029949903617264795f, 73.3900690044579f},
		{101.57574869907863f, 102.19457775849969f},
		{205.49540401572395f, 53.62123372787093f},
		{189.58738685234516f, 9.701164194232533f},
		{124.70589300300401f, 93.82507926280334f},
		{186.7526636978445f, 1.8639795621347588f},
		{81.31118285950008f, 109.51505018700985f},
		{178.4541773501536f, -21.019013743841356f},
		{205.68925104320164f, 54.13909186713974f},
		{7.2233790357346095f, 30.00398598365412f},
		{121.91745349267318f, 94.80932122498007f},
		{156.09801024778747f, -23.91151026481336f},
		{-1.1099077604911542f, 70.47635819182715f},
		{174.98751373783182f, -30.63060645995897f}
    };

    // Convert to matrix
    return arrayToMatrix(&data[0][0], 100, 2);
}

MatrixXf polar_to_cartesian(MatrixXf polar){
    
}