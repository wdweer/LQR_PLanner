#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <fstream>   // For file output
#include <cstdlib> 

using namespace Eigen;
using namespace std;

class LQRPlanner {
public:
    double MAX_TIME = 100.0;
    double DT = 0.1;
    double GOAL_DIST = 0.1;
    int MAX_ITER = 150;
    double EPS = 0.01;
    double v = 10.0;
    double theta = 0.0;
    double delta = 0.0;

    pair<vector<double>,vector<double>> lqr_planning(double sx, double sy, double gx, double gy);
    
private:
    pair<MatrixXd, MatrixXd> get_system_model();
    MatrixXd solve_dare(const MatrixXd &A, const MatrixXd &B, const MatrixXd &Q, const MatrixXd &R);
    MatrixXd lqr_control(const MatrixXd &A, const MatrixXd &B, const MatrixXd &x);
    MatrixXd dlqr(const MatrixXd &A, const MatrixXd &B, const MatrixXd &Q, const MatrixXd &R);
};

pair<vector<double>,vector<double>> LQRPlanner::lqr_planning(double sx, double sy, double gx, double gy) {
    vector<double> rx = {sx}, ry = {sy};
    MatrixXd x(4, 1);
    x << sx - gx, sy - gy, 0, 0;

    auto [A, B] = get_system_model();

    double time = 0.0;
    while (time <= MAX_TIME) {
        time += DT;

        auto u = lqr_control(A, B, x);
        x = A * x + B * u;

        rx.push_back(x(0, 0) + gx);
        ry.push_back(x(1, 0) + gy);

        double d = sqrt(pow(gx - rx.back(), 2) + pow(gy - ry.back(), 2));
        if (d <= GOAL_DIST) {
            break;
        }
    }

    return {rx, ry};
}

pair<MatrixXd, MatrixXd> LQRPlanner::get_system_model() {
    double Lf = 1.5, Lr = 1.5;
    double wheelbase = Lf + Lr;

    MatrixXd A(4, 4);
    A << 1.0, 0.0, -DT * v * sin(theta), DT * cos(theta),
         0.0, 1.0, DT * v * cos(theta), DT * sin(theta),
         0.0, 0.0, 1.0, DT * tan(delta) / wheelbase,
         0.0, 0.0, 0.0, 1.0;

    MatrixXd B(4, 2);
    B << 0.0, 0.0,
         0.0, 0.0,
         DT / wheelbase, 0.0,
         0.0, DT;

    return {A, B};
}

MatrixXd LQRPlanner::solve_dare(const MatrixXd &A, const MatrixXd &B, const MatrixXd &Q, const MatrixXd &R) {
    MatrixXd X = Q;
    for (int i = 0; i < MAX_ITER; ++i) {
        MatrixXd Xn = A.transpose() * X * A - A.transpose() * X * B * (R + B.transpose() * X * B).inverse() * B.transpose() * X * A + Q;
        if ((X - Xn).norm() < EPS) {
            break;
        }
        X = Xn;
    }
    return X;
}

MatrixXd LQRPlanner::dlqr(const MatrixXd &A, const MatrixXd &B, const MatrixXd &Q, const MatrixXd &R) {
    MatrixXd X = solve_dare(A, B, Q, R);
    MatrixXd K = (B.transpose() * X * B + R).inverse() * (B.transpose() * X * A);
    return K;
}

MatrixXd LQRPlanner::lqr_control(const MatrixXd &A, const MatrixXd &B, const MatrixXd &x) {
    auto Q = MatrixXd::Identity(4, 4);
    auto R = MatrixXd::Identity(2, 2);
    MatrixXd K = dlqr(A, B, Q, R);
    return -K * x;
}

int main() {
    LQRPlanner planner;
    vector<pair<double,double>> goal = {{0, 3}, {1, 8}, {2, 13}, {3, 18}, {4, 23}, {5, 28}, {6, 33}, {7, 38}, {8, 43}, {9, 48}};
    vector<pair<vector<double>, vector<double>>> results;
    
    // Loop through the goal points and run the lqr_planning method
    for (size_t i = ; i < goal.size() - 1; i++) {
        // Send each consecutive pair of points in the goal as a line segment

        
        // Call lqr_planning with start (x, y) and line segment (start, end)
        pair<vector<double>, vector<double>> result = planner.lqr_planning(goal[i-1].first,goal[i-1].second, goal[i].first, goal[i].second);
        results.push_back(result);
    }
    
    // Check if we have any valid results and output all paths
    if (!results.empty()) {
        ofstream data_file("lqr_path.dat");
        cout << "Paths (x, y):" << endl;
        
        for (const auto& result : results) {
            if (result.first.size() >= 2 && result.second.size() >= 2) {
                // Output each (x, y) path
                for (size_t i = 0; i < result.first.size(); ++i) {
                    cout << "(" << result.first[i] << ", " << result.second[i] << ")" << endl;
                    data_file << result.first[i] << " " << result.second[i] << endl;
                }
                cout << "----------------" << endl;  // Separator between paths
            }
        }
        
        data_file.close();
        
        // Plot the data using gnuplot
        system("gnuplot -e \"set terminal wxt; set title 'LQR Path'; plot 'lqr_path.dat' with linespoints; pause -1\"");
        
    } else {
        cout << "No valid paths found." << endl;
    }

    return 0;
}



