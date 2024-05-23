#include <iostream>
#include <fstream>
#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>

#include "custom_msgs/msg/local_map_msg.hpp"
#include "custom_msgs/msg/pose_msg.hpp"
#include "custom_msgs/msg/vel_estimation.hpp"

using namespace Eigen;
using namespace std;
using namespace std::chrono;

// For testing
bool mapping = true;

class SlamNode : public rclcpp::Node {
public:
    SlamNode() : Node("Slam_node") {
       map_publisher_ = create_publisher<custom_msgs::msg::LocalMapMsg>("local_map", 10);
	   pose_publisher_ = create_publisher<custom_msgs::msg::PoseMsg>("pose", 10);    
    }

    void runSlamAlgorithm() {
        // Initialize state vector, covariance matrix, noise
        VectorXd state_vector(3);
        state_vector << 0, 0, 0;

        MatrixXd Sigma(3,3);
        Sigma << 0.5, 0, 0,
                 0, 0.5, 0,
                 0, 0, 0.2;

        Matrix3d Q;
        Q << 0.1, 0, 0,
             0, 0.1, 0,
             0, 0, 0.01;

        velocityFile.open("/home/chris/Desktop/P23-DV/ROS_Workspace/src/4.SLAM/ekf_slam/DataFiles/good_velocityLog.txt");
        perceptionFile.open("/home/chris/Desktop/P23-DV/ROS_Workspace/src/4.SLAM/ekf_slam/DataFiles/good_perceptionLog.txt");
        if(!mapping){trackFile.open("/home/chris/Desktop/P23-DV/ROS_Workspace/src/4.SLAM/slam/Tracks/Acceleration.txt");}

        if (!velocityFile.is_open() || !perceptionFile.is_open()) {
            std::cerr << "Error opening input files." << std::endl;
            return;
        }
    
    int step_cnt = 0;
    uint32_t globalIndexVelocity;
    uint32_t globalIndexPerception;
    
    perceptionFile >> globalIndexPerception;
    velocityFile >> globalIndexVelocity;
    Vector3d velocity;

    // Variables to store timing data
    vector<double> update_times;
    auto start_time = high_resolution_clock::now();

    while(rclcpp::ok() && !velocityFile.eof() && !perceptionFile.eof())
    {
        while(globalIndexVelocity != globalIndexPerception){
            velocity = readVelocity();
            predictionStep(state_vector, Sigma, velocity, Q);
            publishPoseMessage(state_vector, globalIndexVelocity, velocity);

            velocityFile >> globalIndexVelocity;  
            //cout<<globalIndexVelocity<<endl;
        }

        velocity = readVelocity();
        while(globalIndexVelocity == globalIndexPerception){
            readOdometry();
            predictionStep(state_vector, Sigma, velocity, Q);
            publishPoseMessage(state_vector, globalIndexVelocity, velocity);

            // Measure the time before the update step
            auto update_start = high_resolution_clock::now();
            updateStep(state_vector, Sigma, velocity);
            auto update_end = high_resolution_clock::now();

            // Calculate and store the duration in milliseconds
            duration<double, milli> update_duration = duration_cast<duration<double, milli>>(update_end - update_start);
            update_times.push_back(update_duration.count());

            publishPoseMessage(state_vector, globalIndexVelocity, velocity);
            publishMapMessage(state_vector, globalIndexPerception, velocity);

            perceptionFile >> globalIndexPerception;
            //cout<<globalIndexVelocity<<endl;
        }
        velocityFile >> globalIndexVelocity; 
        if(globalIndexPerception == 1644){break;}
        //std::cout<<globalIndexVelocity<<" Velocity goes with Perception: "<<globalIndexPerception<<std::endl;
        if(velocityFile.eof() || perceptionFile.eof()){
            velocityFile.close();
            perceptionFile.close();
            cout<<"THATS ALL FOLKS\n"; 
            break;}
            //cout<<globalIndexPerception<<endl;
            //cout<<(state_vector.size()-3)/2<<endl;
        
        // Estimated State
        //rclcpp::sleep_for(std::chrono::milliseconds(100));
        //cout<<"STATE: "<<state_vector.head(3).transpose()<<endl;
        cout<<"LANDMARKS: "<<state_vector.segment(3, 3).transpose()<<endl;
        //std::cout << "Step: " << globalIndexPerception<<endl; //<< ", Estimated State: " << state_vector.transpose() << std::endl;
        step_cnt++;
    }

    // Output timing data to a file
    ofstream timing_file("/home/chris/Desktop/update_times.txt");
    if (timing_file.is_open()) {
        for (const auto& t : update_times) {
            timing_file << t << endl;
        }
        timing_file.close();
    } else {
        cerr << "Error opening timing file for writing." << endl;
    }
  }


// Function to perform kinematic update on pose
Vector3d kinematic_update(const VectorXd& pose, const VectorXd& velocity) 
{
    double dt = 0.025;  
    double v_x = velocity(0);
    double v_y = velocity(1);  
    double omega = velocity(2);

    // Update the pose (x, y, theta)
    double x = pose(0) + (v_x * std::cos(pose(2)) * dt) - (v_y * std::sin(pose(2)) * dt);    
    double y = pose(1) + (v_x * std::sin(pose(2)) * dt) + (v_y * std::cos(pose(2)) * dt);  
    double theta = pose(2) + omega * dt;     
        
    Vector3d new_pose(3);
        new_pose << x,y,theta;                                              

    return new_pose;
}

// Function to compute the motion model Jacobian
Matrix3d motion_jacobian(const VectorXd& pose, const VectorXd& velocity) 
{
    double dt = 0.025;  
    double v_x = velocity(0);
    double v_y = velocity(1);  
    double theta = pose(2);    // * dt;

        Matrix3d Gx;                                                 // change - and 1 in dt
        Gx << 1, 0, - v_x * sin(theta) * dt - v_y * cos(theta) * dt,
              0, 1, v_x * cos(theta) * dt - v_y * sin(theta) * dt,
              0, 0, 1; //or 0
    
    return Gx;
}

// Function to compute noise transformation into state space
Matrix3d noise_transformation(const VectorXd& pose)
{
    double dt = 0.025;  
    double theta = pose(2);

        Matrix3d Vx;
        Vx << cos(theta) * dt, - sin(theta) * dt, 0,
              sin(theta) * dt, cos(theta) * dt, 0,
              0, 0, dt;

    return Vx;
}

// Function to update covariance matrix
MatrixXd covariance_update(MatrixXd& Sigma, const MatrixXd& Gt, int state_size)
{
    int N = state_size - 3;

    Sigma.topLeftCorner(3, 3) = Gt * Sigma.topLeftCorner(3, 3) * Gt.transpose();
    Sigma.topRightCorner(3, N) = Gt * Sigma.topRightCorner(3, N);
    Sigma.bottomLeftCorner(N, 3) = (Sigma.topRightCorner(3, N)).transpose();
    return Sigma;
}

// Function to perform the prediction step
void predictionStep(VectorXd& state_vector, MatrixXd& Sigma, const VectorXd& velocity, const Matrix3d& Q) 
{
    // Calculating jacobian of motion model
    Matrix3d Gt = motion_jacobian(state_vector.head(3), velocity); 

    // Noise Transformation into State Space
    MatrixXd Vt = noise_transformation(state_vector.head(3));
    Matrix3d Qt = Vt * Q * Vt.transpose();
    //cout<<Qt.transpose()<<endl;
    // State Prediction
    state_vector.head(3) = kinematic_update(state_vector.head(3), velocity);

    // Covariance Prediction
    if(mapping){
        MatrixXd expandedQ = MatrixXd::Zero(Sigma.rows(), Sigma.cols());
        expandedQ.topLeftCorner(3, 3) = Qt;
        //cout<<"SIGMA BEFORE PREDICT : "<<endl;
        //cout<<Sigma.topLeftCorner(3,3).transpose()<<endl;
        //cout<<"Q:\n"<<expandedQ.transpose()<<endl;
        Sigma = covariance_update(Sigma, Gt, state_vector.size()) + expandedQ;
        //cout<<"SIGMA AFTER PREDICT : "<<endl;
        //cout<<Sigma.col(2).head(2).transpose()<<endl;
    }
    else{Sigma = (Gt * Sigma * Gt.transpose()) + Q;}
}

// Function to perform data association
int test = 0;
vector<pair<int, int> > data_association(const VectorXd& state_vector, vector<double> range,
                                        vector<double> bearing, vector<int> color, vector<int> &unmatched) 
{
    double x = state_vector(0);
    double y = state_vector(1);
    double theta = state_vector(2);
    double association_distance_threshold = 1.9;
    vector<pair<int, int> > proccessing;

    for(int j = 0; j < range.size(); ++j){
        double least_distance_square = std::pow(association_distance_threshold, 2);
        //if(color[j] == 3){least_distance_square = std::pow(1.4 * association_distance_threshold, 2);}
        int best_match = -1;
        double x_land = x + range[j] * cos(theta + bearing[j]);
        double y_land = y + range[j] * sin(theta + bearing[j]);

        // Iterate through all of the cones in the current map
        for (int i = 0; i < landmark_cnt; ++i) 
        {   
          double x_existing = state_vector(2 * i + 3);
          double y_existing = state_vector(2 * i + 4);
          double current_distance_square = std::pow(x_land - x_existing, 2) + std::pow(y_land - y_existing, 2);//cout<<"state vector: "<<state_vector.transpose()<<endl;
         if(current_distance_square < least_distance_square && color[j] == landmark_colors[i]) 
          {
            least_distance_square = current_distance_square; 
            best_match = i;
          }//cout<<"PANIK"<<endl;
        }
        if(best_match >= 0){test++; proccessing.push_back(std::make_pair(best_match, j));}
        else{unmatched.push_back(j);}
    }
    return proccessing;
}

// Function to add new landmarks
void add_new_landmarks(VectorXd& state_vector, MatrixXd& Sigma, vector<double> range,
                       vector<double> bearing, vector<int> color, vector<int> unmatched)
{
    double x = state_vector(0);
    double y = state_vector(1);
    double theta = state_vector(2);
    int current;

    for(int i = 0; i < unmatched.size(); ++i)
    {   
        current = unmatched[i];    
        if(range[current] > 7.6){continue;}
        double x_land = x + range[current] * cos(theta + bearing[current]);
        double y_land = y + range[current] * sin(theta + bearing[current]);
        //cout<<"range: "<<range[current]<<" cos: "<<cos(theta+bearing[current])<<endl;
        //cout<<"x_land: "<<x_land<<endl;
        //cout<<" y_land: "<<y_land<<endl;
        landmark_colors.push_back(color[current]);
        ++landmark_cnt;
        
        VectorXd new_state_vector(state_vector.size() + 2);
        new_state_vector << state_vector, x_land, y_land;

        MatrixXd Hu_inv(2,3);
            Hu_inv << 1, 0, - range[current] * sin(theta + bearing[current]),
                      0, 1, range[current] * cos(theta + bearing[current]);
        
        MatrixXd H_inv = MatrixXd::Zero(2, state_vector.size());
        H_inv.block(0, 0, 2, 3) = Hu_inv;
        
        MatrixXd Hi_inv(2,2);
            Hi_inv << cos(theta + bearing[current]), - range[current] * sin(theta + bearing[current]),
                      sin(theta + bearing[current]), range[current] * cos(theta + bearing[current]);

        Matrix2d Rt;                              
        Rt << 0.011*std::pow(range[current]+1,2) - 0.082*(range[current]+1) + 0.187, 0,
			  0, 0.001;
        
        // Expand the covariance matrix
        //cout<<"SIGMA BEFORE UPDATE : "<<endl;
        //cout<<Sigma.transpose()<<endl;
        MatrixXd new_Sigma = MatrixXd::Zero(Sigma.rows() + 2, Sigma.cols() + 2);
        new_Sigma.topLeftCorner(Sigma.rows(), Sigma.cols()) = Sigma;
        //cout<<"H_inv * Sigma : "<<H_inv * Sigma<<endl;
        new_Sigma.bottomLeftCorner(2, Sigma.cols()) = H_inv * Sigma;
        new_Sigma.topRightCorner(Sigma.rows(), 2) = new_Sigma.bottomLeftCorner(2, Sigma.cols()).transpose();
        new_Sigma.bottomRightCorner(2, 2) = H_inv * Sigma * H_inv.transpose() + Hi_inv * Rt * Hi_inv.transpose();
        //cout<<"SIGMA AFTER UPDATE : "<<endl;
        state_vector = new_state_vector;
        Sigma = new_Sigma;
        //cout<<Sigma.transpose()<<endl;
    }
}

// UPDATE STEP
void updateStep(VectorXd& state_vector, MatrixXd& Sigma, Vector3d& velocity) 
{
    double x = state_vector(0);
    double y = state_vector(1);
    double theta = state_vector(2);
    vector<double> range = measurements.range_list;
    vector<double> bearing = measurements.theta_list;
    vector<int> color = measurements.class_list;
    vector<int> unmatched;
    vector<pair<int, int> > matched;
    
    // 1: matched_landmark, 2: measurement_index
    matched = data_association(state_vector, range, bearing, color, unmatched);
    //cout<<"DATA ASSOCIATED: "<<test<<endl;
    //cout<<"AFTER DA: "<<state_vector.head(3).transpose()<<endl;
    // add unmatched measurements on mapping mode
    if(mapping)
    {
        add_new_landmarks(state_vector, Sigma, range, bearing, color, unmatched);
    }  
      //  cout<<"AFTER NEW_LANDMARKS: "<<state_vector.head(3).transpose()<<endl;
    MatrixXd Ht = MatrixXd::Zero(2 * matched.size(), state_vector.size());
    MatrixXd Dzt = MatrixXd::Zero(2 * matched.size(), 1);
    MatrixXd Rt = MatrixXd::Zero(2 * matched.size(), 2 * matched.size());
      
    for(int i = 0; i < matched.size(); ++i)
    {   
            // Actual observation
            MatrixXd zt(2,1);
                zt << range[matched[i].second],
                      bearing[matched[i].second];
            
            double x_land = state_vector(2 * matched[i].first + 3);
            double y_land = state_vector(2 * matched[i].first + 4);
            double dx = x_land - x;
            double dy = y_land - y;
            //cout<<"dx: "<<dx<<endl;
            //cout<<"dy: "<<dy<<endl;

            MatrixXd d(2,1);
                d << dx,
                     dy;

            double q = (d.transpose() * d)(0,0);
            double q_sqrt = sqrt(q);
            //cout<<range[matched[i].second]<<endl;
            //cout<<atan2(dy,dx)<<endl;
            //cout<<"RANGE DIFF: "<<range[matched[i].second] - q_sqrt<<endl;
            //cout<<"BEARING DIFF: "<<bearing[matched[i].second]-atan2(dy,dx)<<endl;//-atan2(dy,dx)<<endl;
            // EXPECTED OBSERVATION
            MatrixXd zt_exp(2,1);
                zt_exp << q_sqrt,
                          atan2(dy,dx) - theta;
            
            MatrixXd Htu(2,3);                              //Partial Jacobian per pose
                Htu << - q_sqrt * dx, - q_sqrt * dy, 0,
                            dy, - dx, - q; 

           if(mapping)
           { 
            Dzt.block(2 * i, 0, 2, 1) = zt - zt_exp;
            //cout<<"for measurements num: "<<matched[i].second<<" : "<<(zt-zt_exp).transpose()<<endl;
            MatrixXd Htj(2,2);                              //Partial jacobian per range - theta
                Htj << q_sqrt * dx, q_sqrt * dy,
                        - dy, dx;
            
            Ht.block(2 * i, 0, 2, 3) = Htu;
            Ht.block(2 * i, 2 * i + 3, 2, 2) = Htj;  //2 * matched[i].first + 3
            //cout<<"Ht: "<<endl;
            //cout<<Ht<<endl;
            Rt.block(2 * i, 2 * i, 1, 1).setConstant(0.011 * std::pow(
                range[matched[i].second] + 1, 2) - 0.082 * (range[matched[i].second] + 1) + 0.187);
            Rt.block(2 * i + 1, 2 * i + 1, 1, 1).setConstant(0.001);
           }
           else{
                MatrixXd Ht = Htu;
                MatrixXd Dzt = zt - zt_exp;
           }
    }   
        if(matched.size() > 0){
        //cout<<Rt.transpose()<<endl;
        MatrixXd SHt = Sigma * Ht.transpose();
        MatrixXd HSRt = (Ht * SHt) + Rt;
        MatrixXd Kt = SHt * HSRt.inverse();
        //noalias()
        // FINAL STATE
        VectorXd KD = Kt * Dzt;
        //state_vector.segment(3, state_vector.rows() - 3) += KD.bottomRows(KD.rows() - 3); 
        //cout<<"STATE BEFORE:\n"<<state_vector.transpose()<<endl;
        //cout<<"KALMAN GAIN:\n"<<Kt.row(1)<<endl;
        //cout<<"DZT:\n"<<Dzt.transpose()<<endl;
        //cout<<"KD:\n"<<KD.transpose()<<endl;
        //if(KD(1) > 2){cout<<"FOUND"<<endl;}
        /*test_val.push_back(Dzt(2,0));
        sort(test_val.begin(), test_val.end());
        cout<<test_val.back()<<endl; */
        state_vector += KD;
        //if(KD(1)> 0.9){
        //cout<<KD(1)<<endl;}
        //state_vector(1) = 0;
        //state_vector(2) = 0;
        //cout<<"STATE AFTER: "<<endl;
        //cout<<state_vector.transpose()<<endl;
        // FINAL COV MATRIX
        MatrixXd KH = Kt * Ht; // Compute Kt * Ht
        Sigma -= KH * Sigma; // Update Sigma in-place
        } 
}

// Function to read a list of values from a line and store them in a vector
template<typename T>
void readList(std::istream& input, std::vector<T>& output) {
    std::string line;
    std::getline(input >> std::ws, line);
    std::istringstream stream(line);
    T value;
    while (stream >> value) {
        output.push_back(value);
    }
}

Vector3d readVelocity(){
    // Read velocity from file
    Vector3d velocity;
    for (int i = 0; i < 3; ++i)
    {
        velocityFile >> velocity(i);
    }

    // Read variance matrix from velocity file
    Matrix<double, 3, 3> varianceMatrixVelocity;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            velocityFile >> varianceMatrixVelocity(i, j);
        }
    }
    //cout<<varianceMatrixVelocity.transpose()<<endl;
    //cout<<velocity.transpose()<<endl;
    return velocity;
}

void readOdometry(){
    measurements.class_list.clear();
    measurements.range_list.clear();
    measurements.theta_list.clear();
    
    // Read perception measurements from file
    readList(perceptionFile, measurements.class_list);
    readList(perceptionFile, measurements.range_list);
    readList(perceptionFile, measurements.theta_list);

    if(measurements.theta_list.size() != measurements.range_list.size()){
        cout<<" FUCKED UP TXT FORMAT!!!\n";
        return;
    }
}

void publishPoseMessage(const Eigen::VectorXd& state_vector, uint32_t globalIndexVelocity, const Eigen::VectorXd& velocity) {
    custom_msgs::msg::PoseMsg pose_msg;
    pose_msg.position.x = state_vector(0);
    pose_msg.position.y = state_vector(1);
    pose_msg.theta = state_vector(2);
    pose_msg.velocity_state.global_index = globalIndexVelocity;
    pose_msg.velocity_state.velocity_x = velocity(0);
    pose_msg.velocity_state.velocity_y = velocity(1);
    pose_msg.velocity_state.yaw_rate = velocity(2);
    pose_msg.lap_count = 0;

    pose_publisher_->publish(pose_msg);
}

void publishMapMessage(const Eigen::VectorXd& state_vector, uint32_t globalIndexPerception, const Eigen::VectorXd& velocity){
    // Publish map
	custom_msgs::msg::LocalMapMsg map_msg{};
	custom_msgs::msg::ConeStruct cone_msg{};

	map_msg.cones_count_actual = (state_vector.size()-3)/2;
	map_msg.pose.position.x = state_vector(0);
	map_msg.pose.position.y = state_vector(1);
	map_msg.pose.theta = state_vector(2);
	map_msg.pose.velocity_state.global_index = globalIndexPerception;
	map_msg.pose.velocity_state.velocity_x = velocity(0);
	map_msg.pose.velocity_state.velocity_y = velocity(1);
	map_msg.pose.velocity_state.yaw_rate = velocity(2);
	map_msg.lap_count = 0;
	for (int i{ 0 }; i < 9;i++) { map_msg.pose.velocity_state.variance_matrix[i] = 0;}
        int j = 0;
		for (int i=3; i<state_vector.size(); i += 2)
		{   
			cone_msg.coords.x = state_vector[i];
			cone_msg.coords.y = state_vector[i+1];
            cone_msg.color = landmark_colors[j];
           	map_msg.local_map.push_back(cone_msg);
            ++j;
        }
        
        map_publisher_->publish(map_msg);
}

private:
    vector<int> landmark_colors;
    int landmark_cnt = 0;
    vector<double> test_val;

    struct Measurements {
        std::vector<int> class_list;
        std::vector<double> range_list;
        std::vector<double> theta_list;
    };

    Measurements measurements; 

    std::ifstream velocityFile;
    std::ifstream perceptionFile;
    std::ifstream trackFile;
    rclcpp::Publisher<custom_msgs::msg::LocalMapMsg>::SharedPtr map_publisher_;
    rclcpp::Publisher<custom_msgs::msg::PoseMsg>::SharedPtr pose_publisher_;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = make_shared<SlamNode>();
    node->runSlamAlgorithm();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
