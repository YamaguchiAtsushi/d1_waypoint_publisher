#include "d1_waypoint_publisher/d1_waypoint_publisher.hpp"  // ローカルヘッダーをインクルード

//int D1WaypointPublisher::state_ = NO_SEND;
int D1WaypointPublisher::state_ = APPROACH_WHITE_LINE;
//int D1WaypointPublisher::state_ = APPROACH_AREA; //ボックス認識せずにエリアに移動できるかだけを確認したいとき


D1WaypointPublisher::D1WaypointPublisher() : rclcpp::Node("d1_waypoint_manager"), id_(0), elapsed_time_(0)
{
    RCLCPP_INFO(this->get_logger(), "Shutting down after 5 seconds.");

    rclcpp::QoS latched_qos{1};
    latched_qos.transient_local();  // 遅延QoSの設定

    // Area のサブスクライバーを初期化//int D1WaypointPublisher::state_ = APPROACH_WHITE_LINE;

    state_and_feedback_sub_ = this->create_subscription<tsukutsuku2_msgs::msg::StateAndFeedback>(
        "state_and_feedback_topic", 10, 
        std::bind(&D1WaypointPublisher::StateAndFeedbackCallback, this, std::placeholders::_1)
    );

    d1_msg_sub_ = this->create_subscription<tsukutsuku2_msgs::msg::D1>(
        "d1_messages", 10, std::bind(&D1WaypointPublisher::D1MsgCallback, this, std::placeholders::_1));

    d1_waypoints_publisher_ = this->create_publisher<tsukutsuku2_msgs::msg::Waypoints>("d1_waypoints_topic", 10);

    // ウェイポイントのCSVファイルのパスを設定
    waypoints_file_1_ = "/home/yamaguchi-a/ros2_ws/src/d1_waypoint_publisher/csv/waypoints_1.csv";
    waypoints_file_2_ = "/home/yamaguchi-a/ros2_ws/src/d1_waypoint_publisher/csv/waypoints_2.csv";
    waypoints_file_3_ = "/home/yamaguchi-a/ros2_ws/src/d1_waypoint_publisher/csv/waypoints_3.csv";
    waypoints_file_4_ = "/home/yamaguchi-a/ros2_ws/src/d1_waypoint_publisher/csv/waypoints_a.csv";
    waypoints_file_5_ = "/home/yamaguchi-a/ros2_ws/src/d1_waypoint_publisher/csv/waypoints_b.csv";
    waypoints_file_6_ = "/home/yamaguchi-a/ros2_ws/src/d1_waypoint_publisher/csv/waypoints_c.csv";

    // CSVファイルのリストを初期化
    csv_file_ = {waypoints_file_1_, waypoints_file_2_, waypoints_file_3_, waypoints_file_4_, waypoints_file_5_, waypoints_file_6_};


    approach_white_line_flag = 0;
    approach_green_box_flag = 0;
    approach_blue_box_flag = 0;
    approach_goal_flag = 0;
    approach_area_flag = 0;

    // ロボットの現在位置と向きを初期化
    current_pose_.position.x = 0.0;
    current_pose_.position.y = 0.0;
    current_yaw_ = 0.0;

    // タイマーの初期化 (100msごとにコールバック関数が呼ばれる)
    timer_ = create_wall_timer(100ms, std::bind(&D1WaypointPublisher::SendWaypointsTimerCallback, this));

    // 経過時間をカウントするタイマー
    shutdown_timer_ = create_wall_timer(100ms, [this]() {
        elapsed_time_ += 100;  // 100msごとに経過時間を加算
        //RCLCPP_INFO(this->get_logger(), "Elapsed time: %d ms", elapsed_time_);

        // 5秒経過したらシャットダウン
        if (elapsed_time_ >= 50000) {
            //RCLCPP_INFO(this->get_logger(), "Shutting down after 5 seconds.");
            state_ = APPROACH_GOAL;
            //rclcpp::shutdown();
            //RCLCPP_INFO(this->get_logger(), "Shutdown complete.");
        }
    });
    // 最初のウェイポイントをCSVファイルから読み込み
    ReadWaypointsFromCSV(csv_file_[0], waypoints_);
    // std::cout << "waypoints_.size():" << waypoints_.size()<< std::endl;

    start_index_ = 1;  // 開始インデックスを設定
}

// yaw角をクォータニオンに変換
geometry_msgs::msg::Quaternion rpyYawToQuat(double yaw) {
    tf2::Quaternion tf_quat;
    geometry_msgs::msg::Quaternion msg_quat;
    tf_quat.setRPY(0.0, 0.0, yaw);
    msg_quat = tf2::toMsg(tf_quat);
    return msg_quat;
}

// CSVの行をパースしてベクターに格納
std::vector<std::string> D1WaypointPublisher::getCSVLine(std::string& input, char delimiter) {
    std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while (getline(stream, field, delimiter)) {
        result.push_back(field);
    }
    return result;
}

// CSVファイルからウェイポイントを読み込む
void D1WaypointPublisher::ReadWaypointsFromCSV(std::string& csv_file, std::vector<tsukutsuku2_msgs::msg::Waypoint>& waypoints_) {
    std::ifstream ifs(csv_file);
    if (!ifs) {
        std::cerr << "Error opening file: " << csv_file << std::endl;  // ファイルオープンエラーのチェック
        return;
    }

    std::string line;
    while (getline(ifs, line)) {
        std::cout << "Read line: " << line << std::endl;  // 読み取った行を表示

        if (line.empty()) continue;  // 空行をスキップ

        std::vector<std::string> strvec = getCSVLine(line, ',');
        if (strvec.size() < 3) {
            std::cerr << "Invalid line format: " << line << std::endl;  // フォーマットエラーのチェック
            continue;
        }
        
        tsukutsuku2_msgs::msg::Waypoint waypoint;
        waypoint.pose.position.x = std::stod(strvec.at(0));
        waypoint.pose.position.y = std::stod(strvec.at(1));
        waypoint.pose.position.z = 0.0;
        waypoint.pose.orientation = rpyYawToQuat(std::stod(strvec.at(2)) / 180.0 * M_PI);
        waypoints_.push_back(waypoint);
    }
}

// ウェイポイントを送信するタイマーコールバック
void D1WaypointPublisher::SendWaypointsTimerCallback() {
    static size_t sending_index = start_index_ - 1;
    //static int state_ = NO_SEND;
    //std::cout << "1" << std::endl;
    // std::cout << "waypoints_.size():" << waypoints_.size()<< std::endl;



    switch (state_) {
        case NO_SEND:
            std::cout << "NO_SEND state" << std::endl;
            break;

        case APPROACH_WHITE_LINE://approach white line

            // std::cout << "APPROACH_WHITE_LINE" << std::endl;

            if(sending_index < waypoints_.size() && approach_white_line_flag == 0){
                std::cout << "APPROACH_WHITE_LINE : sending waypoint" << std::endl;
                sending_index =  SendWaypointsOnce(sending_index);
                approach_white_line_flag = 1;
                std::cout << "3" << std::endl;

            }
            if(now_feedback_ == STANBY){
                std::cout << "now_feedback:" << now_feedback_<< std::endl;
                state_ = APPROACH_GREEN_BOX;
        }
            break;

        case APPROACH_GREEN_BOX://approach box
            // std::cout << "APPROACH_GREEN_BOX" << std::endl;
            if (approach_green_box_flag == 0) {
                ReadWaypointsFromCSV(csv_file_[1], waypoints_);
                //approach_green_box_flag = 1;
            }
            if (sending_index < waypoints_.size() && approach_green_box_flag == 0) {
                std::cout << "APPROACH_GREEN_BOX : sending waypoint" << std::endl;

                sending_index = SendWaypointsOnce(sending_index);
                approach_green_box_flag = 1;
            }
            // std::cout << "find_box" << find_box <<std::endl;


            
            if (now_feedback_ == STANBY) {
                if(find_box == true){
                    //find_box = false;
                    std::cout << "STANBY 状態に入りました。3秒停止します。" << std::endl;
                    std::this_thread::sleep_for(std::chrono::seconds(3));
                    // 3秒経ったら状態を APPROACH_BOX に変更
                    state_ = APPROACH_AREA;
                    std::cout << "3秒経過しました。BOX_CHECK 状態に遷移します。" << std::endl;
            }
            break;

        case APPROACH_AREA:
            // std::cout << "APPROACH_AREA" << std::endl;

            if (approach_area_flag == 0) {
                if (character == "a") {
                    ReadWaypointsFromCSV(csv_file_[3], waypoints_);
                } else if (character == "b") {
                    ReadWaypointsFromCSV(csv_file_[4], waypoints_);
                } else if (character == "c") {
                    ReadWaypointsFromCSV(csv_file_[5], waypoints_);
                }
                //approach_area_flag = 1;
            }
            if (sending_index < waypoints_.size() && approach_area_flag == 0) {
                std::cout << "APPROACH_AREA : sending waypoint" << std::endl;
                sending_index = SendWaypointsOnce(sending_index);
                approach_area_flag = 1;
            }
            if(find_box == true){
                find_box = false;
                state_ = APPROACH_BLUE_BOX;
            }
            break;

        case APPROACH_BLUE_BOX:
            // std::cout << "APPROACH_BLUE_BOX" << std::endl;

            if (approach_blue_box_flag == 0) {
                ReadWaypointsFromCSV(csv_file_[1], waypoints_);
                //approach_blue_box_flag = 1;
            }
            if (sending_index < waypoints_.size() && approach_blue_box_flag == 0) {
                std::cout << "APPROACH_BLUE_BOX : sending waypoint" << std::endl;
                sending_index = SendWaypointsOnce(sending_index);
                approach_blue_box_flag = 1;
            }
            if (now_feedback_ == STANBY) {
                if(find_box == true){
                    std::cout << "STANBY 状態に入りました。3秒停止します。" << std::endl;
                    std::this_thread::sleep_for(std::chrono::seconds(3)); //いらんかも
                    // 3秒経ったら状態を APPROACH_BOX に変更
                    state_ = APPROACH_GOAL;
                    std::cout << "3秒経過しました。BOX_CHECK 状態に遷移します。" << std::endl;
            }
            break;

        case APPROACH_GOAL:
            // std::cout << "APPROACH_GOAL" << std::endl;

            if (approach_goal_flag == 0) {
                ReadWaypointsFromCSV(csv_file_[1], waypoints_);
                // approach_goal_flag = 1;
            }
            if (sending_index < waypoints_.size() && approach_goal_flag == 0) {
                std::cout << "APPROACH_GOAL : sending waypoint" << std::endl;
                sending_index = SendWaypointsOnce(sending_index);
                approach_goal_flag = 1;

            }
            break;


        default:
            RCLCPP_INFO(this->get_logger(), "UNKNOWN ERROR");
            timer_->cancel();
            break;
    }
            }
    }
}

// 一度に1つのウェイポイントを送信
size_t D1WaypointPublisher::SendWaypointsOnce(size_t sending_index) {
    std::cout << "sendwaypointsonece1" << std::endl;
    //std::cout << "waypoints_.size():" << waypoints_.size()<< std::endl;

    if (sending_index < waypoints_.size()) {
        // std::cout << "sendwaypointsonce2" << std::endl;

        tsukutsuku2_msgs::msg::Waypoints waypoints_msg;
        waypoints_msg.waypoints = {waypoints_[sending_index]};
        d1_waypoints_publisher_->publish(waypoints_msg);
    }
    return sending_index + 1;
}


void D1WaypointPublisher::StateAndFeedbackCallback(const tsukutsuku2_msgs::msg::StateAndFeedback::SharedPtr msg) {
    //RCLCPP_INFO(this->get_logger(), "Received state: %d, feedback: %s", msg->state, msg->feed_back.c_str());
    // RCLCPP_INFO(this->get_logger(), "State: %d, feedback: %d", msg->state, msg->feed_back);
    next_state_ = msg->state;
    now_feedback_ = msg->feed_back;
    // if(next_state_ == 1){
    //     state_ = APPROACH_WHITE_LINE;
    // }
}



void D1WaypointPublisher::D1MsgCallback(const tsukutsuku2_msgs::msg::D1::SharedPtr msg)
{
    character = msg->character.c_str(),
    find_box = msg->find_box ? "true" : "false",
    find_character = msg->find_character ? "true" : "false",
    pose.position.x = msg->pose.position.x,
    pose.position.y = msg->pose.position.y,
    pose.position.z = msg->pose.position.z;
    // ログに出力
    // RCLCPP_INFO(this->get_logger(), 
    //             "Received D1 message: character=%s, find_box=%s, find_character=%s, position=(%f, %f, %f)",
    //             character, find_box, find_character, pose.position.x, pose.position.y = msg->pose.position.y, pose.position.z);

}