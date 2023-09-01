# include "fastdash.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
#define DEBUG true
#define BRAKE "brake_report"
#define MOTEC "motec_report"
#define SUSP "suspension_report"
#define DASH "dash_report"
#define IMU "imu"
#define GPS "gps"
#define Tire_diameter 0.521
#define Primary_Gear_Ratio 2.073
#define Differential_Ratio 3.370


fastdash::fastdash(std::string can_socket): Node("datalogger"), stream(ios), signals(ios, SIGINT, SIGTERM)
{
    std::string s1 = "Using can socket " +  can_socket + "\n";
    RCLCPP_INFO(this->get_logger(), s1.c_str());
    
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    std::chrono::milliseconds pub_rate(100);
    this->timer_ = create_wall_timer(pub_rate, std::bind(&fastdash::publish_msg, this));
    data_collection_hyst = this->get_clock()->now().nanoseconds();
      
    rclcpp::executors::MultiThreadedExecutor exec;
    
    publisher_ = this->create_publisher<dash_msgs::msg::DashReport>(DASH, 1);
    
    strcpy(ifr.ifr_name, can_socket.c_str());
    ioctl(natsock, SIOCGIFINDEX, &ifr);
    
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if(bind(natsock,(struct sockaddr *)&addr,sizeof(addr))<0)
    {
        perror("Error in socket bind");
    }
    stream.assign(natsock);

    // initalize_topics();
    
    start_bag();
    
    // std::cout << "ROS2 to CAN-Bus topic:" << subscription_->get_topic_name() 	<< std::endl;
    // std::cout << "CAN-Bus to ROS2 topic:" << publisher_->get_topic_name() 	<< std::endl;
    
    stream.async_read_some(boost::asio::buffer(&rec_frame, sizeof(rec_frame)),std::bind(&fastdash::CanListener, this,std::ref(rec_frame),std::ref(stream)));
    
    signals.async_wait(std::bind(&fastdash::stop, this));
    
    boost::system::error_code ec;
    
    std::size_t (boost::asio::io_service::*run)() = &boost::asio::io_service::run;
    std::thread bt(std::bind(run, &ios));
    bt.detach();
}

void fastdash::stop()
{
    printf("\nEnd of Listener Thread. Please press strg+c again to stop the whole program.\n");
    stop_bag();
    ios.stop();
    signals.clear();
}

void fastdash::start_bag(){
    struct stat sb;
    int count = 1;
    std::string filename = "/home/va/bags/rosbag2_recording_000";
    char c_filename[sizeof(filename)];
    for(int i = 0; i < (int)sizeof(filename); i++){
        c_filename[i] = filename[i];
    }

    while (stat(c_filename, &sb) == 0){
        if(count < 10){
            c_filename[sizeof(c_filename) - 1] = '0' + count;
        }
        else{
            c_filename[sizeof(c_filename) - 2] = '0' + (count / 10);
            c_filename[sizeof(c_filename) - 1] = '0' + (count % 10);
        }
        count++;
    }
    std::filesystem::path s = c_filename;
    // Storage Options
    storage_options_.uri = s;
    storage_options_.storage_id = "sqlite3";
    // Converter Options
    converter_options_.input_serialization_format = "cdr";
    converter_options_.output_serialization_format = "cdr";

    writer_->open(storage_options_, converter_options_);
    writer_->create_topic({BRAKE, "dash_msgs/msg/BrakeReport", rmw_get_serialization_format(),""});
    writer_->create_topic({MOTEC, "dash_msgs/msg/MotecReport", rmw_get_serialization_format(),""});
    writer_->create_topic({SUSP, "dash_msgs/msg/SuspensionReport", rmw_get_serialization_format(),""});
    writer_->create_topic({IMU, "sensor_msgs/msg/Imu", rmw_get_serialization_format(),""});
    writer_->create_topic({GPS, "sensor_msgs/msg/NavSatFix", rmw_get_serialization_format(),""});
    // if(DEBUG){
        curr_bag_state = true;
        prev_bag_state = true;
    // }
    // std::string s1 = "Starting bag at " + filename + "\n";
    // RCLCPP_INFO(this->get_logger(), s1.c_str());
}

void fastdash::initialize_headers(){
    imu_msg.header.frame_id = "imu_link";
    imu_msg.header.stamp = this->get_clock()->now();

    gps_msg.header.frame_id = "imu_link";
    gps_msg.header.stamp = this->get_clock()->now();
}

void fastdash::stop_bag(){

    curr_bag_state = false;
    
    std::string s1 = "Stopping msg collection\n";
    RCLCPP_INFO(this->get_logger(), s1.c_str());
}

void fastdash::publish_msg(){
    dash_msg.oil_temp = motec_msg.oil_temp;
    dash_msg.engine_rpm = motec_msg.engine_rpm;
    dash_msg.coolant_temp = motec_msg.coolant_temp;
    dash_msg.oil_pressure = motec_msg.oil_pressure;
    dash_msg.gear = motec_msg.gear;
    dash_msg.wheel_speed = motec_msg.wheel_speed;
    dash_msg.fuel_pressure = motec_msg.fuel_pressure;
    dash_msg.throttle_pos = motec_msg.throttle_position;
    publisher_->publish(dash_msg);
}


fastdash::~fastdash(){printf("\nEnd of Publisher Thread. \n");}

void fastdash::CanListener(struct can_frame& rec_frame, boost::asio::posix::basic_stream_descriptor<>& stream)
{
    
    can_msgs::msg::Frame frame;
    
    frame.id = rec_frame.can_id; 
    frame.dlc = int(rec_frame.can_dlc);
    for(int i=0; i<rec_frame.can_dlc; i++)
    {
         frame.data[i]=rec_frame.data[i];
    }
    if(frame.id >= 0x100 && frame.id <= 0x103){
        // std::thread t1(&fastdash::log_motec, this, frame);
        // t1.join();
        log_motec(frame); // just moves data around for ease of reading
    }
    else if(frame.id >= 0x300 && frame.id <= 0x3FF){
        // std::thread t1(&fastdash::log_teensy, this, frame);
        // t1.join();
        log_imu(frame);
    }
    else if(frame.id >= 0x400 && frame.id <= 0x4D0){
        // std::thread t1(&fastdash::log_brake, this, frame);
        // t1.join();
        log_brake(frame);
    }
    
    stream.async_read_some(boost::asio::buffer(&rec_frame, sizeof(rec_frame)),std::bind(&fastdash::CanListener,this, std::ref(rec_frame),std::ref(stream)));
    
}

void fastdash::log_imu(can_msgs::msg::Frame frame){
    switch(frame.id){
        case(0x321):{
            imu_msg.linear_acceleration.x = (((((short)frame.data[1]) << 8) | frame.data[0])) / 100.0;
            imu_msg.linear_acceleration.y = (((((short)frame.data[3]) << 8) | frame.data[2])) / 100.0;
            imu_msg.linear_acceleration.z = (((((short)frame.data[5]) << 8) | frame.data[4])) / 100.0;
            break;
        }
        case(0x331):{
            imu_msg.orientation.x = (((((short)frame.data[1]) << 8) | frame.data[0])) * 3.05185094759972E-005;
            imu_msg.orientation.y = (((((short)frame.data[3]) << 8) | frame.data[2])) * 3.05185094759972E-005;
            imu_msg.orientation.z = (((((short)frame.data[5]) << 8) | frame.data[4])) * 3.05185094759972E-005;
            imu_msg.orientation.w = (((((short)frame.data[7]) << 8) | frame.data[6])) * 3.05185094759972E-005;
            break;
        }
        case(0x324):{
            imu_msg.angular_velocity.x = (((((short)frame.data[1]) << 8) | frame.data[0])) / 1000.0;
            imu_msg.angular_velocity.y = (((((short)frame.data[3]) << 8) | frame.data[2])) / 1000.0;
            imu_msg.angular_velocity.z = (((((short)frame.data[5]) << 8) | frame.data[4])) / 1000.0;
            break;
        }
        // case(0x339):{
        //     imu_msg.angular_velocity.x = (((((short)frame.data[1]) << 8) | frame.data[0])) / 100.0;
        //     imu_msg.angular_velocity.y = (((((short)frame.data[3]) << 8) | frame.data[2])) / 100.0;
        //     imu_msg.angular_velocity.z = (((((short)frame.data[5]) << 8) | frame.data[4])) / 100.0;
        //     break;
        // }
        case(0x375):{
            gps_msg.latitude = (((((int)frame.data[3]) << 24) | (frame.data[2] << 16) | (frame.data[1] << 8) | frame.data[0])) / 10000000.0;
            gps_msg.longitude = (((((int)frame.data[7]) << 24) | (frame.data[6] << 16) | (frame.data[5] << 8) | frame.data[4])) / 10000000.0;
            break; 
        }  
    }
    if(curr_bag_state){
        imu_msg.header.stamp = this->get_clock()->now();
        gps_msg.header.stamp = this->get_clock()->now();
        writer_->write(imu_msg, IMU, now());
        writer_->write(gps_msg, GPS, now());
    }
}

void fastdash::log_motec(can_msgs::msg::Frame frame){
    switch(frame.id){
        case(0x100):{
            motec_msg.battery_voltage = (((((short)frame.data[0]) << 8) | frame.data[1]) / 100.0); // this code turns the raw bytes into a short which is what we believe that messages come in as
            motec_msg.fuel_pressure = (((((short)frame.data[2]) << 8) | frame.data[3]) / 10.0);
            if(motec_msg.fuel_pressure > 1000){
                motec_msg.fuel_pressure = 0;
            }
            motec_msg.coolant_temp = (((((short)frame.data[4]) << 8) | frame.data[5]) / 10.0);
            motec_msg.oil_pressure = (((((short)frame.data[6]) << 8) | frame.data[7]) / 10.0);
            break;
        }
        case(0x101):{
            motec_msg.oil_temp = (((((short)frame.data[0]) << 8) | frame.data[1]) / 10.0);
            sus_msg.rear_left_linpot = (((((short)frame.data[2]) << 8) | frame.data[3]) / 10.0);
            sus_msg.rear_right_linpot = (((((short)frame.data[4]) << 8) | frame.data[5]) / 10.0);
            motec_msg.engine_rpm = (((((short)frame.data[6]) << 8) | frame.data[7]));
            // if(motec_msg.engine_rpm > 700 && prev_bag_state == false){
            //     if(data_collection_hyst != -1)
            //         data_collection_hyst = -1;
            //     prev_bag_state = true;
            //     curr_bag_state = true;
            // }
            // else if(motec_msg.engine_rpm < 400 && prev_bag_state == true){
            //     prev_bag_state = false;
            //     data_collection_hyst = this->get_clock()->now().nanoseconds(); // gets time that the engine rmp went below the limit
            // }
            // if((long int)(this->get_clock()->now().nanoseconds()) - data_collection_hyst > 1000000000 && curr_bag_state == true && prev_bag_state == false){
            //     stop_bag(); // stops recording when the engine rpm was too low for 10s long
            // }
            break;
        }
        case(0x102):{
            motec_msg.throttle_position = (((((short)frame.data[0]) << 8) | frame.data[1]) / 10.0);
            motec_msg.front_brake_pressure = (((((short)frame.data[2]) << 8) | frame.data[3]) / 10.0);
            motec_msg.rear_brake_pressure = (((((short)frame.data[4]) << 8) | frame.data[5]) / 10.0);
            motec_msg.ecu_temp = (((((short)frame.data[6]) << 8) | frame.data[7]) / 10.0);
            break;
        }
        case(0x103):{
            motec_msg.map_sensor = (((((short)frame.data[0]) << 8) | frame.data[1]) / 10.0);
            motec_msg.intake_air_temp = (((((short)frame.data[2]) << 8) | frame.data[3]) / 10.0);
            motec_msg.gear = (((((short)frame.data[4]) << 8) | frame.data[5]));
            
            motec_msg.wheel_speed = (((((short)frame.data[6]) << 8) | frame.data[7]) / 10.0);
            motec_msg.calculated_gear =  get_gear(motec_msg.wheel_speed, motec_msg.engine_rpm);
            break;
        }
    }
    if(curr_bag_state){
        writer_->write(motec_msg, MOTEC, now());
    }
}

void fastdash::log_brake(can_msgs::msg::Frame frame){
    switch(frame.id){
        case(0x4C4):{
            brake_msg.front_left[0] = (((((short)frame.data[0]) << 8) | frame.data[1]) / 10.0 - 100.);
            brake_msg.front_left[1] = (((((short)frame.data[2]) << 8) | frame.data[3]) / 10.0 - 100.);
            brake_msg.front_left[2] = (((((short)frame.data[4]) << 8) | frame.data[5]) / 10.0 - 100.);
            brake_msg.front_left[3] = (((((short)frame.data[6]) << 8) | frame.data[7]) / 10.0 - 100.);
            break;
        }
        case(0x4C5):{
            brake_msg.front_left[4] = (((((short)frame.data[0]) << 8) | frame.data[1]) / 10.0 - 100.);
            brake_msg.front_left[5] = (((((short)frame.data[2]) << 8) | frame.data[3]) / 10.0 - 100.);
            brake_msg.front_left[6] = (((((short)frame.data[4]) << 8) | frame.data[5]) / 10.0 - 100.);
            brake_msg.front_left[7] = (((((short)frame.data[6]) << 8) | frame.data[7]) / 10.0 - 100.);
            break;
        }
        case(0x4C6):{
            brake_msg.front_left[8] = (((((short)frame.data[0]) << 8) | frame.data[1]) / 10.0 - 100.);
            brake_msg.front_left[9] = (((((short)frame.data[2]) << 8) | frame.data[3]) / 10.0 - 100.);
            brake_msg.front_left[10] = (((((short)frame.data[4]) << 8) | frame.data[5]) / 10.0 - 100.);
            brake_msg.front_left[11] = (((((short)frame.data[6]) << 8) | frame.data[7]) / 10.0 - 100.);
            break;
        }
        case(0x4C7):{
            brake_msg.front_left[12] = (((((short)frame.data[0]) << 8) | frame.data[1]) / 10.0 - 100.);
            brake_msg.front_left[13] = (((((short)frame.data[2]) << 8) | frame.data[3]) / 10.0 - 100.);
            brake_msg.front_left[14] = (((((short)frame.data[4]) << 8) | frame.data[5]) / 10.0 - 100.);
            brake_msg.front_left[15] = (((((short)frame.data[6]) << 8) | frame.data[7]) / 10.0 - 100.);
            break;
        }
        case(0x4C8):{
            brake_msg.front_left_sensor_temp = (((((short)frame.data[0]) << 8) | frame.data[1]));
            break;
        }
        case(0x4C9):{
            brake_msg.rear_left[0] = (((((short)frame.data[0]) << 8) | frame.data[1]) / 10.0 - 100.);
            brake_msg.rear_left[1] = (((((short)frame.data[2]) << 8) | frame.data[3]) / 10.0 - 100.);
            brake_msg.rear_left[2] = (((((short)frame.data[4]) << 8) | frame.data[5]) / 10.0 - 100.);
            brake_msg.rear_left[3] = (((((short)frame.data[6]) << 8) | frame.data[7]) / 10.0 - 100.);
            break;
        }
        case(0x4CA):{
            brake_msg.rear_left[4] = (((((short)frame.data[0]) << 8) | frame.data[1]) / 10.0 - 100.);
            brake_msg.rear_left[5] = (((((short)frame.data[2]) << 8) | frame.data[3]) / 10.0 - 100.);
            brake_msg.rear_left[6] = (((((short)frame.data[4]) << 8) | frame.data[5]) / 10.0 - 100.);
            brake_msg.rear_left[7] = (((((short)frame.data[6]) << 8) | frame.data[7]) / 10.0 - 100.);
            break;
        }
        case(0x4CB):{
            brake_msg.rear_left[8] = (((((short)frame.data[0]) << 8) | frame.data[1]) / 10.0 - 100.);
            brake_msg.rear_left[9] = (((((short)frame.data[2]) << 8) | frame.data[3]) / 10.0 - 100.);
            brake_msg.rear_left[10] = (((((short)frame.data[4]) << 8) | frame.data[5]) / 10.0 - 100.);
            brake_msg.rear_left[11] = (((((short)frame.data[6]) << 8) | frame.data[7]) / 10.0 - 100.);
            break;
        }
        case(0x4CC):{
            brake_msg.rear_left[12] = (((((short)frame.data[0]) << 8) | frame.data[1]) / 10.0 - 100.);
            brake_msg.rear_left[13] = (((((short)frame.data[2]) << 8) | frame.data[3]) / 10.0 - 100.);
            brake_msg.rear_left[14] = (((((short)frame.data[4]) << 8) | frame.data[5]) / 10.0 - 100.);
            brake_msg.rear_left[15] = (((((short)frame.data[6]) << 8) | frame.data[7]) / 10.0 - 100.);
            break;
        }
        case(0x4CD):{
            brake_msg.rear_left_sensor_temp = (((((short)frame.data[0]) << 8) | frame.data[1]));
            
            break;
        }
    }
    if(curr_bag_state){
        writer_->write(brake_msg, BRAKE, now());
    }
}



int fastdash::get_gear(float Mph, float RPM){
    float Gear_Trans_Ratio = 0.0;
    try{
        Gear_Trans_Ratio = (RPM * Tire_diameter * 6. * M_PI) / (Mph * Primary_Gear_Ratio * Differential_Ratio * 161.);
    }
    catch(int myNum){
        std::string s1 = "Error while trying to calc gear number\n";
        RCLCPP_INFO(this->get_logger(), s1.c_str());
        return 0;
    }
    if (Gear_Trans_Ratio < 2.000){ // >= 2.583?
        return 1;
    } else if (Gear_Trans_Ratio < 1.667){ // >= 2.000?
        return 2;
    } else if (Gear_Trans_Ratio < 1.444){ // >= 1.667?
        return 3;
    } else if (Gear_Trans_Ratio < 1.286){ // >= 1.444?
        return 4;
    } else if (Gear_Trans_Ratio < 1.150){ // >= 1.286?
        return 5;
    } else if (Gear_Trans_Ratio < 1.0){
        return 6;
    } else
        return 0;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fastdash>());
  rclcpp::shutdown();
  return 0;
}