// spawn the blocks on the conveyor belt
// and give them initial speed (by apply_body_wrench) to slide on conveyor

//ros communications:
  // spawn model throught gazebo service: /gazebo/spawn_urdf_model
  // initialize blocks speed: /gazebo/apply_body_wrench
  // get urdf file path of blocks from parameter servicer
  //publish all current blocks through topic: /current_blocks

  #include <ros/ros.h>
  #include <iostream>
  #include <sstream>
  #include <fstream>
  #include <string>
  #include <urdf/model.h>
  #include <gazebo_msgs/SpawnModel.h>
  #include <gazebo_msgs/ApplyBodyWrench.h>
  #include <std_msgs/Int8MultiArray.h>
  #include <gazebo_msgs/SetModelState.h>

  //int to string converter
  std::string intToString (int a) {
     std::stringstream ss;
     ss << a;
     return ss.str();
  }

  int main(int argc, char **argv) {
      ros::init(argc, argv, "blocks_spawner");
      ros::NodeHandle nh;
      srand(time(0));
      //service client for service /gazebo/spawn_urdf_model
      ros::ServiceClient spawnClient = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
      gazebo_msgs::SpawnModel::Request spawn_model_req;
      gazebo_msgs::SpawnModel::Response spawn_model_resp;

      ros::ServiceClient wrenchClient = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
      gazebo_msgs::ApplyBodyWrench::Request apply_wrench_req;
      gazebo_msgs::ApplyBodyWrench::Response apply_wrench_resp;

      ros::ServiceClient setstateClient = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
      gazebo_msgs::ApplyBodyWrench::Request set_state_req;
      gazebo_msgs::ApplyBodyWrench::Response set_state_resp;

      //publisher for current_blocks
      ros::Publisher current_blocks_publisher = nh.advertise<std_msgs::Int8MultiArray>("current_blocks",1);
      std_msgs::Int8MultiArray current_blocks_msg;
      current_blocks_msg.data.clear();

      // make sure /gazebo/spawn_urdf_model service is service_ready
      bool service_ready = false;
      while (!service_ready){
        service_ready = ros::service::exists("/gazebo/spawn_urdf_model", true);
        ROS_INFO("waiting for spawn_urdf_model service");
        ros::Duration(0.5).sleep();
      }
      ROS_INFO("spawn_urdf_model service is ready");

      service_ready = false;
      while (!service_ready) {
          service_ready = ros::service::exists("/gazebo/apply_body_wrench", true);
          ROS_INFO("waiting for apply_body_wrench service");
          ros::Duration(0.5).sleep();
      }
      ROS_INFO("apply_body_wrench service is ready");

      service_ready = false;
      while (!service_ready) {
          service_ready = ros::service::exists("/gazebo/set_model_state", true);
          ROS_INFO("waiting for set_model_state service");
          ros::Duration(0.5).sleep();
      }
      ROS_INFO("set_model_state service is ready");

      //get file path of red_blocks from parameter service
      std::string red_box_path;
      bool get_redbox_path;
      get_redbox_path = nh.getParam("/red_box_path", red_box_path);


      if (!(get_redbox_path)){
          return 0;}
          else{ROS_INFO_STREAM(red_box_path << " has been extracted");
      }

      std::ifstream redbox_inXml(red_box_path.c_str());
      std::stringstream redbox_strStream;
      std::string redbox_xmlStr;

      /*red_inXml.open(red_box_path.c_str());*/
      redbox_strStream << redbox_inXml.rdbuf();
      redbox_xmlStr = redbox_strStream.str();
     // ROS_INFO_STREAM("urdf: \n" <<red_xmlStr);

      //get file path of red cylinders
      std::string red_cyl_path;
      bool get_redcyl_path;
      get_redcyl_path = nh.getParam("/red_cyl_path", red_cyl_path);

      if (!(get_redcyl_path)){
          return 0;}
          else{ROS_INFO_STREAM(red_cyl_path << " has been extracted");
      }

      std::ifstream redcyl_inXml(red_cyl_path.c_str());
      std::stringstream redcyl_strStream;
      std::string redcyl_xmlStr;

      redcyl_strStream << redcyl_inXml.rdbuf();
      redcyl_xmlStr = redcyl_strStream.str();

      //get file path of blue_blocks from parameter service
      std::string blue_box_path;
      bool get_bluebox_path;
      get_bluebox_path = nh.getParam("/blue_box_path", blue_box_path);


      if (!(get_bluebox_path)){
          return 0;}
          else{ROS_INFO_STREAM(blue_box_path << " has been extracted");
      }

      std::ifstream bluebox_inXml(blue_box_path.c_str());
      std::stringstream bluebox_strStream;
      std::string bluebox_xmlStr;

      bluebox_strStream << bluebox_inXml.rdbuf();
      bluebox_xmlStr = bluebox_strStream.str();

      //get file path of blue cylinders
      std::string blue_cyl_path;
      bool get_bluecyl_path;
      get_bluecyl_path = nh.getParam("/blue_cyl_path", blue_cyl_path);

      if (!(get_bluecyl_path)){
          return 0;}
          else{ROS_INFO_STREAM(blue_cyl_path << " has been extracted");
      }

      std::ifstream bluecyl_inXml(blue_cyl_path.c_str());
      std::stringstream bluecyl_strStream;
      std::string bluecyl_xmlStr;

      bluecyl_strStream << bluecyl_inXml.rdbuf();
      bluecyl_xmlStr = bluecyl_strStream.str();

      //get file path of yellow_blocks from parameter service
      std::string yellow_box_path;
      bool get_yellowbox_path;
      get_yellowbox_path = nh.getParam("/yellow_box_path", yellow_box_path);


      if (!(get_yellowbox_path)){
          return 0;}
          else{ROS_INFO_STREAM(yellow_box_path << " has been extracted");
      }

      std::ifstream yellowbox_inXml(yellow_box_path.c_str());
      std::stringstream yellowbox_strStream;
      std::string yellowbox_xmlStr;

      /*yellow_inXml.open(yellow_box_path.c_str());*/
      yellowbox_strStream << yellowbox_inXml.rdbuf();
      yellowbox_xmlStr = yellowbox_strStream.str();
     // ROS_INFO_STREAM("urdf: \n" <<yellow_xmlStr);

      //get file path of yellow cylinders
      std::string yellow_cyl_path;
      bool get_yellowcyl_path;
      get_yellowcyl_path = nh.getParam("/yellow_cyl_path", yellow_cyl_path);

      if (!(get_yellowcyl_path)){
          return 0;}
          else{ROS_INFO_STREAM(yellow_cyl_path << " has been extracted");
      }

      std::ifstream yellowcyl_inXml(yellow_cyl_path.c_str());
      std::stringstream yellowcyl_strStream;
      std::string yellowcyl_xmlStr;

      yellowcyl_strStream << yellowcyl_inXml.rdbuf();
      yellowcyl_xmlStr = yellowcyl_strStream.str();

      // prepare the pawn model service message
      spawn_model_req.initial_pose.position.x = 2;
      spawn_model_req.initial_pose.position.z = 0.2;
      spawn_model_req.initial_pose.orientation.x=0.0;
      spawn_model_req.initial_pose.orientation.y=0.0;
      spawn_model_req.initial_pose.orientation.z=0.0;
      spawn_model_req.initial_pose.orientation.w=1.0;
      spawn_model_req.reference_frame = "world";

      ros::Time time_temp(0, 0);
      ros::Duration duration_temp(0, 1000000);
      apply_wrench_req.wrench.force.x = -5.1;
      apply_wrench_req.wrench.force.y = 0.0;
      apply_wrench_req.wrench.force.z = 0.0;
      apply_wrench_req.start_time = time_temp;
      apply_wrench_req.duration = duration_temp;
      apply_wrench_req.reference_frame = "world";

      int i = 0;
     
     int block_choice; 

      while (ros::ok()){
          std::string index = intToString(i);
          std::string model_name;

          spawn_model_req.initial_pose.position.y = (float)rand()/(float)(RAND_MAX) * 0.4;  // random between -0.4 to 0.4
          ROS_INFO_STREAM("y position of new box: "
          << spawn_model_req.initial_pose.position.y);
         
          //randomly choose which block
          block_choice = rand() % 6;
          if (block_choice == 0) {
              model_name = "red_blocks_" + index;  // initialize model_name
              spawn_model_req.model_name = model_name;
              spawn_model_req.robot_namespace = model_name;
              spawn_model_req.model_xml = redbox_xmlStr;
          }
          else if(block_choice == 1) {
              model_name = "red_cylinder_" + index;  // initialize model_name
              spawn_model_req.model_name = model_name;
              spawn_model_req.robot_namespace = model_name;
              spawn_model_req.model_xml = redcyl_xmlStr;
          }
          else if (block_choice == 2) {
              model_name = "blue_blocks_" + index;  // initialize model_name
              spawn_model_req.model_name = model_name;
              spawn_model_req.robot_namespace = model_name;
              spawn_model_req.model_xml = bluebox_xmlStr;
          }
          else if(block_choice == 3) {
              model_name = "blue_cylinder_" + index;  // initialize model_name
              spawn_model_req.model_name = model_name;
              spawn_model_req.robot_namespace = model_name;
              spawn_model_req.model_xml = bluecyl_xmlStr;
          }
          else if (block_choice == 4) {
              model_name = "yellow_blocks_" + index;  // initialize model_name
              spawn_model_req.model_name = model_name;
              spawn_model_req.robot_namespace = model_name;
              spawn_model_req.model_xml = yellowbox_xmlStr;
          }
          else {
              model_name = "yellow_cylinder_" + index;  // initialize model_name
              spawn_model_req.model_name = model_name;
              spawn_model_req.robot_namespace = model_name;
              spawn_model_req.model_xml = yellowcyl_xmlStr;
          }

          bool call_service = spawnClient.call(spawn_model_req, spawn_model_resp);
          if (call_service) {
              if (spawn_model_resp.success) {
                  ROS_INFO_STREAM(model_name << " has been spawned");
              }
              else {
                  ROS_INFO_STREAM(model_name << " spawn failed");
              }
          }
          else {
              ROS_INFO("fail in first call");
              ROS_ERROR("fail to connect with gazebo server");
              return 0;
          }

          // prepare apply body wrench service message
          apply_wrench_req.body_name = model_name + "::base_link";

          // call apply body wrench service
          call_service = wrenchClient.call(apply_wrench_req, apply_wrench_resp);
          if (call_service) {
              if (apply_wrench_resp.success) {
                  ROS_INFO_STREAM(model_name << " speed initialized");
              }
              else {
                  ROS_INFO_STREAM(model_name << " fail to initialize speed");
              }
          }
          else {
              ROS_ERROR("fail to connect with gazebo server");
              return 0;
          }

          // publish current cylinder blocks status, all cylinder blocks will be published
          // no matter if it's successfully spawned, or successfully initialized in speed
          current_blocks_publisher.publish(current_blocks_msg);

          // loop end, increase index by 1, add blank line
          i = i + 1;
          ROS_INFO_STREAM("");

          ros::spinOnce();
          ros::Duration(20.0).sleep();  // frequency control, spawn one cylinder in each loop
          // delay time decides density of the cylinders


      }
      return 0;
  }
