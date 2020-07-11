#include "tamp_motion.hpp"


tamp_motion::tamp_motion(planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor):
planning_scene_monitor_(planning_scene_monitor)
,optsright_(RIGHT_PLANNING_GROUP, ROBOT_DESCRIPTION)
,optsleft_(LEFT_PLANNING_GROUP, ROBOT_DESCRIPTION){

    
    cout<<"tamp_motion::tamp_motion"<<endl;



	trajservice = node_handle.advertiseService("tamp_motion_service", &tamp_motion::motionQuery,this);
    ackservice = node_handle.advertiseService("tamp_ack_service",&tamp_motion::ackQuery,this);
  tampKnowledgeClient= node_handle.serviceClient<tamp_msgs::knowledge>("tamp_knowledge_service");



   if( !planning_scene_monitor_ )
  {
    loadPlanningSceneMonitor();
  }

  planning_scene_monitor_->startStateMonitor(JOINT_STATE_TOPIC);



  //moveit::planning_interface::MoveGroupInterface::Options optsright(RIGHT_PLANNING_GROUP, ROBOT_DESCRIPTION);
  //moveit::planning_interface::MoveGroupInterface::Options optsleft(LEFT_PLANNING_GROUP, ROBOT_DESCRIPTION);
  optsright_.robot_model_ = planning_scene_monitor_->getRobotModel();
  optsleft_.robot_model_ = planning_scene_monitor_->getRobotModel();
  move_group_left_.reset(new moveit::planning_interface::MoveGroupInterface(optsleft_));

 move_group_right_.reset(new moveit::planning_interface::MoveGroupInterface(optsright_));

    cout<<"*****move_group.getEndEffectorLink();"<<move_group_right_->getEndEffectorLink()<<endl;
    
    cout<<move_group_right_->getEndEffectorLink()<<"this was ee link"<<endl;
    cout<<"*****move_group.getEndEffectorLink();"<<move_group_left_->getEndEffectorLink()<<endl;
    cout<<move_group_left_->getEndEffectorLink()<<"this was ee link"<<endl;
  ROS_DEBUG_STREAM_NAMED("baxter_move","Baxter Move Group Interface loaded");
  nuofplanning_=0;
}
bool tamp_motion::ackQuery(tamp_msgs::ackquest::Request& request, tamp_msgs::ackquest::Response& response){
     //  ros::AsyncSpinner spinner(1);
 //spinner.start();
     ROS_INFO("requested ee pos for %s arm",request.arm.c_str());
     geometry_msgs::PoseStamped temp;

     temp = getEECurrentpos(request.arm.c_str());
     std::vector<double> rpy = getEECurrentRPY(request.arm.c_str());
     temp.pose.orientation.x = rpy[0];
     temp.pose.orientation.y = rpy[1];
     temp.pose.orientation.z = rpy[2];
     response.eepos = temp;
     return true;




}
bool tamp_motion::motionQuery(tamp_msgs::trajquest::Request& request, tamp_msgs::trajquest::Response& response){

  //ROS_INFO("Received motion planning request for %s arm with detail:",request.arm.c_str);
  //cout<<FBOLD()<<endl;
  cout<<"arm: "<<request.arm<<endl;
  cout<<"target position x y z: "<< request.targetpos.position.x <<" "<< request.targetpos.position.y <<" "<< request.targetpos.position.z <<endl;
  cout<<"target orientation Roll Pitch Yaw: "<< request.targetpos.orientation.x <<" "<< request.targetpos.orientation.y <<" "<< request.targetpos.orientation.z <<endl;
  cout <<"position and orientation tolerances: "<<request.position_tolerance.data<<" "<<request.orientation_tolerance.data<<endl;

 
 
 nuofplanning_++;

 double timenow = ros::Time::now().toSec();
 dominant_arm_ = request.arm;
 targetQuaternion_.setRPY(request.targetpos.orientation.x, request.targetpos.orientation.y, request.targetpos.orientation.z);
 request.targetpos.orientation = tf2::toMsg(targetQuaternion_);
 targetPositionTolerance_ = request.position_tolerance.data;
 targetOrientationTolerance_ = request.orientation_tolerance.data;
 targetPos_ = request.targetpos;
 toexecute_ = request.execute;
 addcollision_ = request.withcollision;
 if(addcollision_){

  addColision();
 }

 if(request.objecttoremve!=""){
    removeObject(request.objecttoremve);

 }
 //collisionObjects_ = request.collision_objects;
 //planning_scene_interface.addCollisionObjects(collisionObjects_);
 ros::AsyncSpinner spinner(1);
 spinner.start();
 if(dominant_arm_ == "right"){


   
    //move_group_right_.reset(new moveit::planning_interface::MoveGroupInterface(optsright_));

    move_group_right_->setMaxVelocityScalingFactor(0.7);
    move_group_right_->clearPoseTargets();

    move_group_right_->setNumPlanningAttempts(20);
    move_group_right_->setPlanningTime(2);


    move_group_right_->setGoalPositionTolerance(targetPositionTolerance_);
    move_group_right_->setGoalOrientationTolerance(targetOrientationTolerance_);
    move_group_right_->setPoseTarget(targetPos_);



    //moveit::planning_interface::MoveGroupInterface::Plan motion_plan_;

    bool motion_result_ = (move_group_right_->plan(motion_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    response.success = motion_result_;
    if(motion_result_){
        solvedTraj_ = motion_plan_.trajectory_;
        response.soltraj = solvedTraj_;
        planningtime_ = motion_plan_.planning_time_;
        response.time = planningtime_;
         moveit::planning_interface::MoveItErrorCode code;
         double timenow2 = ros::Time::now().toSec();
         double timepassed =timenow2-timenow;
         timespentforplanning_ +=timepassed;
         cout<<"overall motiom planning time is: "<<timespentforplanning_<<" Seconds"<<endl;
        if(toexecute_){
            double timenow3 = ros::Time::now().toSec();
           //code = move_group_right_->asyncExecute(motion_plan_);
           code = move_group_right_->execute(motion_plan_);
           double timenow4 = ros::Time::now().toSec();
         double timepassed2 =timenow4-timenow3;
         timespentforexecution_ +=timepassed2;
         cout<<"overall motiom execution time is: "<<timespentforexecution_<<" Seconds"<<endl;
         cout<<"There has been : "<<nuofplanning_<<" plannings"<<endl;

        }
        if(code==moveit::planning_interface::MoveItErrorCode::SUCCESS){

          response.executedtrajectory = true;
        }
        else{
          response.executedtrajectory = false;
        }


    }
 
    //moveit::planning_interface::MoveItErrorCode code = move_group_right_->asyncMove();
    // response.success = motion_result_;
   //  if(motion_result_){
   //     response.soltraj = solvedTraj_;
   //     response.time = planningtime_;
    //    move_group_right_->execute(motion_plan_);
     //}
 
      // printCurrentJointValues();
    // getCurrentPose();
 }
 else if(dominant_arm_ == "left") {
     
    move_group_left_->setMaxVelocityScalingFactor(0.7);
    move_group_left_->clearPoseTargets();
    move_group_left_->setNumPlanningAttempts(20);
    move_group_left_->setPlanningTime(2);
    move_group_left_->setGoalPositionTolerance(targetPositionTolerance_);
    move_group_left_->setGoalOrientationTolerance(targetOrientationTolerance_);
    move_group_left_->setPoseTarget(targetPos_);
    bool motion_result_ =  (move_group_left_->plan(motion_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    response.success = motion_result_;
    if(motion_result_){
        solvedTraj_ = motion_plan_.trajectory_;
        response.soltraj = solvedTraj_;
        planningtime_ = motion_plan_.planning_time_;
        response.time = planningtime_;
        moveit::planning_interface::MoveItErrorCode code;
        double timenow2 = ros::Time::now().toSec();
         double timepassed =timenow2-timenow;
         timespentforplanning_ +=timepassed;
         cout<<"overall motiom planning time is: "<<timespentforplanning_<<" Seconds"<<endl;
        if(toexecute_){
            double timenow3 = ros::Time::now().toSec();
           //code = move_group_left_->asyncExecute(motion_plan_);
           code = move_group_left_->execute(motion_plan_);
           double timenow4 = ros::Time::now().toSec();
         double timepassed2 =timenow4-timenow3;
         timespentforexecution_ +=timepassed2;
         cout<<"overall motiom execution time is: "<<timespentforexecution_<<" Seconds"<<endl;
        cout<<"There has been : "<<nuofplanning_<<" plannings"<<endl;


        }
          if(code==moveit::planning_interface::MoveItErrorCode::SUCCESS){

          response.executedtrajectory = true;
        }
        else{
          response.executedtrajectory = false;
        }

    }
    

    
    // printPosition(getEECurrentpos());
     //printRotation(getEECurrentRPY());
     
 }
 else{ROS_INFO("inserted arm %s is wrong!!!",dominant_arm_.c_str());}

 return true;
}

string tamp_motion::getEndEffectorLink(){
    if(dominant_arm_ == "right"){
        //move_group_right_.reset(new moveit::planning_interface::MoveGroupInterface(optsright_));
        EELink = move_group_right_->getEndEffectorLink();
    }
    else if (dominant_arm_ == "left"){

       EELink = move_group_left_->getEndEffectorLink();
    }
   
return EELink;
}

geometry_msgs::PoseStamped tamp_motion::getEECurrentpos(string arm){

//	 if( !planning_scene_monitor_ )
  //{
 //   loadPlanningSceneMonitor();
//  }

//  planning_scene_monitor_->startStateMonitor(JOINT_STATE_TOPIC);
  
     ros::AsyncSpinner spinner(1);
     spinner.start();
    if(arm == "right"){
        ROS_INFO("Getting right arm ee Pos");
        EECurrentpos_ = move_group_right_->getCurrentPose(move_group_right_->getEndEffectorLink());
        printPosition(EECurrentpos_);
        printRotation(getEECurrentRPY("right"));
    }
    else if (arm == "left"){
        ROS_INFO("Getting left arm ee Pos");
        EECurrentpos_ = move_group_left_->getCurrentPose(move_group_left_->getEndEffectorLink());
        printPosition(EECurrentpos_);
        printRotation(getEECurrentRPY("left"));
          
    }
return EECurrentpos_;
}


vector<double> tamp_motion::getEECurrentRPY(string arm){
    ros::AsyncSpinner spinner(1);
    spinner.start();
    if(arm == "right"){

        EECurrentRPY_ = move_group_right_->getCurrentRPY(move_group_right_->getEndEffectorLink());  
    }
    else if(arm == "left"){

         EECurrentRPY_ = move_group_left_->getCurrentRPY(move_group_left_->getEndEffectorLink());
    }


 return EECurrentRPY_;
}
void tamp_motion::setVisualTools(){

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("torso");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
  //  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    //text_pose.translation().z() = 1.75;
   // visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
  
}

void tamp_motion::printPosition(geometry_msgs::PoseStamped pos){

 std::cout<<"**********this is pos of end effector**********"<<std::endl;
 std::cout<<"x: "<<pos.pose.position.x<<std::endl;
 std::cout<<"y: "<<pos.pose.position.y<<std::endl;
 std::cout<<"z: "<<pos.pose.position.z<<std::endl;

}

void tamp_motion::printRotation(vector<double> v){

 std::cout<<"**********this is rotation of end effector**********"<<std::endl;
 std::cout<<"roll: "<<v.at(0)<<std::endl;
 std::cout<<"pitch: "<<v.at(1)<<std::endl;
 std::cout<<"yaw: "<<v.at(2)<<std::endl;

}


bool tamp_motion::loadPlanningSceneMonitor(){


  ROS_DEBUG_STREAM_NAMED("baxter_move","Loading planning scene monitor");

  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION));

  ros::spinOnce();
  ros::Duration(0.5).sleep(); // todo: reduce this time?
  ros::spinOnce();

  if (!planning_scene_monitor_->getPlanningScene())
  {
    ROS_ERROR_STREAM_NAMED("baxter_move","Planning scene not configured");
    return false;
  }

  return true;
}




bool tamp_motion::sendToPose(const geometry_msgs::PoseStamped& pose)
                                           
{
  if (pose.header.frame_id.empty())
  {
    ROS_ERROR_STREAM_NAMED("baxter_move","Frame ID was not set for requested pose");
    return false;
  }
         moveit::planning_interface::MoveGroupInterface::Options optsright(RIGHT_PLANNING_GROUP, ROBOT_DESCRIPTION);

    move_group_right_.reset(new moveit::planning_interface::MoveGroupInterface(optsright));

  move_group_right_->clearPoseTargets();

  move_group_right_->setPoseTarget(pose);
  move_group_right_->setNumPlanningAttempts(4);
  move_group_right_->setPlanningTime(20);
  move_group_right_->setGoalPositionTolerance(1e-3); // meters
  move_group_right_->setGoalOrientationTolerance(1e-2); // radians


   ros::AsyncSpinner spinner(1);
     spinner.start();
  
  motion_result_ =  (move_group_right_->plan(motion_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  std::cout << "motion_result_ " << motion_result_<< std::endl;

  //std::cout << "sending " << std::endl;
  //moveit::planning_interface::MoveItErrorCode code = move_group_right_->asyncMove();
  //std::cout << "sending finished" << std::endl;
  //ros::spinOnce();
  //ros::Duration(0.5).sleep();

  //return convertResult(code);
  return motion_result_;
}


bool tamp_motion::convertResult(moveit::planning_interface::MoveItErrorCode& code)
{
  switch (code.val)
  {
    case moveit_msgs::MoveItErrorCodes::SUCCESS:
      ROS_INFO_STREAM_NAMED("baxter_move","Planning and execution succeeded");
      return true;

    case moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN:
      ROS_ERROR_STREAM_NAMED("baxter_move","Failed because of invalid motion plan");
      return false;

   
  }

  ROS_ERROR_STREAM_NAMED("baxter_move","Planning and execution failed with code " << code.val);
  return false;

}



 geometry_msgs::Pose tamp_motion::getCurrentPose()
                                                          
{
   
  robot_state::RobotState state = planning_scene_monitor_->getPlanningScene()->getCurrentState();
    const double *p =  state.getJointPositions("right_e1");
    std::cout << "position e1 is"<<*p<<std::endl;
  state.updateLinkTransforms();
  //Eigen::Affine3d pose = state.getGlobalLinkTransform("right_hand");
   //Eigen::Isometry3d  pose= state.getGlobalLinkTransform("right_hand");
  //geometry_msgs::Pose pose_msg = visual_tools->convertPose(pose);

  ROS_INFO_STREAM_NAMED("baxter_move","pose is:");
  std::cout << "geometry_msgs::PoseStamped pose_msg;\n";
  /*
  std::cout << "pose_msg.pose.position.x = " << pose_msg.position.x << ";\n";
  std::cout << "pose_msg.pose.position.y = " << pose_msg.position.y << ";\n";
  std::cout << "pose_msg.pose.position.z = " << pose_msg.position.z << ";\n";
  std::cout << "pose_msg.pose.orientation.x = " << pose_msg.orientation.x << ";\n";
  std::cout << "pose_msg.pose.orientation.y = " << pose_msg.orientation.y << ";\n";
  std::cout << "pose_msg.pose.orientation.z = " << pose_msg.orientation.z << ";\n";
  std::cout << "pose_msg.pose.orientation.w = " << pose_msg.orientation.w << ";\n";
*/
  // Feedback
  //std::cout << "pose_msg = " << pose << ";\n";
  //visual_tools->publishArrow(pose_msg, rviz_visual_tools::RED, rviz_visual_tools::LARGE);
  geometry_msgs::Pose pose_msg;
  return pose_msg;
}


moveit::planning_interface::MoveItErrorCode tamp_motion::sendToPose(const std::string &pose_name)
{
  ROS_INFO_STREAM_NAMED("baxter_move_group_interface","Sending to pose '" << pose_name << "'");

  move_group_right_->setNamedTarget(pose_name);
  move_group_right_->setPlanningTime(15);
  moveit::planning_interface::MoveItErrorCode result = move_group_right_->move();

  if( !result )
    ROS_ERROR_STREAM_NAMED("baxter_move_group_interface","Failed to send Baxter to pose '" << pose_name << "'");

  return result;
}


void tamp_motion::printCurrentJointValues(){
    

    ros::AsyncSpinner spinner(1);
    spinner.start();


   // planning_scene_monitor_->updatesScene(planning_scene_monitor_->getPlanningScene());

    std::vector<double> joint_values;
    joint_values = move_group_right_->getCurrentJointValues();
    std::vector<std::string> joint_names = move_group_right_->getJointNames();
    std::cout<<"size of joint name is: "<<joint_names.size()<<std::endl;
    std::cout<<"size of joint value is: "<<joint_values.size()<<std::endl;
    // Output XML
    std::cout << "<group_state name=\"\" group=\"" << "dominant_arm_" << "\">\n";
    for (std::size_t i = 0; i < joint_values.size(); ++i)
    {
    std::cout << "  <joint name=\"" << joint_names[i] <<"\" value=\"" << joint_values[i] << "\" />\n";
    }
    std::cout << "</group_state>\n\n\n\n";







}




void tamp_motion::addColision(){

 ROS_INFO("Adding Colission Objects to Scene");

  if(!colisionobjectsids_.empty()){

    planning_scene_interface.removeCollisionObjects(colisionobjectsids_);
    colisionobjectsids_.clear();
    object_colors_.clear();
  }
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit_msgs::CollisionObject collision_object,collision_object2,colisionstorage;
  ///////////////////////////////////////////////////////////////Table
  collision_object.header.frame_id = move_group_right_->getPlanningFrame();

  collision_object.id = "table";//the table
  collision_object.header.frame_id="world";
  shape_msgs::SolidPrimitive primitivetable;
  primitivetable.type = primitivetable.BOX;
  primitivetable.dimensions.resize(3);
  primitivetable.dimensions[0] = 1;
  primitivetable.dimensions[1] = 2;
  primitivetable.dimensions[2] = 1;

  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x =  0.7;
  box_pose.position.y = 0.0;
  box_pose.position.z =  -0.65;

  collision_object.primitives.push_back(primitivetable);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

 std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);


moveit_msgs::ObjectColor bst;
bst.color.g=1.0f;
bst.color.b=0.0f;
bst.color.r=0.0f;
bst.color.a=1.0;
object_colors_.push_back(bst);

//////////////////////////////////////////////////////storage
colisionstorage.header.frame_id = move_group_right_->getPlanningFrame();

  colisionstorage.id = "storage";//the table
  colisionstorage.header.frame_id="world";
  shape_msgs::SolidPrimitive primitivestorage;
  primitivestorage.type = primitivestorage.BOX;
  primitivestorage.dimensions.resize(3);
  primitivestorage.dimensions[0] = 0.26;
  primitivestorage.dimensions[1] = 0.3;
  primitivestorage.dimensions[2] = 0.16;

  geometry_msgs::Pose storage_pose;
  storage_pose.orientation.w = 1.0;
  storage_pose.position.x =  0.76;
  storage_pose.position.y = -0.79;
  storage_pose.position.z =  -0.06;

  colisionstorage.primitives.push_back(primitivestorage);
  colisionstorage.primitive_poses.push_back(storage_pose);
  colisionstorage.operation = colisionstorage.ADD;

  collision_objects.push_back(colisionstorage);


moveit_msgs::ObjectColor b1;
b1.color.g=1.0f;
b1.color.b=0.0f;
b1.color.r=1.0f;
b1.color.a=1.0f;
object_colors_.push_back(b1);



////////////////////////////////////////////////////


std::vector<string> obstalsesname;
tamp_msgs::knowledge knmsg;
knmsg.request.reqType="num_of_objects";
int numofobjects;
if(tampKnowledgeClient.call(knmsg)){

   numofobjects = (int)knmsg.response.pose[0];
   obstalsesname = knmsg.response.names;
}
std::vector<std::vector<double>> obstaclespos;


for(std::size_t i=0;i<numofobjects;i++){
    tamp_msgs::knowledge knmsg;
    knmsg.request.reqType=obstalsesname[i]+"-grasp";
    ROS_INFO("Query for cylinders");
   if(tampKnowledgeClient.call(knmsg)){
   
     obstaclespos.push_back(knmsg.response.pose);
}


}

 


int j=0;
int k=0;
ROS_INFO("Query for cubes2");
for(std::size_t i=0;i<numofobjects;i++){
   std::vector<string> obstaclenametwo;
  boost::split(obstaclenametwo, obstalsesname[i], boost::is_any_of("_"));
   if(obstaclenametwo[0]=="cylinder"){
      j++;
      ROS_INFO("Query for cubes3");
      moveit_msgs::CollisionObject collision_cylinder;
      collision_cylinder.header.frame_id="world";
      std::vector<string> obstaclename;
      //boost::split(obstaclename, obstalsesname[i], boost::is_any_of("-"));
      collision_cylinder.id = obstalsesname[i];
      colisionobjectsids_.push_back(collision_cylinder.id);
      /* Define a box to add to the world. */
      shape_msgs::SolidPrimitive primitivecyl;
      primitivecyl.type = primitivecyl.CYLINDER;
      primitivecyl.dimensions.resize(2);
      primitivecyl.dimensions[0] = 0.23;
      primitivecyl.dimensions[1] = 0.02;
      //primitive.dimensions[2] = 1;
        ROS_INFO("Query for cubes5");
      /* A pose for the box (specified relative to frame_id) */
      box_pose.orientation.w = 1.0;
      box_pose.orientation.x =0.0;
      box_pose.orientation.y =0.0;
      box_pose.orientation.z =0.0;
      box_pose.position.x =  obstaclespos[i][0];
      box_pose.position.y = obstaclespos[i][1];
      box_pose.position.z = (primitivecyl.dimensions[0]/2)-0.14 ;

      collision_cylinder.primitives.push_back(primitivecyl);
      collision_cylinder.primitive_poses.push_back(box_pose);
      collision_cylinder.operation = collision_cylinder.ADD;
      collision_objects.push_back(collision_cylinder);
      moveit_msgs::ObjectColor b3;
      if(obstalsesname[i]=="cylinder_target"){
            b3.color.r=1.0f;
           b3.color.g=0.0f;
           b3.color.b=0.0f;
           b3.color.a=1.0f;
      }

      else{
           b3.color.r=0.0f;
           b3.color.g=0.0f;
           b3.color.b=1.0f;
           b3.color.a=1.0f;

      }
    
     object_colors_.push_back(b3);
          ROS_INFO("Query for cubes6");
   }
    if(obstaclenametwo[0]=="cube"){

       ROS_INFO("Query for cubes4");
       k++;
       moveit_msgs::CollisionObject collision_box;
      collision_box.header.frame_id="world";

      collision_box.id = obstalsesname[i];
      colisionobjectsids_.push_back(collision_box.id);
      
      /* Define a box to add to the world. */
      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[0] = 0.12;
      primitive.dimensions[1] = 0.24;
      primitive.dimensions[2] = 0.3;

      /* A pose for the box (specified relative to frame_id) */
      geometry_msgs::Pose boxpose;
      boxpose.orientation.w = 1.0;
      boxpose.position.x =  0.64; /*obstaclespos[i][0]*/;
      boxpose.position.y = -0.05; /*obstaclespos[i][1];*/;
      boxpose.position.z = (primitive.dimensions[2]/2)-0.15 ;

      collision_box.primitives.push_back(primitive);
      collision_box.primitive_poses.push_back(boxpose);
      collision_box.operation = collision_box.ADD;
      collision_objects.push_back(collision_box);
      moveit_msgs::ObjectColor b2;
     b2.color.b=0.8f;
     b2.color.g=0.0;
     b2.color.r=0.8f;
     b2.color.a=0.9f;
     object_colors_.push_back(b2);





//////////////bounding box


    /* 
    geometry_msgs::Pose box_pose2;
    collision_object2.id = "cube_"+to_string(k+1);
      colisionobjectsids_.push_back(collision_object2.id);
      
      shape_msgs::SolidPrimitive primitive2;
      primitive2.type = primitive2.BOX;
      primitive2.dimensions.resize(3);
      primitive2.dimensions[0] = fabs(boundboxvec[7]);
      primitive2.dimensions[1] = fabs(boundboxvec[8]);
      primitive2.dimensions[2] = fabs(boundboxvec[9]);

      box_pose2.orientation.x= boundboxvec[3];
      box_pose2.orientation.y = boundboxvec[4];
      box_pose2.orientation.z = boundboxvec[5];
      box_pose2.orientation.w = boundboxvec[6];
      box_pose2.position.x =  boundboxvec[0];
      box_pose2.position.y = boundboxvec[1];
      box_pose2.position.z = boundboxvec[2] ;

      collision_object2.primitives.push_back(primitive2);
      collision_object2.primitive_poses.push_back(box_pose2);
      collision_object2.operation = collision_object2.ADD;
      collision_objects.push_back(collision_object2);
      moveit_msgs::ObjectColor b3;
     b3.color.b=float(0.3);
     b3.color.g=0.3;
     b3.color.r=0.9;
     b3.color.a=1.0f;
     object_colors_.push_back(b3);

*/




   }
   




}





//moveit_msgs::ObjectColor b1,b2;
//b1.color.g=255;
//b1.color.a=0.9;
//b2.color.r=255;
//b2.color.a=0.9;
//object_colors.push_back(b1);
//object_colors.push_back(b2);



 

  ROS_INFO("Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects,object_colors_);

}

void tamp_motion::removeObject(string objid){

std::vector<std::string> object_ids;
object_ids.push_back(objid);
planning_scene_interface.removeCollisionObjects(object_ids);



}








tamp_motion::~tamp_motion(){

    cout<<"tamp_motion::~tamp_motion()"<<endl;
   cout<<"overall motiom planning time is: "<<timespentforplanning_<<" Seconds"<<endl;

   cout<<"overall motiom execution time is: "<<timespentforexecution_<<" Seconds"<<endl;
   cout<<"There have been overally "<<nuofplanning_<<" number of plannings"<<endl;

}