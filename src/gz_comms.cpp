#include "gz_comms.hh"

gazebo::msgs::IMU IMU_data;
gazebo::msgs::PosesStamped pose_data;
gazebo::msgs::Diagnostics diagnostics_data;
gazebo::msgs::LinkData body_link_data;
gazebo::math::Quaternion body_orientation;
gazebo::math::Vector3 body_location;

////////////////////////////////////////////////////////////////
// Gazebo Message/Transport vars

gazebo::transport::NodePtr node(new gazebo::transport::Node());
gazebo::transport::SubscriberPtr sub_IMU;
gazebo::transport::SubscriberPtr sub_Poses;
gazebo::transport::SubscriberPtr sub_Diagnostics;
gazebo::transport::SubscriberPtr sub_Body_Link;

//
///////////////////////////////////////////

static gazebo::transport::PublisherPtr pub_JointCmd;
static gazebo::transport::PublisherPtr pub_TrackBallPos;
static int comms_mutex = 0;
static std::string mav_name;

void update_IMU_data(ConstIMUPtr &_msg) 
{
  if(comms_mutex == 0)
  {
    IMU_data = *_msg;
  }
}

void poses_callback(ConstPosesStampedPtr &_msg)
{
  static int pose_init = 0;
  static int pose_index;
  int i;
  
  if(pose_init == 0)
  {
    for(i = 0; i < _msg->pose_size(); i++)
    {
      if(_msg->pose(i).name() == mav_name)
      {
	pose_index = i;
	break;
      }
    }
    pose_init = 1;
  }   
  if(comms_mutex == 0)
  {
    if(_msg->pose_size() != 0)
    {
      pose_data = *_msg;
      body_orientation = gazebo::msgs::Convert(pose_data.pose(pose_index).orientation()); // We're cheeting using actual orientaion
      body_location = gazebo::msgs::Convert(pose_data.pose(pose_index).position()); 
    }
    else
    {
      __asm__("int $3");
    }
  }

  
}

void diagnostics_callback(ConstDiagnosticsPtr &_msg)
{
  if(comms_mutex == 0)
  {
    diagnostics_data = *_msg;
  }
}

void body_link_callback(ConstLinkDataPtr &_msg)
{
  if(comms_mutex == 0)
  {
    body_link_data = *_msg;
  }
}

int gz_comms_init(std::string mav_name_in)
{
  std::string tmpStr;
  mav_name = mav_name_in;
  
  // Create our node for communication

  node->Init("gz_comms");
  
  /////////////////////////////////////////////////////////////////////////
  // Publish to a Gazebo topic
  tmpStr = "/gazebo/default/" + mav_name + "/joint_cmd";
//  pub_JointCmd = node->Advertise<gazebo::msgs::JointCmd>("/gazebo/default/quad_2292_1/joint_cmd");
  pub_JointCmd = node->Advertise<gazebo::msgs::JointCmd>(tmpStr.c_str());
  // Wait for a subscriber to connect
  pub_JointCmd->WaitForConnection();

  pub_TrackBallPos = node->Advertise<gazebo::msgs::Model>("/gazebo/default/model/modify");
  
  
//  gazebo::msgs::Model
  
//  if(mav_name == 'quad_2292_1')
//  {
//    tmpStr = "/gazebo/default/model/modify";
//    pub_TrackBallPos = node->Advertise<gazebo::msgs::Model>(tmpStr.c_str());
//    // Wait for a subscriber to connect
//    pub_TrackBallPos->WaitForConnection();
//  }
  ////////////////////////////////////////////////////////////////////////////
  
  ///////////////////////////////////////////////////////////////////////
  // Subscribe to a Gazebo topic

//  sub_IMU 	  = node->Subscribe("/gazebo/default/quad_2292_1/body_1/imu_sensor/imu", update_IMU_data,true);
//  sub_Poses 	  = node->Subscribe("/gazebo/default/pose/info", poses_callback,true);
//  sub_Diagnostics = node->Subscribe("/gazebo/default/diagnostics", diagnostics_callback,true);
//  sub_Body_Link   = node->Subscribe("/gazebo/default/quad_2292_1::body_1",body_link_callback,true);

  tmpStr = "/gazebo/default/" + mav_name + "/body_1/imu_sensor/imu";
  sub_IMU 	  = node->Subscribe(tmpStr.c_str(), update_IMU_data,true);
  sub_Poses 	  = node->Subscribe("/gazebo/default/pose/info", poses_callback,true);
  sub_Diagnostics = node->Subscribe("/gazebo/default/diagnostics", diagnostics_callback,true);
  
  tmpStr = "/gazebo/default/" + mav_name + "::body_1";
  sub_Body_Link   = node->Subscribe(tmpStr.c_str(),body_link_callback,true);
  
  //////////////////////////////////////////////////////////////////////////////
  
  // need to add at least one empty pose
  pose_data.add_pose();
  
  return 0;
}

int gz_comms_end(void)
{
  gz_comms_lock_mutex();
  sub_IMU->Unsubscribe();
  sub_Poses->Unsubscribe();
  sub_Diagnostics->Unsubscribe();
  sub_Body_Link->Unsubscribe();
  pub_JointCmd->Fini();
  node->Fini();
  gz_comms_unlock_mutex();
  
  return 0;
}

int gz_comms_send_update(gazebo::math::Vector3 & desLoc, desCntrl_T & desCntrl,motInfo & mot)
{
  std::string tmpStr;
  static gazebo::msgs::JointCmd msg;	// joint messages to motors
  static gazebo::msgs::Model msg_TB;

  //static gazebo::msgs::Model    msg_TB; // Track Ball position message
  // static gazebo::msgs::Pose	pose_TB;// track ball position
  gazebo::math::Pose pose_TB;		// track ball pose
  gazebo::math::Quaternion ort_TB;	// track ball orientation
  gazebo::math::Vector3 eul_TB; 	// track ball orientation in euler angles [roll, pitch, yaw]
  gazebo::math::Vector3 loc_TB; 	// track ball location
//  gazebo::physics::WorldPtr cur_world_ptr;  
//  gazebo::physics::ModelPtr track_ball_ptr;
  
 
//  std::string _name;
//  _name = "quad_2292_test1";
//  gazebo::physics::worlds_running();
//  gazebo::physics::get_world();

/* 
  msg.set_name("quad_2292_1::rod_1_joint");
  msg.set_force(mot.mot1);
  pub_JointCmd->Publish(msg);
  
  msg.set_name("quad_2292_1::rod_2_joint");
  msg.set_force(mot.mot2);
  pub_JointCmd->Publish(msg);
  
  msg.set_name("quad_2292_1::rod_3_joint");
  msg.set_force(mot.mot3);
  pub_JointCmd->Publish(msg);
  
  msg.set_name("quad_2292_1::rod_4_joint");
  msg.set_force(mot.mot4);
  pub_JointCmd->Publish(msg);
*/

  tmpStr = mav_name + "::rod_1_joint";
  msg.set_name(tmpStr.c_str());
  msg.set_force(mot.mot1);
  pub_JointCmd->Publish(msg);
  
  tmpStr = mav_name + "::rod_2_joint";
  msg.set_name(tmpStr.c_str());
  msg.set_force(mot.mot2);
  pub_JointCmd->Publish(msg);
  
  tmpStr = mav_name + "::rod_3_joint";
  msg.set_name(tmpStr.c_str());
  msg.set_force(mot.mot3);
  pub_JointCmd->Publish(msg);
  
  tmpStr = mav_name + "::rod_4_joint";
  msg.set_name(tmpStr.c_str());
  msg.set_force(mot.mot4);
  pub_JointCmd->Publish(msg);
  
  if(mav_name == "quad_2292_1")
  { 
    eul_TB.x = desCntrl.roll;
    eul_TB.y = desCntrl.pitch;
    eul_TB.z = desCntrl.yaw;
    ort_TB.SetFromEuler(eul_TB);
    
    pose_TB.Set(desLoc, ort_TB);
    
    tmpStr = "track_ball";
    
    msg_TB.set_name("track_ball");
    gazebo::msgs::Set(msg_TB.mutable_pose(), pose_TB);
    pub_TrackBallPos->Publish(msg_TB);
    //msg_TB.set_name(tmpStr.c_str());
    //track_ball_ptr = gazebo::physics::World::GetModel(tmpStr);
    //cur_world_ptr->GetModel(tmpStr);
    //world_name = gazebo::physics::World::GetName();
    //track_ball_ptr->SetWorldPose(pose_TB);   
    
  }

  return 0;
}

int gz_comms_lock_mutex(void)
{
  comms_mutex = 1;
  return 0;
}

int gz_comms_unlock_mutex(void)
{
  comms_mutex = 0;
  return 0;
}