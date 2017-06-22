#include <mbzirc_task2_perception/roi_generator.h>

namespace mbzirc_task2_perception
{

  void ROIGenerator::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_ = advertise<sensor_msgs::CameraInfo>(*pnh_, "output_roi", 1);
    shaft_pub_ = advertise<sensor_msgs::CameraInfo>(*pnh_, "output_shaft_roi", 1);
    debug_pub_ = advertise<geometry_msgs::PoseArray>(*pnh_, "debug_output", 1);
    wrench_pub_ = advertise<geometry_msgs::PoseArray>(*pnh_, "wrench_pose", 1);
    //target_frame = "ground";
    target_frame = "r_shoulder_link";
  }

  void ROIGenerator::subscribe()
  {
    polygon_sub_ = pnh_->subscribe("input", 1, &ROIGenerator::polygonCb, this);
    info_sub_ = pnh_->subscribe("input/info", 1, &ROIGenerator::infoCb, this);
    ros::V_string names = boost::assign::list_of("~input")("~input/info");
    jsk_topic_tools::warnNoRemap(names);
  }

  void ROIGenerator::unsubscribe()
  {
    polygon_sub_.shutdown();
    info_sub_.shutdown();
  }

  geometry_msgs::Pose ROIGenerator::eigen2pose(Eigen::Vector3f vec){
    geometry_msgs::Pose pose;
    pose.position.x = vec.x();
    pose.position.y = vec.y();
    pose.position.z = vec.z();
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1.0;
    return pose;
  }

  geometry_msgs::Pose ROIGenerator::eigen2pose(Eigen::Vector3f vec, Eigen::Quaternionf q){
    geometry_msgs::Pose pose;
    pose.position.x = vec.x();
    pose.position.y = vec.y();
    pose.position.z = vec.z();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
  }

  Eigen::Vector3f ROIGenerator::pose2eigen(geometry_msgs::Pose pose){
    Eigen::Vector3f vec;
    vec.x() = pose.position.x;
    vec.y() = pose.position.y;
    vec.z() = pose.position.z;
    return vec;
  }

  void ROIGenerator::get_side(int start_id, std::vector<int>& side_id, std::vector<geometry_msgs::Pose>& corners){ // corners.size() have to be 8. side_id.at(0) will be right-lower corner and side_id.at(1) will be right-top.
    double max_z = 0.0;
    double z;
    int id, id1, id2;
    for(int i = -1; i < 2; i++){
      id = start_id + i;
      if(id < 0){
        id1 = 7;
        id2 = 0;
      } else if(id > 6){
        id1 = 7;
        id2 = 0;
      } else {
        id1 = id;
        id2 = id + 1;
      }
      z = corners.at(id1).position.z - corners.at(id2).position.z;
      if (fabs(z) >= fabs(max_z)){
        max_z = z;
        if(z >= 0.0){
          side_id.at(0) = id2;
          side_id.at(1) = id1;
        } else {
          side_id.at(0) = id1;
          side_id.at(1) = id2;
        }
      }
    }
  }

  sensor_msgs::CameraInfo ROIGenerator::computeROI(
                  const sensor_msgs::CameraInfo::ConstPtr& msg,
                  std::vector<cv::Point2d>& points)
  {
    double min_u, min_v, max_u, max_v;
    min_u = msg->width;
    min_v = msg->height;
    max_u = max_v = 0;
    for (size_t i = 0; i < points.size(); i++) {
      cv::Point2d uv(points[i]);
      if (uv.x < 0) {
        uv.x = 0;
      }
      if (uv.y < 0) {
        uv.y = 0;
      }
      if (uv.x > msg->width) {
        uv.x = msg->width;
      }

      if (uv.y > msg->height) {
        uv.y = msg->height;
      }
      if (min_u > uv.x) {
        min_u = uv.x;
      }
      if (max_u < uv.x) {
        max_u = uv.x;
      }
      if (min_v > uv.y) {
        min_v = uv.y;
      }

      if (max_v < uv.y) {
        max_v = uv.y;
      }
    }
    cv::Rect raw_roi(min_u, min_v, (max_u - min_u), (max_v - min_v));
    cv::Rect original(0, 0, msg->width, msg->height);
    cv::Rect roi_region = raw_roi & original;
    sensor_msgs::CameraInfo roi(*msg);
    roi.roi.x_offset = roi_region.x;
    roi.roi.y_offset = roi_region.y;
    roi.roi.width = roi_region.width;
    roi.roi.height = roi_region.height;
    roi.roi.do_rectify = true;
    return roi;
  }

  void ROIGenerator::infoCb(const sensor_msgs::CameraInfoConstPtr& msg)
  {
    if(vertices_pose.empty())
      return;
    image_geometry::PinholeCameraModel model;
    bool model_success_p = model.fromCameraInfo(msg);
    if (!model_success_p) {
      return;
    }
    geometry_msgs::PoseStamped vertex, transformed_vertex;
    std::vector<geometry_msgs::PoseStamped> transformed_vertices;
    int i;

    tf_listener_.waitForTransform(target_frame,
                                  msg->header.frame_id,
                                  msg->header.stamp,
                                  ros::Duration(1.0));

    for(i = 4; i < vertices_pose.size(); i++){
      vertex.header = msg->header;
      vertex.header.frame_id = target_frame;
      vertex.pose = vertices_pose.at(i);
      try{
        tf_listener_.transformPose(msg->header.frame_id, vertex, transformed_vertex);
      } catch(tf::TransformException ex){
        return;
      }
      transformed_vertices.push_back(transformed_vertex);
    }

    std::vector<cv::Point2d> local_points;
    for (i = 0; i < transformed_vertices.size(); i++) {
      //for z-optical axis
      cv::Point3d p(transformed_vertices[i].pose.position.x,
                    transformed_vertices[i].pose.position.y,
                    transformed_vertices[i].pose.position.z);

      //for x-optical axis
      // cv::Point3d p(- transformed_vertices[i].pose.position.y,
      //               - transformed_vertices[i].pose.position.z,
      //               transformed_vertices[i].pose.position.x);
      cv::Point2d uv = model.project3dToPixel(p);
      local_points.push_back(uv);
    }
    sensor_msgs::CameraInfo roi_info = computeROI(msg, local_points);
    pub_.publish(roi_info);
  }

  void ROIGenerator::polygonCb(const jsk_recognition_msgs::PolygonArrayConstPtr& msg)
  {
    std::vector<jsk_recognition_utils::Polygon::Ptr> polygons
      = jsk_recognition_utils::Polygon::fromROSMsg(*msg, Eigen::Affine3f::Identity());
    if(polygons.size() == 0) return;
    std::vector<jsk_recognition_utils::Segment::Ptr> edge = polygons[0]->edges();
    if(edge.size() == 0) return;

    int i;
    double angle_sum = 0;
    double angle;
    std::vector<geometry_msgs::Pose> corners;
    geometry_msgs::Pose corner;
    Eigen::Vector3f corner_pos;
    bool is_corner = false;
    bool find_corner = false;
    double corner_thre1 = 0.2;
    double corner_thre2 = M_PI / 2.0 - 0.15;
    geometry_msgs::PoseStamped input_pose, transformed_pose;

    for(i = 0; i < edge.size(); i++){
      if(i > 0){
        angle = edge.at(i)->angle(*edge[i - 1]);
      } else {
        angle = edge.at(0)->angle(*edge[edge.size() - 1]);
      }
      angle_sum += angle;
      std::cout << "angle : " << angle << "  sum : " << angle_sum << std::endl;

      if(!is_corner){
        if(angle > corner_thre1){
          is_corner = true;
          corner_pos = edge[i]->getOrigin();
          input_pose.header = msg->header;
          input_pose.pose = eigen2pose(corner_pos);
          try{
            tf_listener_.transformPose(target_frame, input_pose, transformed_pose);
          } catch(tf::TransformException ex){
            return;
          }
          corners.push_back(transformed_pose.pose);
          std::cout << "start corner pos : " << transformed_pose.pose.position.x << " "
                    << transformed_pose.pose.position.y << " " << transformed_pose.pose.position.z << std::endl;
        }
      }

      if(angle_sum > corner_thre2){
        find_corner = true;
        is_corner = false;
        corner_pos = edge[i]->getOrigin();
          input_pose.header = msg->header;
          input_pose.pose = eigen2pose(corner_pos);
          try{
            tf_listener_.transformPose(target_frame, input_pose, transformed_pose);
          } catch(tf::TransformException ex){
            return;
          }
        corners.push_back(transformed_pose.pose);
        std::cout << "end corner   pos : " << transformed_pose.pose.position.x << " "
                  << transformed_pose.pose.position.y << " " << transformed_pose.pose.position.z << std::endl;
        angle_sum = 0;
      }

      //edge[i]->print();
    }

    if(corners.size() != 8){
      std::cout << "found " << corners.size() << "corners" << std::endl;
      std::cout << "found corner num is wrong." << std::endl << std::endl;;
      return;
    }

    std::vector<geometry_msgs::Pose> sorted_corners;
    sorted_corners.resize(corners.size());

    //find corners
    std::vector<int> cand_id;
    cand_id.resize(2);
    double min_y1, min_y2;
    for(i = 0; i < corners.size(); i++){
      double y = corners.at(i).position.y;
      if(i == 0){
        min_y1 = y;
        cand_id.at(0) = i;
      } else if(i == 1){
        if(y <= min_y1){
          min_y2 = min_y1;
          min_y1 = y;
          cand_id.at(1) = cand_id.at(0);
          cand_id.at(0) = i;
        } else {
          min_y2 = y;
          cand_id.at(1) = i;
        }
      } else {
        if(y <= min_y1){
          min_y2 = min_y1;
          min_y1 = y;
          cand_id.at(1) = cand_id.at(0);
          cand_id.at(0) = i;
        } else if(y <= min_y2){
          min_y2 = y;
          cand_id.at(1) = i;
        }
      }
    }

    if(abs(cand_id.at(0) - cand_id.at(1)) != 1 &&
       abs(cand_id.at(0) - cand_id.at(1)) != corners.size() - 1){ //right-top corner candidates should adjoin.
      std::cout << "estimated corner is wrong." << std::endl;
      std::cout << "corner candidate 0 : " << cand_id.at(0) << std::endl;
      std::cout << "corner candidate 1 : " << cand_id.at(1) << std::endl;
      return;}
    if(cand_id.at(0) > cand_id.at(1))
      std::swap(cand_id.at(0), cand_id.at(1));

    //estimate sides
    // sort order :
    // right-bottom -> right(side)-top -> top-right -> top-left -> left-top -> left-bottom -> bottom-left -> bottom-right
    std::vector<int> rside_id;
    rside_id.resize(2);
    get_side(cand_id.at(0), rside_id, corners);
    int step = rside_id.at(1) - rside_id.at(0); //top - bottom
    int id;
    for(i = 0; i < corners.size(); i++){
      id = (rside_id.at(0) + step * i) % corners.size();
      sorted_corners.at(i) = corners.at(id);
    }
    std::vector<jsk_recognition_utils::Segment::Ptr> sides;
    sides.resize(4);
    for(i = 0; i < sides.size(); i++){
      jsk_recognition_utils::Segment::Ptr side (new jsk_recognition_utils::Segment(pose2eigen(sorted_corners.at(i * 2)), pose2eigen(sorted_corners.at(i * 2 + 1))));
      sides.at(i) = side;
    }

    std::cout << "====================================" << std::endl;
    //sleep(2);
    //debug output
    std::cout << "sides    : " << std::endl;
    for(i = 0; i < sides.size(); i++){
      Eigen::Vector3f debug_origin, debug_direction;
      debug_origin = sides.at(i)->getOrigin();
      debug_direction = sides.at(i)->getDirection();
      std::cout << "side " << i+1 << " - from: " << debug_origin.x() << " "
                << debug_origin.y() << " " << debug_origin.z() << " "
                << "  dir : " << debug_direction.x() << " "
                << debug_direction.y() << " " << debug_direction.z() << " " << std::endl;
    }

    std::cout << "vertices : " << std::endl;

    //estimate vetices
    std::vector<Eigen::Vector3f> vertices;
    Eigen::Vector3f commonperpend1, commonperpend2;
    Eigen::Vector3f vertex;
    vertices.resize(4);
    for(i = 0; i < vertices.size(); i++){
      std::cout << "angle : " << sides.at(i)->angle(*sides.at((i + 1) % vertices.size())) << "   ";
      if(std::isnan(sides.at(i)->angle(*sides.at((i + 1) % vertices.size())))){
        std::cout << "get NAN" << std::endl;
        std::cout << "====================================" << std::endl;
        return;
      }
      sides.at(i)->nearestPoints(*sides.at((i + 1) % vertices.size()), commonperpend1, commonperpend2);
      vertex = (commonperpend1 + commonperpend2) / 2.0f;
      vertices.at(i) = vertex;
      std::cout << vertex.x() << " "
                << vertex.y() << " " << vertex.z() << std::endl;
    }


    Eigen::Vector3f new_x = sides.at(1)->getDirection().cross(sides.at(0)->getDirection());
    Eigen::Matrix3f rot;
    rot.col(0) = new_x.normalized();
    rot.col(1) = - sides.at(1)->getDirection().normalized();
    rot.col(2) = - sides.at(0)->getDirection().normalized();
    std::cout << "rot matrix:" << std::endl << rot << std::endl;

    //not publish rotated coordinates
    Eigen::Vector3f rot_x;
    rot_x = rot.row(2).cross(Eigen::Vector3f::UnitZ());
    std::cout << "rotation around x : " << rot_x.x() << " " << rot_x.y() << " " << rot_x.z() << " norm : " << rot_x.norm() << std::endl;
    if(rot_x.norm() > 0.2){
        std::cout << "coordinates rotated!! stop calculation." << std::endl;
        std::cout << "====================================" << std::endl << std::endl;
      return;
    } else {
      std::cout << "====================================" << std::endl;
    }

    std::vector <Eigen::Vector3f> translation;
    translation.resize(4);

    //wrench roi parameters
    translation.at(0).x() = 0.0;
    translation.at(0).y() = -0.01; //0.02
    translation.at(0).z() = - 0.18;
    translation.at(1).x() = 0.0;
    translation.at(1).y() = 0.36; //0.33
    translation.at(1).z() = - 0.18;
    translation.at(2).x() = 0.0;
    translation.at(2).y() = 0.36; //0.02
    translation.at(2).z() = - 0.49; //-0.42
    translation.at(3).x() = 0.0;
    translation.at(3).y() = - 0.01; //0.33
    translation.at(3).z() = - 0.49; //-0.42

    std::vector <Eigen::Vector3f> roi_vertices;
    roi_vertices.resize(4);
    for(i = 0; i < roi_vertices.size(); i++){
      Eigen::Vector3f roi_vertex = rot * translation.at(i) + vertices.at(0);
      roi_vertices.at(i) = roi_vertex;
    }

    Eigen::Quaternionf rot_q(rot);
    rot_q.normalize();

    geometry_msgs::PoseArray pose_msg;
    pose_msg.header = msg->header;
    pose_msg.header.frame_id = target_frame;
    vertices_pose.clear();
    vertices_pose.push_back(eigen2pose(vertices.at(0), rot_q));
    vertices_pose.push_back(eigen2pose(vertices.at(1), rot_q));
    vertices_pose.push_back(eigen2pose(vertices.at(2), rot_q));
    vertices_pose.push_back(eigen2pose(vertices.at(3), rot_q));
    vertices_pose.push_back(eigen2pose(roi_vertices.at(0), rot_q));
    vertices_pose.push_back(eigen2pose(roi_vertices.at(1), rot_q));
    vertices_pose.push_back(eigen2pose(roi_vertices.at(2), rot_q));
    vertices_pose.push_back(eigen2pose(roi_vertices.at(3), rot_q));
    pose_msg.poses = vertices_pose;

    //valve stem
    Eigen::Vector3f valve_translation;
    valve_translation.x() = -0.07;
    valve_translation.y() = 0.65;
    valve_translation.z() = -0.37;

    Eigen::Vector3f valve_pos;
    valve_pos = rot * valve_translation + vertices.at(0);

    geometry_msgs::Pose valve_pose;
    valve_pose = eigen2pose(valve_pos, rot_q);

    tf::Transform valve_transform;
    tf::Quaternion rot_tf(rot_q.x(), rot_q.y(), rot_q.z(), rot_q.w());
    valve_transform.setOrigin(tf::Vector3(valve_pos.x(), valve_pos.y(), valve_pos.z()));
    valve_transform.setRotation(rot_tf);
    // br_.sendTransform(tf::StampedTransform(valve_transform, msg->header.stamp,
    //                                         target_frame,
    //                                         "valve"));
    br_.sendTransform(tf::StampedTransform(valve_transform, ros::Time::now(),
                                            target_frame,
                                            "valve"));

    std::vector <Eigen::Vector3f> wrench_translation;
    wrench_translation.resize(6);

    //wrench pos parameters
    wrench_translation.at(0).x() = -0.06;
    wrench_translation.at(0).y() = 0.30;
    wrench_translation.at(0).z() = - 0.15;
    wrench_translation.at(1).x() = -0.06;
    wrench_translation.at(1).y() = 0.25;
    wrench_translation.at(1).z() = - 0.15;
    wrench_translation.at(2).x() = -0.06;
    wrench_translation.at(2).y() = 0.20;
    wrench_translation.at(2).z() = - 0.15;
    wrench_translation.at(3).x() = -0.06;
    wrench_translation.at(3).y() = 0.15;
    wrench_translation.at(3).z() = - 0.15;
    wrench_translation.at(4).x() = -0.06;
    wrench_translation.at(4).y() = 0.10;
    wrench_translation.at(4).z() = - 0.15;
    wrench_translation.at(5).x() = -0.06;
    wrench_translation.at(5).y() = 0.05;
    wrench_translation.at(5).z() = - 0.15;


    std::vector <Eigen::Vector3f> wrench_poss;
    wrench_poss.resize(6);
    for(i = 0; i < wrench_poss.size(); i++){
      Eigen::Vector3f wrench_pos = rot * wrench_translation.at(i) + vertices.at(0);
      wrench_poss.at(i) = wrench_pos;
    }

    geometry_msgs::PoseArray wrench_msg;
    wrench_msg.header = msg->header;
    wrench_msg.header.frame_id = target_frame;
    std::vector<geometry_msgs::Pose> wrench_poses;
    wrench_poses.push_back(eigen2pose(wrench_poss.at(0), rot_q));
    wrench_poses.push_back(eigen2pose(wrench_poss.at(1), rot_q));
    wrench_poses.push_back(eigen2pose(wrench_poss.at(2), rot_q));
    wrench_poses.push_back(eigen2pose(wrench_poss.at(3), rot_q));
    wrench_poses.push_back(eigen2pose(wrench_poss.at(4), rot_q));
    wrench_poses.push_back(eigen2pose(wrench_poss.at(5), rot_q));
    wrench_msg.poses = wrench_poses;


    wrench_pub_.publish(wrench_msg);
    debug_pub_.publish(pose_msg);
    std::cout << std::endl;
  }

};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (mbzirc_task2_perception::ROIGenerator, nodelet::Nodelet);
