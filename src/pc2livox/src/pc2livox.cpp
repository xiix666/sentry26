#include <livox_ros_driver/CustomMsg.h>

void LivoxPointsPlugin::OnNewLaserScans()
if (rayShape) 
{
    std::vector<std::pair<int, AviaRotateInfo>> points_pair;
    InitializeRays(points_pair, rayShape);
    rayShape->Update();

    msgs::Set(laserMsg.mutable_time(), world->SimTime());
    msgs::LaserScan *scan = laserMsg.mutable_scan();
    InitializeScan(scan);

    SendRosTf(parentEntity->WorldPose(), world->Name(), raySensor->ParentName());

    auto rayCount = RayCount();
    auto verticalRayCount = VerticalRayCount();
    auto angle_min = AngleMin().Radian();
    auto angle_incre = AngleResolution();
    auto verticle_min = VerticalAngleMin().Radian();
    auto verticle_incre = VerticalAngleResolution();
    livox_ros_driver::CustomMsg pp_livox;//声明要发布的 custom格式的livox 点云
        // 赋值点云帧头
        pp_livox.header.stamp = ros::Time::now();//时间
        pp_livox.header.frame_id = "livox";//坐标系
        int count = 0;
        boost::chrono::high_resolution_clock::time_point start_time = boost::chrono::high_resolution_clock::now();
        for (auto &pair : points_pair)
        {
            auto range = rayShape->GetRange(pair.first);
            auto intensity = rayShape->GetRetro(pair.first);
            if (range >= RangeMax()) {
                range = 0;
            } else if (range <= RangeMin()) {
                range = 0;
            }
            auto rotate_info = pair.second;
            // 将极坐标转为x y  z 
            ignition::math::Quaterniond ray;
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            auto point = range * axis;
            livox_ros_driver::CustomPoint p;//声明一个 Custom 点
            p.x = point.X();
            p.y = point.Y();
            p.z = point.Z();
            p.reflectivity = intensity;
            boost::chrono::high_resolution_clock::time_point end_time = boost::chrono::high_resolution_clock::now();// 当前点时间
            boost::chrono::nanoseconds elapsed_time = boost::chrono::duration_cast<boost::chrono::nanoseconds>(end_time - start_time);
            p.offset_time = elapsed_time.count();
            pp_livox.points.push_back(p);
            // 计点个数的计数器加1
            count ++;
            }//结束遍历每个点
            pp_livox.point_num = count;
            livox_pub.publish(pp_livox);
            ros::spinOnce();
        }
    
