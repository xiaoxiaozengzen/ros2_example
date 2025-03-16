#include <memory>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/compressed_image.hpp"
#include "devastator_perception_msgs/msg/radar_object.hpp"
#include "devastator_perception_msgs/msg/radar_object_array.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/simple_filter.h"

namespace cgz {
    using namespace std::chrono_literals;

    using CompressedImage = sensor_msgs::msg::CompressedImage; 
    using RadarArrayMsg = devastator_perception_msgs::msg::RadarObjectArray;
    using RadarObject = devastator_perception_msgs::msg::RadarObject;

    using SyncPolicyT =
    message_filters::sync_policies::ApproximateTime<CompressedImage,
                                                    RadarArrayMsg>;

class MyFilter : public rclcpp::Node {
public:
    MyFilter(const rclcpp::NodeOptions& options):Node("my_filter_node", options) {
        input_obj_array_sub_ =
        std::make_unique<message_filters::Subscriber<CompressedImage>>(
            this, "/sensor/cam_front_30/compressed");
        
        input_radar_array_sub_ =
            std::make_unique<message_filters::Subscriber<RadarArrayMsg>>(
                this, "/sensor/radar0_object");
        input_radar_array_sub_->registerCallback(std::bind(&MyFilter::Callback, this, std::placeholders::_1));

        radar_simple_filter_ = std::make_unique<message_filters::SimpleFilter<RadarArrayMsg>>();
        radar_simple_filter_->registerCallback(std::bind(&MyFilter::CallbackFilter, this, std::placeholders::_1));

        SyncPolicyT sync_policy(10);
        sync_policy.setInterMessageLowerBound(0, 70ms);
        sync_policy.setInterMessageLowerBound(1, std::chrono::milliseconds(70));
        msg_sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicyT>>(
            SyncPolicyT(sync_policy), *input_obj_array_sub_,
            *input_radar_array_sub_);
        msg_sync_->registerCallback(std::bind(&MyFilter::CallbackSync, this,
                                            std::placeholders::_1,
                                            std::placeholders::_2));
        
        msg_time_sync_ = std::make_unique<message_filters::TimeSynchronizer<CompressedImage, RadarArrayMsg>>(*input_obj_array_sub_, *input_radar_array_sub_, 10);
        msg_sync_->registerCallback(std::bind(&MyFilter::CallbackTimeSync, this,
                                            std::placeholders::_1,
                                            std::placeholders::_2));
    }

    ~MyFilter() = default;

    void CallbackSync(const CompressedImage::ConstSharedPtr& input_objs, const RadarArrayMsg::ConstSharedPtr& radar_objs) {
        RCLCPP_INFO(this->get_logger(), "CompressedImage: %d.%09d, Radar: %d.%09d", input_objs->header.stamp.sec, input_objs->header.stamp.nanosec, radar_objs->header.stamp.sec, radar_objs->header.stamp.nanosec);
    }

    void Callback(const RadarArrayMsg::ConstSharedPtr& radar_objs) {
        RCLCPP_INFO(this->get_logger(), "Radar: %d.%09d, len = %d", radar_objs->header.stamp.sec, radar_objs->header.stamp.nanosec, radar_objs->objects.size());
    }

    void CallbackFilter(const RadarArrayMsg::ConstSharedPtr& radar_objs) {
        RCLCPP_INFO(this->get_logger(), "%s , Radar: %d.%09d, ", __FUNCTION__, radar_objs->header.stamp.sec, radar_objs->header.stamp.nanosec);
    }

    void CallbackTimeSync(const CompressedImage::ConstSharedPtr& input_objs, const RadarArrayMsg::ConstSharedPtr& radar_objs) {
        RCLCPP_INFO(this->get_logger(), "%s : CompressedImage: %d.%09d, Radar: %d.%09d", __FUNCTION__ , input_objs->header.stamp.sec, input_objs->header.stamp.nanosec, radar_objs->header.stamp.sec, radar_objs->header.stamp.nanosec);
    }
private:

    /** message_filters::Subscriber< M > */
    std::unique_ptr<message_filters::Subscriber<CompressedImage>> input_obj_array_sub_;
    std::unique_ptr<message_filters::Subscriber<RadarArrayMsg>> input_radar_array_sub_;

    /** Policy-Based Synchronizer */
    std::unique_ptr<message_filters::Synchronizer<SyncPolicyT>> msg_sync_;

    /** message_filters::SimpleFilter< M > */
    std::unique_ptr<message_filters::SimpleFilter<RadarArrayMsg>> radar_simple_filter_;

    /** message_filters::TimeSynchronizer */
    std::unique_ptr<message_filters::TimeSynchronizer<CompressedImage, RadarArrayMsg>> msg_time_sync_;
};
}

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(cgz::MyFilter)