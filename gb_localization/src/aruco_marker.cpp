#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

using namespace std;

class ArucoMarkerNode : public rclcpp::Node
{
public:
    ArucoMarkerNode() : Node("aruco_marker_node")
    {
        img_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("camera/color/image_raw/compressed", 10, bind(&FactorGraphNode::gpsCallback, this, placeholders::_1));
        yaw_service_ = this->create_service<your_package::srv::GetRelativeYaw>("aruco/get_yaw", std::bind(&ArucoMarkerNode::seviceCallback, this, std::placeholders::_1, std::placeholders::_2));
    }
}
private:
    deque<std::pair<cv::Vec3d, rclcpp::Time>> markerPositionQueue_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr markerPositionPublisher_;
    cv::Mat cameraMatrix_;
    cv::Mat distortionCoefficients_;
    double markerSizeInMeters_;
    rclcpp::Service<gb_localizateion::srv::GlobalYaw>::SharedPtr getRelativeYawService_;

    void ArucoMarkerNode::imgCallback( const sensor_msgs::msg::CompressedImage::SharedPtr Img)
    {
        cv::Mat bgrImage = decodeCompressedImageToMat(Img);
        if (bgrImage.empty()) {
            return;
        }

        auto markerPositionVectors = extractVector(bgrImage);
        if (markerPositionVectors.empty()) {
            return;
        }

        for (const auto& [vec, time] : markerPositionVectors) {
            markerPositionQueue_.emplace_back(vec, time);
        }

        if (!markerPositionQueue_.empty()) {
            const auto& [latestVec, stamp] = markerPositionQueue_.back();

            geometry_msgs::msg::Vector3Stamped msg;
            msg.header.stamp = stamp;
            msg.header.frame_id = "camera_link";

            msg.vector.x = latestVec[0];
            msg.vector.y = latestVec[1];
            msg.vector.z = latestVec[2];

            markerPositionPublisher_->publish(msg);
        }
    }

    void ArucoMarkerNode::seviceCallback(
    const shared_ptr<gb_localization::srv::GlobalYaw::Request> request,
    shared_ptr<gb_localization::srv::GlobalYaw::Response> response)
    {
        if (markerPositionQueue_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No ArUco marker data available to compute yaw.");
            response->yaw = std::numeric_limits<double>::quiet_NaN();  // 또는 -999 등 예외값 처리
            return;
        }

        // 가장 최근 마커 사용
        const auto& [latestVector, stamp] = markerPositionQueue_.back();
        double yaw = computeYaw(latestVector);

        response->yaw = yaw;
    }

    cv::Mat decodeImage(const sensor_msgs::msg::CompressedImage::SharedPtr& img)
    {
        cv::Mat compressedImg;
        cv::Mat decodedImg;
        
        compressed_img = cv::Mat(1, img->data.size(), CV8_UC1, const_cast<uchar*>(img->data.data()));
        decodedImg = cv::imdecode(encodedImageBuffer, cv::IMREAD_COLOR);
        
        return decodedImg;
    }

    vector<MarkerDirection> ArucoMarkerNode::extractVector(const cv::Mat& bgrImage)
    {
        std::vector<std::pair<cv::Vec3d, rclcpp::Time>> positionVectors;

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        std::vector<cv::Vec3d> rvecs, tvecs;

        auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        auto parameters = cv::aruco::DetectorParameters::create();

        cv::aruco::detectMarkers(bgrImage, dictionary, markerCorners, markerIds, parameters);

        if (markerIds.empty() || cameraMatrix_.empty() || distortionCoefficients_.empty()) {
            return positionVectors;
        }

        cv::aruco::estimatePoseSingleMarkers(
            markerCorners,
            markerSizeInMeters_,
            cameraMatrix_,
            distortionCoefficients_,
            rvecs,
            tvecs
        );

        rclcpp::Time now = this->now();
        for (const auto& tvec : tvecs) {
            positionVectors.emplace_back(tvec, now);
        }

        return positionVectors;
    }

    double ArucoMarkerNode::computeYaw(const cv::Vec3d& directionVector)
    {
        double x = directionVector[0];
        double z = directionVector[2];  // OpenCV는 Z-forward, X-right, Y-down

        return std::atan2(x, z);  // 회전은 Y축 기준, rad 단위
    }
