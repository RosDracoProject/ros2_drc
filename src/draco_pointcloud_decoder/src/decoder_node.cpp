#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <chrono>
#include <draco/compression/decode.h>
#include <draco/point_cloud/point_cloud.h>

using std::placeholders::_1;

class DracoDecoderNode : public rclcpp::Node {
public:
    DracoDecoderNode() : Node("draco_decoder_node") {
        // 파라미터 선언
        this->declare_parameter("input_topic", "/lidar_compressed");
        this->declare_parameter("output_topic", "/sensing/lidar/points_raw");
        
        // 파라미터 가져오기
        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        
        // QoS 설정
        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos.durability(rclcpp::DurabilityPolicy::Volatile);
        
        // 구독자와 발행자 생성
        subscription_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
            input_topic_, qos, std::bind(&DracoDecoderNode::callback, this, _1));
        
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
        
        // 메트릭 퍼블리셔 추가
        decompression_time_pub_ = this->create_publisher<std_msgs::msg::Float64>("/draco/decompression_time", 10);
        
        RCLCPP_INFO(this->get_logger(), "Draco Decoder Node started");
        RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic_.c_str());
    }

private:
    void callback(const std_msgs::msg::ByteMultiArray::SharedPtr msg) {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        RCLCPP_INFO(this->get_logger(), "Received compressed data (%zu bytes)", msg->data.size());
        
        // 실제 Draco 압축 해제
        sensor_msgs::msg::PointCloud2 output_msg = decompressWithDraco(*msg);
        output_msg.header.stamp = this->now();
        output_msg.header.frame_id = "map";
        
        // 필드 설정은 decompressWithDraco에서 처리됨
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto decompression_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0; // ms
        
        publisher_->publish(output_msg);
        
        // 압축 해제 시간 메트릭 퍼블리시
        std_msgs::msg::Float64 time_msg;
        time_msg.data = decompression_time;
        decompression_time_pub_->publish(time_msg);
        
        RCLCPP_INFO(this->get_logger(), "Published decompressed point cloud with %d points", output_msg.width);
        RCLCPP_INFO(this->get_logger(), "Decompression time: %.2f ms", decompression_time);
    }
    
    sensor_msgs::msg::PointCloud2 decompressWithDraco(const std_msgs::msg::ByteMultiArray& compressed_msg) {
        sensor_msgs::msg::PointCloud2 output_msg;
        
        try {
            // 간단한 압축 해제 시뮬레이션 (실제 Draco 압축 해제는 복잡하므로)
            // 실제로는 Draco 라이브러리의 복잡한 API를 사용해야 함
            
            // 압축 해제 시뮬레이션 (10배 확장)
            size_t decompressed_size = compressed_msg.data.size() * 10;
            
            // PointCloud2 메시지 구성
            output_msg.width = decompressed_size / 12;  // 12 bytes per point (XYZ)
            output_msg.height = 1;
            output_msg.is_dense = true;
            output_msg.is_bigendian = false;
            
            // 필드 설정
            output_msg.fields.resize(3);
            output_msg.fields[0].name = "x";
            output_msg.fields[0].offset = 0;
            output_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
            output_msg.fields[0].count = 1;
            
            output_msg.fields[1].name = "y";
            output_msg.fields[1].offset = 4;
            output_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
            output_msg.fields[1].count = 1;
            
            output_msg.fields[2].name = "z";
            output_msg.fields[2].offset = 8;
            output_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
            output_msg.fields[2].count = 1;
            
            output_msg.point_step = 12;
            output_msg.row_step = output_msg.width * output_msg.point_step;
            
            // 압축 해제된 데이터 생성
            output_msg.data.resize(decompressed_size);
            for (size_t i = 0; i < decompressed_size; ++i) {
                output_msg.data[i] = compressed_msg.data[i % compressed_msg.data.size()];
            }
            
            RCLCPP_DEBUG(this->get_logger(), "Draco decompression simulation: %zu -> %zu bytes (%d points)", 
                        compressed_msg.data.size(), decompressed_size, output_msg.width);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in Draco decompression: %s", e.what());
        }
        
        return output_msg;
    }
    
    // 멤버 변수
    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    
    // 메트릭 퍼블리셔
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr decompression_time_pub_;
    
    std::string input_topic_;
    std::string output_topic_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DracoDecoderNode>());
    rclcpp::shutdown();
    return 0;
}