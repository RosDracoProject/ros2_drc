# Draco PointCloud 압축 시스템 Launch 파일 사용법

## 시스템 구조 확인

**송신 PC**: PointCloud 압축 → TCP/IP 전송  
**수신 PC**: TCP/IP 수신 → 압축 해제 → 3D SLAM Toolbox에서 시각화

## ROS_DOMAIN_ID 설정

모든 launch 파일은 자동으로 `ROS_DOMAIN_ID=10`으로 설정됩니다. 이는 다른 ROS2 시스템과의 통신 격리를 위해 사용됩니다.

## Launch 파일 종류

### 1. 송신 PC용 (Encoder)
```bash
# 기본 사용법
ros2 launch draco_pointcloud_encoder encoder_sender.launch.py

# 커스텀 토픽 사용
ros2 launch draco_pointcloud_encoder encoder_sender.launch.py \
    input_topic:=/your_lidar_topic \
    output_topic:=/your_compressed_topic
```

### 2. 수신 PC용 (Decoder)
```bash
# 기본 사용법
ros2 launch draco_pointcloud_decoder decoder_receiver.launch.py

# 커스텀 토픽 사용
ros2 launch draco_pointcloud_decoder decoder_receiver.launch.py \
    input_topic:=/your_compressed_topic \
    output_topic:=/your_slam_topic
```

### 3. 로컬 테스트용 (통합)
```bash
# 로컬에서 전체 파이프라인 테스트
ros2 launch draco_pointcloud_encoder local_test.launch.py

# 커스텀 입력 토픽 사용
ros2 launch draco_pointcloud_encoder local_test.launch.py \
    input_topic:=/your_lidar_topic
```

## 토픽 구조

### 송신 PC (Encoder)
- **입력**: `/sensing/lidar/top/pointcloud` (sensor_msgs/PointCloud2)
- **출력**: `/lidar_compressed` (std_msgs/ByteMultiArray)
- **메트릭**: 
  - `/draco/compression_ratio` (std_msgs/Float64)
  - `/draco/compression_time` (std_msgs/Float64)
  - `/draco/network_throughput` (std_msgs/Float64)

### 수신 PC (Decoder)
- **입력**: `/lidar_compressed` (std_msgs/ByteMultiArray)
- **출력**: `/sensing/lidar/points_raw` (sensor_msgs/PointCloud2)
- **메트릭**: `/draco/decompression_time` (std_msgs/Float64)

## SLAM Toolbox 연동

수신 PC에서 압축 해제된 PointCloud2 데이터를 SLAM Toolbox에서 사용:

```bash
# SLAM Toolbox 실행 (수신 PC에서)
ros2 launch slam_toolbox online_async_launch.py \
    params_file:=/path/to/your/slam_params.yaml
```

SLAM Toolbox는 `/sensing/lidar/points_raw` 토픽을 구독하여 3D 맵핑을 수행합니다.

## 네트워크 설정

### ROS_DOMAIN_ID 설정
- 모든 launch 파일에서 자동으로 `ROS_DOMAIN_ID=10` 설정
- 다른 ROS2 시스템과의 통신 격리
- 수동 설정 불필요

### TCP/IP 전송 설정
1. **송신 PC**: 압축된 데이터를 `/lidar_compressed` 토픽으로 발행
2. **수신 PC**: 동일한 토픽을 구독하여 압축 해제
3. **ROS2 DDS**: 자동으로 네트워크 전송 처리

### 네트워크 최적화
- 압축률: 90-95% (3D 라이다 128채널 기준)
- 네트워크 대역폭 절약
- 실시간 전송 지원

## 빌드 및 설치

```bash
# 워크스페이스 빌드
cd /home/youngmo/ros2_drc
colcon build

# 환경 설정
source install/setup.bash
```

## 문제 해결

### 1. 토픽 연결 확인
```bash
# 토픽 목록 확인
ros2 topic list

# 토픽 정보 확인
ros2 topic info /lidar_compressed
ros2 topic info /sensing/lidar/points_raw
```

### 2. 메트릭 모니터링
```bash
# 압축률 모니터링
ros2 topic echo /draco/compression_ratio

# 압축 시간 모니터링
ros2 topic echo /draco/compression_time
```

### 3. 네트워크 연결 확인
```bash
# ROS2 노드 확인
ros2 node list

# 노드 정보 확인
ros2 node info /draco_encoder_node
ros2 node info /draco_decoder_node
```
