# 객체 인식 2축 매니퓰레이터 공장 실내 청소 로봇
![IMG_2108](https://github.com/hallowmelting/ObjRecognizer-Manipulator-Robot/assets/40587712/9276569a-c92f-41af-a7cf-dd18a60347a7)

https://github.com/hallowmelting/ObjRecognizer-Manipulator-Robot/assets/40587712/3b018aae-b8d2-47fc-892a-4990d7aebf03

이 레포지토리는 ROS2 Foxy 기반 청소 로봇에 대한 모든 코드를 포함하고 있습니다. 이 소스코드는 Ubuntu 20.04가 설치된 Raspberry Pi4에서 작동합니다.

## 개요
![ros2 control 개요](https://github.com/hallowmelting/ObjRecognizer-Manipulator-Robot/assets/40587712/5785ed03-baf6-4d9a-ad5f-9d74604b11cc)
이 로봇은 실시간 객체 감지 기능과 2축 매니퓰레이터를 사용한 물체 집기 기능을 지원합니다. 카메라를 이용하여 주변의 쓰레기를 실시간으로 인식하고, 객체 인식에는 YOLOv8n 모델을 사용합니다. 이를 통해 높은 정밀도와 효율성으로 쓰레기를 줍게 되어 공장 실내 청소에 크게 기여합니다.

## 노드 구상도
![KakaoTalk_20231128_190947311](https://github.com/hallowmelting/ObjRecognizer-Manipulator-Robot/assets/40587712/dbaf137e-48c4-4e49-9993-1e9e9ac389b9)

## 주요 기능

- **객체 인식:** 카메라로부터 얻은 이미지 기반 데이터를 이용한 YOLOv8n 모델을 사용한 실시간 객체 감지 기능
- **몸체 조절:** YOLOv8n 모델에서 출력되는 좌표를 피드백으로 받아 실시간 모터 제어 
- **매니퓰레이터 기능:** 인식한 객체를 집어 올리는 2축 매니퓰레이터

## 시작하기
이 로봇을 사용하고 구조를 이해하려면 제공되는 문서를 따라 Raspberry Pi4에서 환경을 설정하면 됩니다. 문서에는 설치 방법, 사용법, 필요한 패키지 정보 등이 단계별로 안내되어 있습니다.

## 요구사항
이 로봇을 사용하기 위해 필요한 주요 사양과 패키지는 다음과 같습니다:

- **하드웨어:** Raspberry Pi4 model B 8GB
- **운영체제:** Ubuntu 20.04
- **프레임워크:** ROS2 Foxy
- **이미지 처리:** YOLOv8n model
- **카메라:** Raspberry Pi Camera Module 2
- **추가 패키지:** 
    - ros2_control
    - ros2_controllers
    - robot_state_publisher
    - teleop_twist_keyboard
    - twist_mux
    - cartographer
