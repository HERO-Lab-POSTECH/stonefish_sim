# stonefish_msgs

Stonefish 해양 로봇 시뮬레이터를 위한 ROS 메시지 및 서비스 정의 통합 패키지입니다.

## Structure

```
stonefish_msgs/
├── stonefish_msgs/     # Stonefish 시뮬레이터 전용 메시지
│   ├── msg/           (9 messages)
│   └── srv/           (3 services)
└── cola2_msgs/         # COLA2 프로젝트 메시지
    ├── msg/           (30 messages)
    ├── srv/           (8 services)
    └── action/        (1 action)
```

## Messages

### stonefish_msgs

**Messages:**
- `BeaconInfo.msg` - USBL 비콘 정보
- `DVL.msg` - Doppler Velocity Log 데이터
- `DVLBeam.msg` - DVL 빔 데이터
- `Event.msg` - 이벤트 기반 카메라 단일 이벤트
- `EventArray.msg` - 이벤트 배열
- `INS.msg` - 관성 항법 시스템 데이터
- `Int32Stamped.msg` - 타임스탬프가 포함된 정수
- `NEDPose.msg` - North-East-Down 좌표계 포즈
- `ThrusterState.msg` - 추진기 상태

**Services:**
- `Respawn.srv` - 로봇 재배치
- `SonarSettings.srv` - 소나 설정 (FLS, SSS)
- `SonarSettings2.srv` - 소나 설정 (MSIS)

### cola2_msgs

COLA2 해양 로봇 제어 및 내비게이션을 위한 메시지 정의

## Dependencies

```xml
<depend>message_generation</depend>
<depend>message_runtime</depend>
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>
```

## Build

```bash
cd catkin_ws
catkin_make
```

## Version

1.0.0 (2025-10-17)
