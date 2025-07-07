# SLAM을 이용한 장애물 탐지 및 맵핑 시스템

이 프로젝트는 TurtleBot4 로봇을 활용하여 동적 환경의 지도를 자율주행으로 생성하는 SLAM(Simultaneous Localization and Mapping) 시스템입니다. ROS2 기반으로 다양한 센서 데이터를 통합하여, 로봇이 주행하면서 실시간으로 지도를 작성하고 위치를 추정할 수 있도록 설계되었습니다. 연구, 교육, 산업 현장 등 다양한 분야에서 활용할 수 있습니다.

## 주요 기능
### 1. 실시간 센서 데이터 수집 및 전처리
- Lidar, IMU, 카메라 등 다양한 센서로부터 데이터를 수집
- 노이즈 제거, 동기화, 좌표 변환 등 전처리 수행

### 2. SLAM 기반 맵 생성
- 로봇의 위치 추정(로컬라이제이션)과 맵 생성(맵핑) 동시 수행
- 실시간 맵 업데이트 및 누적

### 3. 자율 주행 및 경로 계획
- 미탐색 영역 자동 탐색(Frontier-based Exploration) 알고리즘
- 장애물 회피 및 최적 경로 생성
- ROS2 Navigation Stack 연동

### 4. 맵 저장, 불러오기 및 시각화
- 생성된 맵을 파일로 저장(pgm, yaml 등)
- RViz, OpenCV 등으로 실시간 시각화
- 저장된 맵을 활용한 재탐색 및 내비게이션

## 사용 기술
- 로봇 프레임워크: <img src="https://img.shields.io/badge/ROS2-22314E?style=for-the-badge&logo=ros&logoColor=white">
- 프로그래밍 언어: <img src="https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white">
- 맵 데이터 처리: <img src="https://img.shields.io/badge/Numpy-777BB4?style=for-the-badge&logo=numpy&logoColor=white">
- 시뮬레이션: <img src="https://img.shields.io/badge/RViz-22314E?style=for-the-badge&logo=ros&logoColor=white">

## 활용 분야
- 건축물 3D 스캐닝: 작업 중 인간의 개입 없이 대형 건축물 스캔이 가능
- 정찰 시스템: 미확인 지형을 자율 주행으로 정찰 및 변화 감지

## 향후 개선 방향
- 다양한 SLAM 알고리즘 지원: GMapping, Cartographer, Hector 등
- 다중 로봇 협업 맵핑: 여러 대의 로봇이 동시에 맵을 작성하고 통합
- 맵 품질 평가 및 자동 보정: 맵의 정확도 평가 및 자동 보정 기능 추가

## 발표 자료 주요 내용
![alt text](https://github.com/idingg/slam-mapping/blob/main/images/6.png?raw=true)
![alt text](https://github.com/idingg/slam-mapping/blob/main/images/1.png?raw=true)
![alt text](https://github.com/idingg/slam-mapping/blob/main/images/2.png?raw=true)
![alt text](https://github.com/idingg/slam-mapping/blob/main/images/3.png?raw=true)
![alt text](https://github.com/idingg/slam-mapping/blob/main/images/4.png?raw=true)
![alt text](https://github.com/idingg/slam-mapping/blob/main/images/5.png?raw=true)

## 매핑 영상
<video src="https://github.com/idingg/slam-mapping/raw/refs/heads/main/videos/video_rviz.webm" controls width="480"></video>
<video src="https://github.com/idingg/slam-mapping/raw/refs/heads/main/videos/video_real.mp4" controls width="480"></video>