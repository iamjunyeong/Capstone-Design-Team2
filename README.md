# 시각장애인 보행 보조 모빌리티: "동행" 프로젝트

## 💡 프로젝트 개요

본 프로젝트는 시각 장애인 보행 보조 모빌리티 시스템을 ROS2 기반으로 개발하는 프로젝트입니다.  
프로젝트는 크게 3개의 컴포넌트(application, nav2_bringup, hardware_bringup)로 나누어 개발되며, 각 팀은 해당 팀이 속한 컴포넌트"만" 담당합니다.  

ex. **HMI** - application  
ex. **Planning** - nav2_bringup  
ex. **Vision, Hardware** - hardware_bringup   
ex. **Localization** - nav2_bringup, hardware_bringup  

---

## 🗂️ 프로젝트 디렉토리 구조

```
Capstone-Design-Team2/
├── application/             # HMI 및 사용자 인터페이스
├── nav2_bringup/            # Nav2 기반 경로 계획 및 행동
├── hardware_bringup/        # 전체 하드웨어 런치 구성
├── sensor_bringup/          # 각 센서 런치 구성 (하드웨어에서 호출)
├── sensor_drivers/          # 여러 센서 드라이버 ROS2 패키지 모음
├── .github/                 # GitHub 설정 (PR 템플릿 등)
└── README.md
```

---

## 🌳 Git 브랜치 전략

### 주요 브랜치

- `main`: 배포 및 시연용 가장 안정된 브랜치 (직접 수정 ❌)
- `dev`: 전체 통합 개발 브랜치
- `feature-*`: 각 기능 또는 컴포넌트 단위 개발 브랜치

### 브랜치 생성 규칙

```
feature-[컴포넌트]-[기능]
```

### 예시

- `feature-hardware-sensor-bringup`
- `feature-nav2-navigation`
- `feature-application-hmi`

---

## 👥 협업 흐름 예시

### 1. 브랜치 생성

```bash
git checkout dev
git pull origin dev
git checkout -b feature-hardware-sensor-bringup
```

### 2. 커밋 & 푸시

```bash
git add .
git commit -m "Add IMU launch to sensor_bringup"
git push origin feature-hardware-sensor-bringup
```

### 3. PR 생성

- GitHub에서 `feature-* → dev`로 Pull Request 생성
- 코드 리뷰 후 병합

---

## 🔁 dev → main 병합

시연/배포 전에는 `dev` 브랜치를 `main`으로 병합합니다.

```bash
git checkout main
git pull origin main
git merge dev
git push origin main
```

---

## 🚫 브랜치 보호 권장 설정 (GitHub에서)

- `main`: 직접 푸시 ❌, PR만 가능 ✅
- `dev`: 리뷰 후 병합 ✅

---

## 📬 문의

- 정준옹

