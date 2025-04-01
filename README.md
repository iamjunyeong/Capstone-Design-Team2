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

### 브랜치 예시

| 컴포넌트 | 브랜치명 예시 |
| --- | --- |
| application | `feature-application-haptic-feedback` |
| nav2 | `feature-nav2-behavior-tree` |
| hardware | `feature-hardware-control` |

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

### 3. GitHub Pull Request 생성

- `base`: `dev`
- `compare`: `feature/본인브랜치`
- 제목 예시: `Add path tracking module`
- 본문: 작업한 기능, 테스트 여부 등 간단한 설명

### 4. 코드 리뷰 및 머지

- 팀원 1인 이상 코드 리뷰
- 충돌 발생 시 직접 해결 후 다시 커밋
- 리뷰 완료 시 `dev` 브랜치에 병합

## 🔁 dev → main 병합

시연/배포 전에는 `dev` 브랜치를 `main`으로 병합합니다. (팀장이 할거임)

```bash
git checkout main
git pull origin main
git merge dev
git push origin main
```

## 🚫 브랜치 주의 사항
- `main`: 직접 푸시 ❌, PR만 가능 ✅
- `dev`: 리뷰 후 병합 ✅



## 🔁 자주 쓰는 Git 명령어 요약

```bash
git checkout -b feature/xxx        # 새 작업 브랜치
git add . && git commit -m "메시지" # 커밋
git push origin feature/xxx        # 푸시
git pull origin dev                # 최신 dev 반영
git merge origin/dev               # dev 머지
```

## 📬 문의

- 정준옹

