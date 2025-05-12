# HMI 모듈 주의사항 
------------------
혹시 ctrl + C 를 해도 API가 꺼지지 않고 살아있는 경우가 있으니 
끈 후에도 ps -e | grep node 로 아무것도 안뜨는 것을 확인하고 종료하기
------------------------------------------------------------
# requirements.txt 사용법 
requirements.txt 가 있는 디렉토리로 이동 후 
pip install -r requirements.txt 입력
------------------------------------------------------------
# HMI 모듈 처음 다운받으면 빌드하기 
1. 빌드 전 파이썬 가상환경(voice) 설정 및 pip requirement 다운로드 확인 
2. hmi_interface 패키지 먼저 빌드해야함. 
3. application 패키지 빌드해야함 
