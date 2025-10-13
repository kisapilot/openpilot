# kisa_agent.py

## 개요
`kisa_agent.py`는 OpenPilot 기반 환경에서 **커스텀 정책/제어 로직**을 적용하기 위한 에이전트 스크립트입니다. 시스템 상태 모니터링, 파라미터 관리, Git 업데이트 연동, 재부팅 트리거 등의 기능을 포함할 수 있습니다.

---

## 주요 기능
- OpenPilot 파라미터 읽기/쓰기
- 커스텀 정책에 따른 동작 제어
- Git 업데이트 감지 및 강제 동기화
- 시스템 재부팅 트리거
- 로그 출력 및 디버깅 지원
- JSON 스키마 기반 명령 및 파라미터 처리

---

## 실행 환경
- OS: Comma / OpenPilot 환경
- Python: 3.x
- 실행 위치:
  ```bash
  /data/openpilot/selfdrive/kisapilot
  ```

---

## 파일 구조
```text
selfdrive/kisapilot/
├─ kisa_agent.py
├─ cmd_schema.json
├─ param_schema.json
└─ README.md
```

---

## 스키마 파일
`selfdrive/kisapilot` 폴더에는 다음 JSON 스키마가 존재합니다:

- **cmd_schema.json**: 에이전트가 처리할 명령(command) 구조 정의
- **param_schema.json**: 파라미터(parameter) 구조 및 타입 정의

이 스키마를 기반으로 `kisa_agent.py`는 동적으로 명령을 처리하고 파라미터를 읽고 쓸 수 있습니다.

---

## 설치 및 준비
```bash
cd /data/openpilot/selfdrive/kisapilot
chmod +x kisa_agent.py
```

Python Flask 설치:
```bash
pip install flask
```

apk 파일 설치:
- apk파일을 받은 후 안드로이드 기기에 설치
---

## 실행 방법
```
안드로이드 기기와 오픈파일럿 장치는 동일네트워크에 있어야 함
```

```bash
python3 kisa_agent.py
```

백그라운드 실행 예시:
```bash
nohup python3 kisa_agent.py &
```

프로세스 등록 예시:
- PythonProcess("kisa_agent", "selfdrive.kisapilot.kisa_agent", always_run)
---


## 주요 로직 설명

### 1. 파라미터 처리
- OpenPilot `Params` 인터페이스 사용
- BOOL / INT / FLOAT 타입 자동 변환
- `param_schema.json`을 기반으로 동적 파라미터 관리

### 2. 명령 처리
- `cmd_schema.json`을 기반으로 명령 동적 처리
- Git 동기화, 재부팅 등 에이전트 동작 제어

### 3. Git 동기화
- 원격 저장소와 강제 동기화 가능
- Squash / 히스토리 불일치 대응

### 4. Tools
- 브랜치 변경
- 주행 모델 선택
- 파일 탐색기
- 주행 영상, 녹화영상 보기
- 화면녹화

## 로그 및 디버깅
- 핑거프린트 보기
- Tmux Log 실시간 확인
- 터미널 접속

## 라이선스
내부 사용 목적

