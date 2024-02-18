1. Prepare a device with Android version equal or upper than 6.0(marshmallow)
2. For rooted device, use com.kisapilot.connect(root).apk. Non-rooted device is com.kisapilot.connect(no_root).apk
3. Install kisa connect and Navi application on the device.
  3-1. Non-root device requires some prior procedures.
  3-2. Enable Android Debugging in Android developer mode.  * Use internet search for Android Developer menu
  3-3. Connect pc to android device using usb cable. Check a popup message and check a box on your phone at connected.
  3-4. Download adb and run a command on PC "adb shell pm grant com.kisapilot.connect android.permission.READ_LOGS" on command window.
       * To get adb, search "platform tools" on google. The file is inside a zip file.
4. Connect to same Wifi Network the Device and Openpilot Device.
5. Input Your Nav dev IP in UI Menu of Openpilot Settings.
6. Seperate with comma if your Dev IP changed few times when connected.(ex. 192.168.1-254.2,192.168.10.10)
   In most cases, the Dev ip is not changed.
7. If Wifi is working, Run kisa connect and Navi application. Tip. use Tasker App and make automated task.
8. Your OP device will check the connection every 60s.
9. Once connected, OP device stop checking the connection and receive the Navi data. If you see a green circle at the top right, it's connected.
10. Battery Optimization of Android should be off for kisa connect App to run as background. Don't use back button to exit.
11. Be safe and Enjoy your driving~

1. 안드로이드 6.0이상 여분의 디바이스를 준비하세요. 네비어플이 원할하게 깔리는 기기가 좋습니다.
2. 루팅된 장치라면, com.kisapilot.connect(root).apk를 사용하고 비루팅 기기는 com.kisapilot.connect(no_root).apk를 사용하세요.
3. 기사커넥트앱과 네비어플(수정된 어플)을 설치하세요.
  3-1. 비루팅 기기는 몇가지 작업이 필요합니다.
  3-2. 안드로이드 개발자 모드에서 안드로이드 디버깅을 활성화하세요. * 안드로이드 개발자모드 활성화는 인터넷 검색 바람
  3-3. PC와 안드로이드 기기를 USB로 연결하세요. 연결시 안드로이드에서 팝업이 표시되면 체크하세요.
  3-4. adb를 다운로드받고 PC커맨드창에서 "adb shell pm grant com.kisapilot.connect android.permission.READ_LOGS" 를 실행하세요.
       * adb는 구글에서 "platform tools"를 검색하여 다운로드 받으면 압축파일 안에 들어있음.
4. C3와 안드로이드 단말기가 같은 네트워크에 있어야 합니다. LTE라우터나 안드로이드 핫스팟을 이용하세요.
5. C3 UI메뉴에서 네비어플 종류를 선택하고 안드로이드 장치 IP를 입력하세요.
6. IP입력시 ,(콤마)로 구분하여 입력하고 공백은 넣지마세요. 입력예시는 고정의경우, 예를들면 IP 2개 입력시 192.168.0.101,192.168.0.102 이런식으로 입력하고,
   할당 IP가 계속 변동된다면 범위로도 입력 가능합니다. 범위로 입력시 192.168.0.1-254 (끝자리 숫자가 변동시) 혹은 192.168.1-254.100 (중간 숫자가 변동시)
7. 와이파이가 정상적으로 작동되면 기사커넥트앱과 네비어플을 실행합니다.
8. C3는 60초마다 안드로이드 기사커넥트앱과 통신을 시도합니다.
9. C3와 안드로이드 연결되면 네비어플 로그를 읽어서 기사커넥트앱에 데이터가 표시됩니다. 우측 상단 동그라미가 초록색으로 표시되면 정상적으로 연결된 것입니다.
10. 기사커넥트앱은 배터리최적화 예외가 되어야 하며, 뒤로가기로 종료하지 않으면 백그라운드에서 실행됩니다.
11. 항상 안전운전하시고 펀드라이빙 하시길 바랍니다.
12. 수정된 네비 apk파일은 배포자의 노력이 상당히 들어간 것으로 편리한 오파생활을 누릴 수 있도록 순수한 의도로 배포하는 것이니 개인용도로만 이용하시기 바랍니다.