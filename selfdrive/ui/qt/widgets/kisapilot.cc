#include <algorithm>
#include <iterator>
#include <unistd.h>

#include <QHBoxLayout>
#include <QTextStream>
#include <QFile>
#include <QNetworkReply>

#include <QProcess>
#include <QAction>
#include <QMenu>
#include <QDateTime>
#include <QVBoxLayout>

#include "common/params.h"

#include "selfdrive/ui/qt/api.h"
#include "selfdrive/ui/qt/offroad/settings.h"
#include "selfdrive/ui/qt/widgets/input.h"

#include "selfdrive/ui/qt/widgets/kisapilot.h"


CLateralControlGroup::CLateralControlGroup() : CGroupWidget( tr("Lateral Control(Reboot)") ) 
{
  QString str_param = "LateralControlMethod";

  auto str = QString::fromStdString( params.get( str_param.toStdString() ) );
  int value = str.toInt();
  m_nMethod = value;     


  // label
  method_label = new QPushButton("method"); // .setAlignment(Qt::AlignVCenter|Qt::AlignHCenter);
  method_label->setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #00A12E;
  )");
  method_label->setFixedSize( 500, 100);
  hlayout->addWidget(method_label);
  connect(method_label, &QPushButton::clicked, [=]() {
    m_nMethod += 1;
    if( m_nMethod >= LAT_ALL )
      m_nMethod = 0;

    QString values = QString::number(m_nMethod);
    params.put( str_param.toStdString(), values.toStdString());      
    refresh();
  });

  main_layout->addLayout(hlayout);

  FramePID( CreateBoxLayout(LAT_PID) );
  FrameINDI( CreateBoxLayout(LAT_INDI) );
  FrameLQR( CreateBoxLayout(LAT_LQR) );
  FrameTORQUE( CreateBoxLayout(LAT_TOROUE));
  FrameMULTI( CreateBoxLayout(LAT_MULTI) );

  refresh();
}

void  CLateralControlGroup::FramePID(QVBoxLayout *layout)
{
  // QVBoxLayout *pBoxLayout = CreateBoxLayout(LAT_PID);

    layout->addWidget(new PidKp());
    layout->addWidget(new PidKi());
    layout->addWidget(new PidKd());
    layout->addWidget(new PidKf());
}


void  CLateralControlGroup::FrameINDI(QVBoxLayout *layout)
{
 //  QVBoxLayout *pBoxLayout = CreateBoxLayout(LAT_INDI);
   
    layout->addWidget(new InnerLoopGain());
    layout->addWidget(new OuterLoopGain());
    layout->addWidget(new TimeConstant());
    layout->addWidget(new ActuatorEffectiveness());
}

void  CLateralControlGroup::FrameLQR(QVBoxLayout *layout)
{
 //  QVBoxLayout *layout = CreateBoxLayout(LAT_LQR);
   
    layout->addWidget(new Scale());
    layout->addWidget(new LqrKi());
    layout->addWidget(new DcGain());
}


void  CLateralControlGroup::FrameTORQUE(QVBoxLayout *layout)
{
  // QVBoxLayout *layout = CreateBoxLayout(LAT_TOROUE);
   
    layout->addWidget(new TorqueMaxLatAccel());
    layout->addWidget(new TorqueKp());
    layout->addWidget(new TorqueKf());
    layout->addWidget(new TorqueKi());
    layout->addWidget(new TorqueFriction());
    layout->addWidget(new TorqueUseLiveFriction());
    layout->addWidget(new TorqueUseAngle());
    layout->addWidget(new TorqueAngDeadZone());
}



void  CLateralControlGroup::FrameMULTI(QVBoxLayout *layout)
{
  // QVBoxLayout *layout = CreateBoxLayout(LAT_MULTI);
    layout->addWidget(new MultipleLatSelect());
    layout->addWidget(new MultipleLateralSpeed());
    layout->addWidget(new MultipleLateralAngle());

    layout->addWidget(new AbstractControl("[3.TORQUE]","torque","../assets/offroad/icon_shell.png"));
    FrameTORQUE( layout );
    layout->addWidget(new AbstractControl("[2.LQR]","lqr","../assets/offroad/icon_shell.png"));
    FrameLQR( layout );
    layout->addWidget(new AbstractControl("[1.INDI]","indi","../assets/offroad/icon_shell.png"));
    FrameINDI( layout );
    layout->addWidget(new AbstractControl("[0.PID]","pid","../assets/offroad/icon_shell.png"));
    FramePID( layout );
}


void CLateralControlGroup::refresh( int nID )
{
  CGroupWidget::refresh( m_nMethod );
 

  QString  str;
  switch( m_nMethod )
  {
    case LAT_PID : str = "0.PID"; break;
    case LAT_INDI : str = "1.INDI";  break;
    case LAT_LQR : str = "2.LQR";  break;
    case LAT_TOROUE : str = "3.TORQUE";  break;
    case LAT_MULTI : str = "4.MULTI";  break;
  }

  method_label->setText( str ); 
}


CLongControlGroup::CLongControlGroup() : CGroupWidget( tr("Long Control") ) 
{
   QVBoxLayout *pBoxLayout = CreateBoxLayout();

  //pBoxLayout->addWidget(new LabelControl("〓〓〓〓〓〓〓【 LONGCONTROL 】〓〓〓〓〓〓〓", ""));
  pBoxLayout->addWidget(new CustomTRToggle());
  pBoxLayout->addWidget(new CruiseGapTR());
  pBoxLayout->addWidget(new DynamicTRGap());
  pBoxLayout->addWidget(new DynamicTRUD());
  pBoxLayout->addWidget(new DynamicTRBySpeed());
  pBoxLayout->addWidget(new RadarLongHelperOption());
  pBoxLayout->addWidget(new StoppingDistAdjToggle());
  pBoxLayout->addWidget(new StoppingDist());
  //pBoxLayout->addWidget(new E2ELongToggle());
  //pBoxLayout->addWidget(new StopAtStopSignToggle());
  pBoxLayout->addWidget(new StockDecelonCamToggle());
  //pBoxLayout->addWidget(new RadarDisableToggle());
  pBoxLayout->addWidget(new UseRadarTrackToggle());
  pBoxLayout->addWidget(new UseRadarValue());
  pBoxLayout->addWidget(new LongAlternative());
  pBoxLayout->addWidget(new KISACruiseGapSet());
}


CGitGroup::CGitGroup(void *p) : CGroupWidget( tr("Git Repository/Branch") ) 
{
   QVBoxLayout *pBoxLayout = CreateBoxLayout();

  const char* git_reset = "/data/openpilot/selfdrive/assets/addon/script/git_reset.sh ''";
  auto gitresetbtn = new ButtonControl(tr("Git Reset"), tr("RUN"));
  QObject::connect(gitresetbtn, &ButtonControl::clicked, [=]() {
    if (ConfirmationDialog::confirm2(tr("Apply the latest commitment details of Remote Git after forced initialization of local changes. Do you want to proceed?"), this)){
      std::system("touch /data/kisa_compiling");
      std::system(git_reset);
    }
  });

  const char* gitpull_cancel = "/data/openpilot/selfdrive/assets/addon/script/gitpull_cancel.sh ''";
  auto gitpullcanceltbtn = new ButtonControl(tr("GitPull Restore"), tr("RUN"));
  QObject::connect(gitpullcanceltbtn, &ButtonControl::clicked, [=]() {
    std::system(gitpull_cancel);
    GitPullCancel::confirm(this);
  });

  pBoxLayout->addWidget( new GitPullOnBootToggle() );

  pBoxLayout->addWidget( new SwitchOpenpilot() ); // kisa
  pBoxLayout->addWidget( new BranchSelectCombo() ); // kisa

  pBoxLayout->addWidget( gitresetbtn );
  pBoxLayout->addWidget( gitpullcanceltbtn );  
}

CResumeGroup::CResumeGroup(void *p) : CGroupWidget( tr("SCC Resume Option") ) 
{
  QVBoxLayout *pBoxLayout = CreateBoxLayout();

  pBoxLayout->addWidget(new AutoResumeToggle());
  pBoxLayout->addWidget(new RESCountatStandstill());
  pBoxLayout->addWidget(new StandstillResumeAltToggle());
  pBoxLayout->addWidget(new DepartChimeAtResume());
  pBoxLayout->addWidget(new CruiseAutoResToggle());
  pBoxLayout->addWidget(new RESChoice());
  pBoxLayout->addWidget(new AutoResCondition());
  pBoxLayout->addWidget(new AutoResLimitTime());
  pBoxLayout->addWidget(new AutoRESDelay());
}

CCruiseGapGroup::CCruiseGapGroup(void *p) : CGroupWidget( tr("Cruise Gap Option") ) 
{
  QVBoxLayout *pBoxLayout = CreateBoxLayout();

  pBoxLayout->addWidget(new CruiseGapAdjustToggle());
  pBoxLayout->addWidget(new CruiseGapBySpdOn());
  pBoxLayout->addWidget(new CruiseGapBySpd());
  pBoxLayout->addWidget(new KISAEarlyStoppingToggle());
}

CVariableCruiseGroup::CVariableCruiseGroup(void *p) : CGroupWidget( tr("Variable Cruise Option") ) 
{
  QVBoxLayout *pBoxLayout = CreateBoxLayout();

  pBoxLayout->addWidget(new VariableCruiseToggle());
  pBoxLayout->addWidget(new CruiseSpammingLevel());
  pBoxLayout->addWidget(new KISACruiseSpammingInterval());
  pBoxLayout->addWidget(new KISACruiseSpammingBtnCount());
  pBoxLayout->addWidget(new CruisemodeSelInit());
  pBoxLayout->addWidget(new CruiseOverMaxSpeedToggle());
  pBoxLayout->addWidget(new SetSpeedPlus());
}

CLaneChangeGroup::CLaneChangeGroup(void *p) : CGroupWidget( tr("Lane Change Option") ) 
{
  QVBoxLayout *pBoxLayout = CreateBoxLayout();

  pBoxLayout->addWidget(new LaneChangeSpeed());
  pBoxLayout->addWidget(new LaneChangeDelay());
  pBoxLayout->addWidget(new LCTimingFactorUD());
  pBoxLayout->addWidget(new LCTimingFactor());
  pBoxLayout->addWidget(new LCTimingKeepFactorUD());
  pBoxLayout->addWidget(new LCTimingKeepFactor());
}

CDrivingQuality::CDrivingQuality(void *p) : CGroupWidget( tr("Driving Quality Option") ) 
{
  QVBoxLayout *pBoxLayout = CreateBoxLayout();

  pBoxLayout->addWidget(new LeftCurvOffset());
  pBoxLayout->addWidget(new RightCurvOffset());
  pBoxLayout->addWidget(new TurnSteeringDisableToggle());
  pBoxLayout->addWidget(new LaneWidth());
  pBoxLayout->addWidget(new SpeedLaneWidthUD());
  pBoxLayout->addWidget(new SpeedLaneWidth());
  pBoxLayout->addWidget(new CloseToRoadEdgeToggle());
  pBoxLayout->addWidget(new KISAEdgeOffset());
  pBoxLayout->addWidget(new ToAvoidLKASFaultToggle());
  pBoxLayout->addWidget(new ToAvoidLKASFault());
  pBoxLayout->addWidget(new AutoEnabledToggle());
  pBoxLayout->addWidget(new AutoEnableSpeed());
  pBoxLayout->addWidget(new RegenBrakeFeatureToggle());
  pBoxLayout->addWidget(new RegenBrakeFeature());
}

CSafetyandMap::CSafetyandMap(void *p) : CGroupWidget( tr("Safety Speed and Map Option") ) 
{
  QVBoxLayout *pBoxLayout = CreateBoxLayout();

  pBoxLayout->addWidget(new OSMEnabledToggle());
  pBoxLayout->addWidget(new OSMSpeedLimitEnabledToggle());
  pBoxLayout->addWidget(new SpeedLimitOffset());
  pBoxLayout->addWidget(new OSMCustomSpeedLimitUD());
  pBoxLayout->addWidget(new OSMCustomSpeedLimit());
  pBoxLayout->addWidget(new SpeedLimitSignType());
  pBoxLayout->addWidget(new CamDecelDistAdd());
  pBoxLayout->addWidget(new CurvDecelSelect());
  pBoxLayout->addWidget(new VCurvSpeedUD());
  pBoxLayout->addWidget(new VCurvSpeed());
  pBoxLayout->addWidget(new OCurvSpeedUD());
  pBoxLayout->addWidget(new OCurvSpeed());
  pBoxLayout->addWidget(new SpeedBumpDecelToggle());
  pBoxLayout->addWidget(new CruiseSetwithRoadLimitSpeed());
  pBoxLayout->addWidget(new CruiseSetwithRoadLimitSpeedOffset());
  pBoxLayout->addWidget(new RoutineDriveOnToggle());
  pBoxLayout->addWidget(new RoutineDriveOption());
}

CUtilWidget::CUtilWidget( void *p ) : CGroupWidget( tr("Util Program") ) 
{
  QVBoxLayout *pBoxLayout = CreateBoxLayout();

  const char* panda_flashing = "/data/openpilot/selfdrive/assets/addon/script/panda_flashing.sh";
  auto pandaflashingtbtn = new ButtonControl(tr("Panda Flashing(OLD)"), tr("RUN"));
  QObject::connect(pandaflashingtbtn, &ButtonControl::clicked, [=]() {
    if (ConfirmationDialog::confirm2(tr("Panda's green LED blinks quickly during panda flashing. Never turn off or disconnect the device arbitrarily. Do you want to proceed?"), this)) {
      std::system(panda_flashing);
    }
  });

  const char* panda_flashing_new = "/data/openpilot/panda/board/recover.sh";
  auto pandaflashingtbtn_new = new ButtonControl(tr("Panda Flashing(NEW)"), tr("RUN"));
  QObject::connect(pandaflashingtbtn_new, &ButtonControl::clicked, [=]() {
    if (ConfirmationDialog::confirm2(tr("Panda's green LED blinks quickly during panda flashing. Never turn off or disconnect the device arbitrarily. Do you want to proceed?"), this)) {
      std::system(panda_flashing_new);
    }
  });

  pBoxLayout->addWidget( pandaflashingtbtn );
  pBoxLayout->addWidget( pandaflashingtbtn_new );
}
CPresetWidget::CPresetWidget() : CGroupWidget( tr("Parameter Preset") ) 
{
  QVBoxLayout *pBoxLayout = CreateBoxLayout();
/*
  MenuControl *pMenu1 = new MenuControl( 
    "KisaMaxSteeringAngle",
    "Driver to Steer Angle",
    "mprove the edge between the driver and the openpilot.",
    "../assets/offroad/icon_shell.png"    
    );
  pMenu1->SetControl( 10, 180, 5 );
  m_pBoxLayout->addWidget( pMenu1 );
*/
  // preset1 buttons
  QHBoxLayout *presetone_layout = new QHBoxLayout();
  presetone_layout->setSpacing(50);

  QPushButton *presetoneload_btn = new QPushButton(tr("Load Preset1"));
  presetoneload_btn->setStyleSheet("height: 120px;border-radius: 15px;background-color: #393939;");
  presetone_layout->addWidget(presetoneload_btn);
  QObject::connect(presetoneload_btn, &QPushButton::clicked, [=]() {
    if (ConfirmationDialog::confirm2(tr("Do you want to load Preset1?"), this)) {
      std::system("/data/openpilot/selfdrive/assets/addon/script/load_preset1.sh");
    }
  });

  QPushButton *presetonesave_btn = new QPushButton(tr("Save Preset1"));
  presetonesave_btn->setStyleSheet("height: 120px;border-radius: 15px;background-color: #393939;");
  presetone_layout->addWidget(presetonesave_btn);
  QObject::connect(presetonesave_btn, &QPushButton::clicked, [=]() {
    if (ConfirmationDialog::confirm2(tr("Do you want to save Preset1?"), this)) {
      std::system("/data/openpilot/selfdrive/assets/addon/script/save_preset1.sh");
    }
  });

  // preset2 buttons
  QHBoxLayout *presettwo_layout = new QHBoxLayout();
  presettwo_layout->setSpacing(50);

  QPushButton *presettwoload_btn = new QPushButton(tr("Load Preset2"));
  presettwoload_btn->setStyleSheet("height: 120px;border-radius: 15px;background-color: #393939;");
  presettwo_layout->addWidget(presettwoload_btn);
  QObject::connect(presettwoload_btn, &QPushButton::clicked, [=]() {
    if (ConfirmationDialog::confirm2(tr("Do you want to load Preset2?"), this)) {
      std::system("/data/openpilot/selfdrive/assets/addon/script/load_preset2.sh");
    }
  });

  QPushButton *presettwosave_btn = new QPushButton(tr("Save Preset2"));
  presettwosave_btn->setStyleSheet("height: 120px;border-radius: 15px;background-color: #393939;");
  presettwo_layout->addWidget(presettwosave_btn);
  QObject::connect(presettwosave_btn, &QPushButton::clicked, [=]() {
    if (ConfirmationDialog::confirm2(tr("Do you want to save Preset2?"), this)) {
      std::system("/data/openpilot/selfdrive/assets/addon/script/save_preset2.sh");
    }
  });

  auto paraminit_btn = new ButtonControl(tr("Parameters Init"), tr("RUN"));
  QObject::connect(paraminit_btn, &ButtonControl::clicked, [=]() {
    if (ConfirmationDialog::confirm2(tr("Initialize parameters. Changes in the Device menu are changed to the initial set value. Do you want to proceed?"), this)){
      std::system("/data/openpilot/selfdrive/assets/addon/script/init_param.sh");
    }
  });

  pBoxLayout->addLayout( presetone_layout );
  pBoxLayout->addLayout( presettwo_layout );
  pBoxLayout->addWidget( paraminit_btn );

  pBoxLayout->addWidget( new OpenpilotUserEnv() ); // kisa

  main_layout->addStretch();
  refresh();
}

void CPresetWidget::refresh( int nID )
{
  CGroupWidget::refresh( nID );  
}

SwitchOpenpilot::SwitchOpenpilot() : ButtonControl(tr("Change Repo/Branch"), "", tr("Change to another open pilot code. You can change it by entering ID/repository/branch.")) {
  QObject::connect(this, &ButtonControl::clicked, [=]() {
    if (text() == tr("CHANGE")) {
      QString userid = InputDialog::getText(tr("First: Input the Git ID."), this, "github.com/<ID>/<Repository>.git -b <Branch>", false, 1, "kisapilot");
      if (userid.length() > 0) {
        getUserID(userid);
        QString repoid = InputDialog::getText(tr("Second: Input the repository."), this, "github.com/"+userid+"/<Repository>.git -b <Branch>", false, 1, "openpilot");
        if (repoid.length() > 0) {
          getRepoID(repoid);
          QString branchid = InputDialog::getText(tr("Last: Input the branch name."), this, "github.com/"+userid+"/"+repoid+".git -b <Branch>", false, 1, "KISA");
          if (branchid.length() > 0) {
            getBranchID(branchid);
            githubbranch = branchid;
            QString cmd0 = tr("This will download the branch and takes a little time.") + "\n" + QString::fromStdString("https://github.com/") + githubid + QString::fromStdString("/") + githubrepo + QString::fromStdString(".git\n") + tr("Branch: ") + githubbranch;
            if (ConfirmationDialog::confirm2(cmd0, this)) {
              setText(tr("DONE"));
              setEnabled(true);
              QString time_format = "yyyyMMddHHmmss";
              QDateTime a = QDateTime::currentDateTime();
              QString as = a.toString(time_format);
              QString cmd1 = "mv /data/openpilot /data/openpilot_" + as;
              QString tcmd = "git clone --progress -b " + githubbranch + " --single-branch https://github.com/" + githubid + "/" + githubrepo + ".git /data/openpilot";
              QString cmd3 = "rm -f /data/openpilot_" + as + "/prebuilt";
			        QString cmd4 = "touch /data/kisa_compiling";
              QProcess::execute(cmd1);
              QProcess::execute(cmd3);
			        QProcess::execute(cmd4);
              QObject::connect(&textMsgProcess, SIGNAL(finished(int, QProcess::ExitStatus)), this, SLOT(processFinished(int, QProcess::ExitStatus)));
              textMsgProcess.start(tcmd);
            }
          }
        }
      }
    } else {
      refresh();
    }
  });
  refresh();
}

void SwitchOpenpilot::processFinished(int exitCode, QProcess::ExitStatus exitStatus) {
  if(exitStatus == QProcess::NormalExit) {
    QProcess::execute("rm -f /data/openpilot/prebuilt");
    QProcess::execute("sudo reboot");
  }
}

void SwitchOpenpilot::refresh() {
  setText(tr("CHANGE"));
  setEnabled(true);
}

void SwitchOpenpilot::getUserID(const QString &userid) {
  HttpRequest *request = new HttpRequest(this, false);
  QObject::connect(request, &HttpRequest::requestDone, [=](const QString &resp, bool success) {
    if (success) {
      if (!resp.isEmpty()) {
        githubid = userid;
      } else {
        ConfirmationDialog::alert(userid + tr(" The ID does not exist. Return to the input window, press the cancel button, and try again from the beginning."), this);
      }
    } else {
      if (request->timeout()) {
        ConfirmationDialog::alert(tr("The requested time has exceeded."), this);
      } else {
        ConfirmationDialog::alert(tr("The ID does not exist. Return to the input window, press the cancel button, and try again from the beginning."), this);
      }
    }

    refresh();
    request->deleteLater();
  });
  request->sendRequest("https://github.com/" + userid);
}

void SwitchOpenpilot::getRepoID(const QString &repoid) {
  HttpRequest *request = new HttpRequest(this, false);
  QObject::connect(request, &HttpRequest::requestDone, [=](const QString &resp, bool success) {
    if (success) {
      if (!resp.isEmpty()) {
        githubrepo = repoid;
      } else {
        ConfirmationDialog::alert(repoid + tr(" The repository does not exist. Return to the input window, press the cancel button, and try again from the beginning."), this);
      }
    } else {
      if (request->timeout()) {
        ConfirmationDialog::alert(tr("The requested time has exceeded."), this);
      } else {
        ConfirmationDialog::alert(tr("The Repository does not exist. Return to the input window, press the cancel button, and try again from the beginning."), this);
      }
    }

    refresh();
    request->deleteLater();
  });
  request->sendRequest("https://github.com/" + githubid + "/" + repoid);
}

void SwitchOpenpilot::getBranchID(const QString &branchid) {
  HttpRequest *request = new HttpRequest(this, false);
  QObject::connect(request, &HttpRequest::requestDone, [=](const QString &resp, bool success) {
    if (success) {
      if (!resp.isEmpty()) {
        githubbranch = branchid;
      } else {
        ConfirmationDialog::alert(branchid + tr(" The branch does not exist. Press the cancel button and try again from the beginning."), this);
      }
    } else {
      if (request->timeout()) {
        ConfirmationDialog::alert(tr("The requested time has exceeded."), this);
      } else {
        ConfirmationDialog::alert(tr("The Branch does not exist. Return to the input window, press the cancel button, and try again from the beginning."), this);
      }
    }

    refresh();
    request->deleteLater();
  });
  request->sendRequest("https://github.com/" + githubid + "/" + githubrepo + "/tree/" + branchid);
}

OpenpilotUserEnv::OpenpilotUserEnv() : ButtonControl(tr("Get Your Params"), "", tr("Get parameters from github. This is useful to apply your own file.")) {
  QObject::connect(this, &ButtonControl::clicked, [=]() {
    if (text() == tr("GET")) {
      QString userid = InputDialog::getText(tr("Input your Git ID"), this, "github.com/<your id>/openpilot_user/main/user_params.txt", false, 1, "multikyd");
      if (userid.length() > 0) {
        getUserID(userid);
        QString repoid = InputDialog::getText(tr("Input your repository"), this, "github.com/"+userid, false, 1, "openpilot_user");
        if (repoid.length() > 0) {
          getRepoID(repoid);
          QString branchid = InputDialog::getText(tr("Input your branch"), this, "github.com/"+userid+"/"+repoid, false, 1, "main");
          if (branchid.length() > 0) {
            getBranchID(branchid);
            QString fileid = InputDialog::getText(tr("Input your file"), this, "github.com/"+userid+"/"+repoid+"/"+branchid, false, 1, "user_params.txt");
            if (fileid.length() > 0) {
              getFileID(fileid);
              githubbranch = branchid;
              QString cmd0 = tr("Check Inputs. Do you want to Run?") + "\n" + QString::fromStdString("https://github.com/") + userid + QString::fromStdString("/") + repoid + QString::fromStdString("/") + branchid + QString::fromStdString("/") + fileid;
              if (ConfirmationDialog::confirm2(cmd0, this)) {
                setText(tr("DONE"));
                setEnabled(true);
                QString tcmd = "wget https://raw.githubusercontent.com/" + githubid + "/" + githubrepo + "/" + githubbranch + "/" + githubfile + " -O /data/User_Params.txt";
                QString cmd4 = "touch /data/kisa_compiling";
                QProcess::execute(cmd4);
                QObject::connect(&textMsgProcess, SIGNAL(finished(int, QProcess::ExitStatus)), this, SLOT(processFinished(int, QProcess::ExitStatus)));
                textMsgProcess.start(tcmd);
              }
            }
          }
        }
      }
    } else {
      refresh();
    }
  });
  refresh();
}

void OpenpilotUserEnv::processFinished(int exitCode, QProcess::ExitStatus exitStatus) {
  if(exitStatus == QProcess::NormalExit) {
    QProcess::execute("rm -f /data/openpilot/prebuilt");
    QProcess::execute("sudo reboot");
  }
}

void OpenpilotUserEnv::refresh() {
  setText(tr("GET"));
  setEnabled(true);
}

void OpenpilotUserEnv::getUserID(const QString &userid) {
  HttpRequest *request = new HttpRequest(this, false);
  QObject::connect(request, &HttpRequest::requestDone, [=](const QString &resp, bool success) {
    if (success) {
      if (!resp.isEmpty()) {
        githubid = userid;
      } else {
        ConfirmationDialog::alert(userid + tr(" The ID does not exist. Return to the input window, press the cancel button, and try again from the beginning."), this);
      }
    } else {
      if (request->timeout()) {
        ConfirmationDialog::alert(tr("The requested time has exceeded."), this);
      } else {
        ConfirmationDialog::alert(tr("The ID does not exist. Return to the input window, press the cancel button, and try again from the beginning."), this);
      }
    }

    refresh();
    request->deleteLater();
  });
  request->sendRequest("https://github.com/" + userid);
}

void OpenpilotUserEnv::getRepoID(const QString &repoid) {
  HttpRequest *request = new HttpRequest(this, false);
  QObject::connect(request, &HttpRequest::requestDone, [=](const QString &resp, bool success) {
    if (success) {
      if (!resp.isEmpty()) {
        githubrepo = repoid;
      } else {
        ConfirmationDialog::alert(repoid + tr(" The repository does not exist. Return to the input window, press the cancel button, and try again from the beginning."), this);
      }
    } else {
      if (request->timeout()) {
        ConfirmationDialog::alert(tr("The requested time has exceeded."), this);
      } else {
        ConfirmationDialog::alert(tr("The Repository does not exist. Return to the input window, press the cancel button, and try again from the beginning."), this);
      }
    }

    refresh();
    request->deleteLater();
  });
  request->sendRequest("https://github.com/" + githubid + "/" + repoid);
}

void OpenpilotUserEnv::getBranchID(const QString &branchid) {
  HttpRequest *request = new HttpRequest(this, false);
  QObject::connect(request, &HttpRequest::requestDone, [=](const QString &resp, bool success) {
    if (success) {
      if (!resp.isEmpty()) {
        githubbranch = branchid;
      } else {
        ConfirmationDialog::alert(branchid + tr(" The branch does not exist. Press the cancel button and try again from the beginning."), this);
      }
    } else {
      if (request->timeout()) {
        ConfirmationDialog::alert(tr("The requested time has exceeded."), this);
      } else {
        ConfirmationDialog::alert(tr("The Branch does not exist. Return to the input window, press the cancel button, and try again from the beginning."), this);
      }
    }

    refresh();
    request->deleteLater();
  });
  request->sendRequest("https://github.com/" + githubid + "/" + githubrepo + "/tree/" + branchid);
}

void OpenpilotUserEnv::getFileID(const QString &fileid) {
  HttpRequest *request = new HttpRequest(this, false);
  QObject::connect(request, &HttpRequest::requestDone, [=](const QString &resp, bool success) {
    if (success) {
      if (!resp.isEmpty()) {
        githubfile = fileid;
      } else {
        ConfirmationDialog::alert(fileid + tr(" The file does not exist. Press the cancel button and try again from the beginning."), this);
      }
    } else {
      if (request->timeout()) {
        ConfirmationDialog::alert(tr("The requested time has exceeded."), this);
      } else {
        ConfirmationDialog::alert(tr("The file does not exist. Return to the input window, press the cancel button, and try again from the beginning."), this);
      }
    }

    refresh();
    request->deleteLater();
  });
  request->sendRequest("https://github.com/" + githubid + "/" + githubrepo + "/blob/" + githubbranch + "/" + fileid);
}

GitHash::GitHash() : AbstractControl(tr("Commit (Local/Remote)"), "", "") {

  QString lhash = QString::fromStdString(params.get("GitCommit").substr(0, 10));
  QString rhash = QString::fromStdString(params.get("GitCommitRemote").substr(0, 10));
  hlayout->addStretch(1);
  
  local_hash.setText(QString::fromStdString(params.get("GitCommit").substr(0, 10)));
  remote_hash.setText(QString::fromStdString(params.get("GitCommitRemote").substr(0, 10)));
  //local_hash.setAlignment(Qt::AlignVCenter);
  remote_hash.setAlignment(Qt::AlignVCenter);
  local_hash.setStyleSheet("color: #aaaaaa");
  if (lhash == rhash) {
    remote_hash.setStyleSheet("color: #aaaaaa");
  } else {
    remote_hash.setStyleSheet("color: #0099ff");
  }
  hlayout->addWidget(&local_hash);
  hlayout->addWidget(&remote_hash);
}

OpenpilotView::OpenpilotView() : AbstractControl(tr("Driving Camera"), tr("Preview the open pilot driving screen."), "") {

  // setup widget
  hlayout->addStretch(1);

  btn.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");

  btnc.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");

  btn.setFixedSize(250, 100);
  btnc.setFixedSize(250, 100);
  hlayout->addWidget(&btnc);
  hlayout->addWidget(&btn);

  QObject::connect(&btn, &QPushButton::clicked, [=]() {
    bool stat = params.getBool("IsOpenpilotViewEnabled");
    if (stat) {
      params.putBool("IsOpenpilotViewEnabled", false);
      uiState()->scene.cal_view = false;
    } else {
      params.putBool("IsOpenpilotViewEnabled", true);
      uiState()->scene.cal_view = false;
      std::system("sudo pkill -f selfdrive.boardd.pandad");
    }
    refresh();
  });
  QObject::connect(&btnc, &QPushButton::clicked, [=]() {
    bool stat = params.getBool("IsOpenpilotViewEnabled");
    if (stat) {
      params.putBool("IsOpenpilotViewEnabled", false);
      uiState()->scene.cal_view = false;
    } else {
      params.putBool("IsOpenpilotViewEnabled", true);
      uiState()->scene.cal_view = true;
      std::system("sudo pkill -f selfdrive.boardd.pandad");
    }
    refresh();
  });
  refresh();
}

void OpenpilotView::refresh() {
  bool param = params.getBool("IsOpenpilotViewEnabled");
  QString car_param = QString::fromStdString(params.get("CarParams"));
  if (param) {
    btn.setText(tr("UNVIEW"));
    btnc.setText(tr("UNVIEW"));
  } else {
    btn.setText(tr("PREVIEW"));
    btnc.setText(tr("CALVIEW"));
  }
  if (car_param.length()) {
    btn.setEnabled(false);
    btnc.setEnabled(false);
  } else {
    btn.setEnabled(true);
    btnc.setEnabled(true);
  }
}

CarSelectCombo::CarSelectCombo() : AbstractControl("", "", "") 
{
  QStringList stringList;
  QFile carlistfile("/data/CarList");
  if (carlistfile.open(QIODevice::ReadOnly)) {
    QTextStream carname(&carlistfile);
    while (!carname.atEnd()) {
      QString line = carname.readLine();
      stringList.append(line);
    }
    carlistfile.close();
  }

  hlayout->addStretch(1);

  btn1.setStyleSheet(R"(
    padding: 0;
    border-radius: 0px;
    font-size: 50px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");

  btn2.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");

  btn1.setFixedSize(1250, 100);
  btn2.setFixedSize(200, 100);
  hlayout->addWidget(&btn1);
  hlayout->addWidget(&btn2);
  btn1.setText(tr("Select Your Car"));
  btn2.setText(tr("UNSET"));

  QObject::connect(&btn1, &QPushButton::clicked, [=]() {
    QString cur = QString::fromStdString(params.get("CarModel"));
    QString selection = MultiOptionDialog::getSelection(tr("Select Your Car"), stringList, cur, this);
    if (!selection.isEmpty()) {
      params.put("CarModel", selection.toStdString());
    }
    refresh();
  });

  QObject::connect(&btn2, &QPushButton::clicked, [=]() {
    if (btn2.text() == tr("UNSET")) {
      if (ConfirmationDialog::confirm2(tr("Do you want to unset?"), this)) {
        params.remove("CarModel");
        refresh();
      }
    }
  });
  refresh();
}

void CarSelectCombo::refresh() {
  QString selected_carname = QString::fromStdString(params.get("CarModel"));
  if (selected_carname.length()) {
    btn1.setText(selected_carname);
  } else {
    btn1.setText(tr("Select Your Car"));
  }
}

ModelSelectCombo::ModelSelectCombo() : AbstractControl("", "", "") 
{
  QStringList stringList;
  QFile modellistfile("/data/openpilot/selfdrive/assets/addon/model/ModelList");
  if (modellistfile.open(QIODevice::ReadOnly)) {
    QTextStream modelname(&modellistfile);
    while (!modelname.atEnd()) {
      QString line = modelname.readLine();
      stringList.append(line);
    }
    modellistfile.close();
  }

  hlayout->addStretch(1);

  label.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  hlayout->addWidget(&label);

  btn1.setStyleSheet(R"(
    padding: 0;
    border-radius: 0px;
    font-size: 50px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");

  btn2.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");

  btn1.setFixedSize(1100, 100);
  btn2.setFixedSize(200, 100);
  hlayout->addWidget(&btn1);
  hlayout->addWidget(&btn2);
  btn1.setText(QString::fromStdString(params.get("DrivingModel")));
  btn2.setText(tr("Reset"));
  label.setText(tr("Model"));

  QObject::connect(&btn1, &QPushButton::clicked, [=]() {
    QString cur = QString::fromStdString(params.get("DrivingModel"));
    selection = MultiOptionDialog::getSelection(tr("Select Driving Model"), stringList, cur, this);
    if (!selection.isEmpty()) {
      if (selection != cur) {
        if (ConfirmationDialog::confirm2("<" + selection + "> " + tr("Driving model will be changed. Downloading(50MB) takes a time. Will be reboot if done."), this)) {
          params.put("DrivingModel", selection.toStdString());
          QProcess::execute("touch /data/kisa_compiling");
          params.put("RunCustomCommand", "4", 1);
        }
      }
    }
    btn1.setText(QString::fromStdString(params.get("DrivingModel")));
    refresh();
  });

  QObject::connect(&btn2, &QPushButton::clicked, [=]() {
    if (ConfirmationDialog::confirm2(tr("Do you want to restore original model?"), this)) {
      params.remove("DrivingModel");
      QProcess::execute("touch /data/kisa_compiling");
      params.put("RunCustomCommand", "5", 1);
      refresh();
    }
  });
  refresh();
}

void ModelSelectCombo::refresh() {
  QString selected_modelname = QString::fromStdString(params.get("DrivingModel"));
  if (selected_modelname.length()) {
    btn1.setText(selected_modelname);
  } else {
    btn1.setText(tr("Original Model"));
  }
}

BranchSelectCombo::BranchSelectCombo() : AbstractControl("", "", "") 
{
  hlayout->addStretch(1);

  btn1.setStyleSheet(R"(
    padding: 0;
    border-radius: 0px;
    font-size: 50px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");

  btn2.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");

  btn1.setFixedSize(1000, 100);
  btn2.setFixedSize(250, 100);
  hlayout->addWidget(&btn1);
  hlayout->addWidget(&btn2);
  btn1.setText(QString::fromStdString(params.get("GitBranch")));
  btn2.setText(tr("RELOAD"));

  QObject::connect(&btn1, &QPushButton::clicked, [=]() {
    if(access("/data/branches", F_OK ) != -1) {
      QFile branchlistfile("/data/branches");
      if (branchlistfile.open(QIODevice::ReadOnly)) {
        QTextStream branchname(&branchlistfile);
        stringList = QStringList();
        while (!branchname.atEnd()) {
          QString line = branchname.readLine();
          stringList.append(line);
        }
        branchlistfile.close();
      }
      QString cur = QString::fromStdString(params.get("GitBranch"));
      selection = MultiOptionDialog::getSelection(tr("Change Your Branch"), stringList, cur, this);
      if (!selection.isEmpty()) {
        if (selection != cur) {
          if (ConfirmationDialog::confirm2(tr("Now will checkout the branch") +", <" + selection + ">. " + tr("The device will be rebooted if completed."), this)) {
            QProcess::execute("touch /data/kisa_compiling");
            params.put("RunCustomCommand", selection.toStdString());
          }
        }
      }
    } else {
      ConfirmationDialog::alert(tr("Still getting branches, try again in a while"), this);
    }
    btn1.setText(QString::fromStdString(params.get("GitBranch")));
  });

  QObject::connect(&btn2, &QPushButton::clicked, [=]() {
    btn1.setText(tr("Push to check"));
    params.put("RunCustomCommand", "3", 1);
  });
}

TimeZoneSelectCombo::TimeZoneSelectCombo() : AbstractControl("", "", "") 
{
  combobox.setStyleSheet(R"(
    subcontrol-origin: padding;
    subcontrol-position: top left;
    selection-background-color: #111;
    selection-color: yellow;
    color: white;
    background-color: #393939;
    border-style: solid;
    border: 0px solid #1e1e1e;
    border-radius: 0;
    width: 100px;
  )");

  combobox.addItem(tr("Select Your TimeZone"));
  QFile timezonelistfile("/data/openpilot/selfdrive/assets/addon/param/TimeZone");
  if (timezonelistfile.open(QIODevice::ReadOnly)) {
    QTextStream timezonename(&timezonelistfile);
    while (!timezonename.atEnd()) {
      QString line = timezonename.readLine();
      combobox.addItem(line);
    }
    timezonelistfile.close();
  }

  combobox.setFixedWidth(1055);

  btn.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");

  btn.setFixedSize(150, 100);

  QObject::connect(&btn, &QPushButton::clicked, [=]() {
    if (btn.text() == tr("UNSET")) {
      if (ConfirmationDialog::confirm2(tr("Do you want to set default?"), this)) {
        params.put("KISATimeZone", "UTC");
        combobox.setCurrentIndex(0);
        refresh();
      }
    }
  });

  //combobox.view()->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

  hlayout->addWidget(&combobox, Qt::AlignLeft);
  hlayout->addWidget(&btn, Qt::AlignRight);

  QObject::connect(&combobox, static_cast<void(QComboBox::*)(int)>(&QComboBox::activated), [=](int index)
  {
    combobox.itemData(combobox.currentIndex());
    QString str = combobox.currentText();
    if (combobox.currentIndex() != 0) {
      if (ConfirmationDialog::confirm2(tr("Press OK to set your timezone as") + "\n" + str, this)) {
        params.put("KISATimeZone", str.toStdString());
      }
    }
    refresh();
  });
  refresh();
}

void TimeZoneSelectCombo::refresh() {
  QString selected_timezonename = QString::fromStdString(params.get("KISATimeZone"));
  int index = combobox.findText(selected_timezonename);
  if (index >= 0) combobox.setCurrentIndex(index);
  if (selected_timezonename.length()) {
    btn.setEnabled(true);
    btn.setText(tr("UNSET"));
  } else {
    btn.setEnabled(false);
    btn.setText(tr("SET"));
  }
}

//UI
AutoShutdown::AutoShutdown() : AbstractControl(tr("Device AutoShutdown"), tr("Device is automatically turned off after the set time while the engine is turned off (offload) after driving (onload)."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaAutoShutdown"));
    int value = str.toInt();
    value = value - 1;
    if (value <= -1) {
      value = 14;
    }
    QString values = QString::number(value);
    params.put("KisaAutoShutdown", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaAutoShutdown"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 15) {
      value = 0;
    }
    QString values = QString::number(value);
    params.put("KisaAutoShutdown", values.toStdString());
    refresh();
  });
  refresh();
}

void AutoShutdown::refresh() {
  QString option = QString::fromStdString(params.get("KisaAutoShutdown"));
  if (option == "0") {
    label.setText(tr("AlwaysOn"));
  } else if (option == "1") {
    label.setText(tr("RightOff"));
  } else if (option == "2") {
    label.setText(tr("30sec"));
  } else if (option == "3") {
    label.setText(tr("1min"));
  } else if (option == "4") {
    label.setText(tr("3mins"));
  } else if (option == "5") {
    label.setText(tr("5mins"));
  } else if (option == "6") {
    label.setText(tr("10mins"));
  } else if (option == "7") {
    label.setText(tr("30mins"));
  } else if (option == "8") {
    label.setText(tr("1hour"));
  } else if (option == "9") {
    label.setText(tr("3hours"));
  } else if (option == "10") {
    label.setText(tr("5hours"));
  } else if (option == "11") {
    label.setText(tr("10hours"));
  } else if (option == "12") {
    label.setText(tr("24hours"));
  } else if (option == "13") {
    label.setText(tr("48hours"));
  } else if (option == "14") {
    label.setText(tr("72hours"));
  }
}

VolumeControl::VolumeControl() : AbstractControl(tr("Device Volume Control(%)"), tr("Adjust the volume of Device. Android Default/Manual Settings"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaUIVolumeBoost"));
    int value = str.toInt();
    value = value - 10;
    if (value <= -10) {
      value = -10;
    }
    QString values = QString::number(value);
    uiState()->scene.nVolumeBoost = value;
    params.put("KisaUIVolumeBoost", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaUIVolumeBoost"));
    int value = str.toInt();
    value = value + 10;
    if (value >= 100) {
      value = 100;
    }
    QString values = QString::number(value);
    uiState()->scene.nVolumeBoost = value;
    params.put("KisaUIVolumeBoost", values.toStdString());
    refresh();
  });
  refresh();
}

void VolumeControl::refresh() {
  QString option = QString::fromStdString(params.get("KisaUIVolumeBoost"));
  if (option == "0") {
    label.setText(tr("Default"));
  } else if (option == "-10") {
    label.setText(tr("Mute"));
  } else {
    label.setText(QString::fromStdString(params.get("KisaUIVolumeBoost")));
  }
}

BrightnessControl::BrightnessControl() : AbstractControl(tr("Device Brightness Control(%)"), tr("Manually adjust the brightness of the Device screen."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaUIBrightness"));
    int value = str.toInt();
    value = value - 5;
    if (value <= 0) {
      value = 0;
    }
    uiState()->scene.brightness = value;
    QString values = QString::number(value);
    params.put("KisaUIBrightness", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaUIBrightness"));
    int value = str.toInt();
    value = value + 5;
    if (value >= 100) {
      value = 100;
    }
    uiState()->scene.brightness = value;
    QString values = QString::number(value);
    params.put("KisaUIBrightness", values.toStdString());
    refresh();
  });
  refresh();
}

void BrightnessControl::refresh() {
  QString option = QString::fromStdString(params.get("KisaUIBrightness"));
  if (option == "0") {
    label.setText(tr("Auto"));
  } else {
    label.setText(QString::fromStdString(params.get("KisaUIBrightness")));
  }
}

BrightnessOffControl::BrightnessOffControl() : AbstractControl(tr("Brightness at SCR Off(%)"), tr("When using the Device screen off function, the brightness is reduced according to the automatic brightness ratio."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaUIBrightnessOff"));
    int value = str.toInt();
    value = value - 5;
    if (value <= 0) {
      value = 0;
    }
    uiState()->scene.brightness_off = value;
    QString values = QString::number(value);
    params.put("KisaUIBrightnessOff", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaUIBrightnessOff"));
    int value = str.toInt();
    value = value + 5;
    if (value >= 100) {
      value = 100;
    }
    uiState()->scene.brightness_off = value;
    QString values = QString::number(value);
    params.put("KisaUIBrightnessOff", values.toStdString());
    refresh();
  });
  refresh();
}
void BrightnessOffControl::refresh() {
  QString option = QString::fromStdString(params.get("KisaUIBrightnessOff"));
  if (option == "0") {
    label.setText(tr("Dark"));
  } else if (option == "5") {
    label.setText(tr("MinBr"));
  } else {
    label.setText(QString::fromStdString(params.get("KisaUIBrightnessOff")));
  }
}
AutoScreenOff::AutoScreenOff() : AbstractControl(tr("Device SCR Off Timer"), tr("Turn off the Device screen or reduce brightness to protect the screen after driving starts. It automatically brightens or turns on when a touch or event occurs."), "../assets/offroad/icon_shell.png") 
{

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaAutoScreenOff"));
    int value = str.toInt();
    value = value - 1;
    if (value <= -3) {
      value = -3;
    }
    uiState()->scene.autoScreenOff = value;
    QString values = QString::number(value);
    params.put("KisaAutoScreenOff", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaAutoScreenOff"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 10) {
      value = 10;
    }
    uiState()->scene.autoScreenOff = value;
    QString values = QString::number(value);
    params.put("KisaAutoScreenOff", values.toStdString());
    refresh();
  });
  refresh();
}
void AutoScreenOff::refresh() 
{
  QString option = QString::fromStdString(params.get("KisaAutoScreenOff"));
  if (option == "-3") {
    label.setText(tr("AlwaysOn"));
  } else if (option == "-2") {
    label.setText(tr("5secs"));
  } else if (option == "-1") {
    label.setText(tr("15secs"));
  } else if (option == "0") {
    label.setText(tr("30secs"));
  } else {
    label.setText(QString::fromStdString(params.get("KisaAutoScreenOff")) + tr("min(s)"));
  }
}
ChargingMin::ChargingMin() : AbstractControl(tr("BAT MinCharging Value"), tr("Sets the minimum battery charge value."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaBatteryChargingMin"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 10) {
      value = 10;
    }
    QString values = QString::number(value);
    params.put("KisaBatteryChargingMin", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaBatteryChargingMin"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 90) {
      value = 90;
    }
    QString values = QString::number(value);
    params.put("KisaBatteryChargingMin", values.toStdString());
    refresh();
  });
  refresh();
}

void ChargingMin::refresh() {
  label.setText(QString::fromStdString(params.get("KisaBatteryChargingMin")));
}

ChargingMax::ChargingMax() : AbstractControl(tr("BAT MaxCharging Value"), tr("Sets the maximum battery charge value."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaBatteryChargingMax"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 10) {
      value = 10;
    }
    QString values = QString::number(value);
    params.put("KisaBatteryChargingMax", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaBatteryChargingMax"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 90) {
      value = 90;
    }
    QString values = QString::number(value);
    params.put("KisaBatteryChargingMax", values.toStdString());
    refresh();
  });
  refresh();
}

void ChargingMax::refresh() {
  label.setText(QString::fromStdString(params.get("KisaBatteryChargingMax")));
}

RecordCount::RecordCount() : AbstractControl(tr("Number of Recorded Files"), tr("Sets the maximum number of recording files. Check file size and max recording count to not exceed your storage."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("RecordingCount"));
    int value = str.toInt();
    value = value - 10;
    if (value <= 10) {
      value = 10;
    }
    QString values = QString::number(value);
    params.put("RecordingCount", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("RecordingCount"));
    int value = str.toInt();
    value = value + 10;
    if (value >= 1000) {
      value = 1000;
    }
    QString values = QString::number(value);
    params.put("RecordingCount", values.toStdString());
    refresh();
  });
  refresh();
}

void RecordCount::refresh() {
  label.setText(QString::fromStdString(params.get("RecordingCount")));
}

MonitoringMode::MonitoringMode() : AbstractControl(tr("Driver Monitoring Mode"), tr("Set the monitoring mode. In the case of preference/prevention of sleepiness and sleepiness prevention, you can send a warning message faster by adjusting (lowering) the threshold value below."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("◀");
  btnplus.setText("▶");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaMonitoringMode"));
    int value = str.toInt();
    value = value - 1;
    if (value <= -1) {
      value = 1;
    }
    QString values = QString::number(value);
    params.put("KisaMonitoringMode", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaMonitoringMode"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 2) {
      value = 0;
    }
    QString values = QString::number(value);
    params.put("KisaMonitoringMode", values.toStdString());
    refresh();
  });
  refresh();
}

void MonitoringMode::refresh() {
  QString option = QString::fromStdString(params.get("KisaMonitoringMode"));
  if (option == "0") {
    label.setText(tr("Default"));
  } else if (option == "1") {
    label.setText(tr("UnSleep"));
  }
}

MonitorEyesThreshold::MonitorEyesThreshold() : AbstractControl(tr("E2E EYE Threshold"), tr("Adjust the reference value for the eye detection range. Set the reference value for the value that suits you. When you close your eyes, you should set it lower than the distracted Eyes value. Default: 0.75"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaMonitorEyesThreshold"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 10) {
      value = 10;
    }
    QString values = QString::number(value);
    params.put("KisaMonitorEyesThreshold", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaMonitorEyesThreshold"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 100) {
      value = 100;
    }
    QString values = QString::number(value);
    params.put("KisaMonitorEyesThreshold", values.toStdString());
    refresh();
  });
  refresh();
}

void MonitorEyesThreshold::refresh() {
  auto strs = QString::fromStdString(params.get("KisaMonitorEyesThreshold"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.01;
  QString valuefs = QString::number(valuef);
  label.setText(QString::fromStdString(valuefs.toStdString()));
}

NormalEyesThreshold::NormalEyesThreshold() : AbstractControl(tr("Normal EYE Threshold"), tr("Adjust the eye recognition reference value. Lower the value when the recognition rate is low. Default: 0.5"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaMonitorNormalEyesThreshold"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 10) {
      value = 10;
    }
    QString values = QString::number(value);
    params.put("KisaMonitorNormalEyesThreshold", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaMonitorNormalEyesThreshold"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 100) {
      value = 100;
    }
    QString values = QString::number(value);
    params.put("KisaMonitorNormalEyesThreshold", values.toStdString());
    refresh();
  });
  refresh();
}

void NormalEyesThreshold::refresh() {
  auto strs = QString::fromStdString(params.get("KisaMonitorNormalEyesThreshold"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.01;
  QString valuefs = QString::number(valuef);
  label.setText(QString::fromStdString(valuefs.toStdString()));
}

BlinkThreshold::BlinkThreshold() : AbstractControl(tr("Blink Threshold"), tr("Adjust the recognition value for the degree of blinking. When you close your eyes, check BlinkProb and lower the value. Default: 0.5"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaMonitorBlinkThreshold"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 10) {
      value = 10;
    }
    QString values = QString::number(value);
    params.put("KisaMonitorBlinkThreshold", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaMonitorBlinkThreshold"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 100) {
      value = 100;
    }
    QString values = QString::number(value);
    params.put("KisaMonitorBlinkThreshold", values.toStdString());
    refresh();
  });
  refresh();
}

void BlinkThreshold::refresh() {
  auto strs = QString::fromStdString(params.get("KisaMonitorBlinkThreshold"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.01;
  QString valuefs = QString::number(valuef);
  label.setText(QString::fromStdString(valuefs.toStdString()));
}

//Driving
CruisemodeSelInit::CruisemodeSelInit() : AbstractControl(tr("Cruise Start Mode"), tr("Set the cruise start mode. OP mode/dist+curve/dist only/curve only/one-way 1 lane/safetycam deceleration Only. op mode:no button speed control, dist+curve:use button speed control in the inter-vehicle distance and curve section, dist only:curve only:curve one-way speed only, one-way speed control"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("◀");
  btnplus.setText("▶");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("CruiseStatemodeSelInit"));
    int value = str.toInt();
    value = value - 1;
    if (value <= -1) {
      value = 5;
    }
    QString values = QString::number(value);
    params.put("CruiseStatemodeSelInit", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("CruiseStatemodeSelInit"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 6) {
      value = 0;
    }
    QString values = QString::number(value);
    params.put("CruiseStatemodeSelInit", values.toStdString());
    refresh();
  });
  refresh();
}

void CruisemodeSelInit::refresh() {
  QString option = QString::fromStdString(params.get("CruiseStatemodeSelInit"));
  if (option == "0") {
    label.setText(tr("OP Stock"));
  } else if (option == "1") {
    label.setText(tr("Dist+Curv"));
  } else if (option == "2") {
    label.setText(tr("DistOnly"));
  } else if (option == "3") {
    label.setText(tr("CurvOnly"));
  } else if (option == "4") {
    label.setText(tr("OneWay"));
  } else {
    label.setText(tr("CamOnly"));
  }
}

LaneChangeSpeed::LaneChangeSpeed() : AbstractControl(tr("LaneChange On/Off/Spd"), tr("On/Off lane change(push (-) btn till Off value) and set the lane changeable speed. This value can be kph or mph."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaLaneChangeSpeed"));
    int value = str.toInt();
    value = value - 1;
    if (value <= -1) {
      value = 100;
    }
    QString values = QString::number(value);
    params.put("KisaLaneChangeSpeed", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaLaneChangeSpeed"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 101) {
      value = 0;
    }
    QString values = QString::number(value);
    params.put("KisaLaneChangeSpeed", values.toStdString());
    refresh();
  });
  refresh();
}

void LaneChangeSpeed::refresh() {
  QString option = QString::fromStdString(params.get("KisaLaneChangeSpeed"));
  if (option == "0") {
    label.setText(tr("Off"));
  } else {
    label.setText(QString::fromStdString(params.get("KisaLaneChangeSpeed")));
  }
}

LaneChangeDelay::LaneChangeDelay() : AbstractControl(tr("LaneChange Delay"), tr("Set the delay time after turn signal operation before lane change."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaAutoLaneChangeDelay"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 0) {
      value = 0;
    }
    QString values = QString::number(value);
    params.put("KisaAutoLaneChangeDelay", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaAutoLaneChangeDelay"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 5) {
      value = 5;
    }
    QString values = QString::number(value);
    params.put("KisaAutoLaneChangeDelay", values.toStdString());
    refresh();
  });
  refresh();
}

void LaneChangeDelay::refresh() {
  QString option = QString::fromStdString(params.get("KisaAutoLaneChangeDelay"));
  if (option == "0") {
    label.setText(tr("Nudge"));
  } else if (option == "1") {
    label.setText(tr("RightNow"));
  } else if (option == "2") {
    label.setText(tr("0.5sec"));
  } else if (option == "3") {
    label.setText(tr("1sec"));
  } else if (option == "4") {
    label.setText(tr("1.5sec"));
  } else {
    label.setText(tr("2secs"));
  }
}

LeftCurvOffset::LeftCurvOffset() : AbstractControl(tr("LeftCurv Offset"), tr("Adjust the position of the vehicle in the curve section. (-value: move the car to the left, +value: move the car to the right)"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("LeftCurvOffsetAdj"));
    int value = str.toInt();
    value = value - 1;
    if (value <= -50) {
      value = -50;
    }
    QString values = QString::number(value);
    params.put("LeftCurvOffsetAdj", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("LeftCurvOffsetAdj"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 50) {
      value = 50;
    }
    QString values = QString::number(value);
    params.put("LeftCurvOffsetAdj", values.toStdString());
    refresh();
  });
  refresh();
}

void LeftCurvOffset::refresh() {
  label.setText(QString::fromStdString(params.get("LeftCurvOffsetAdj")));
}

RightCurvOffset::RightCurvOffset() : AbstractControl(tr("RightCurv Offset"), tr("Adjust the position of the vehicle in the curve section. (-value: move the car to the left, +value: move the car to the right)"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("RightCurvOffsetAdj"));
    int value = str.toInt();
    value = value - 1;
    if (value <= -50) {
      value = -50;
    }
    QString values = QString::number(value);
    params.put("RightCurvOffsetAdj", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("RightCurvOffsetAdj"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 50) {
      value = 50;
    }
    QString values = QString::number(value);
    params.put("RightCurvOffsetAdj", values.toStdString());
    refresh();
  });
  refresh();
}

void RightCurvOffset::refresh() {
  label.setText(QString::fromStdString(params.get("RightCurvOffsetAdj")));
}


SteerAngleCorrection::SteerAngleCorrection() : AbstractControl(tr("Str Angle Adjust"), tr("On the straight path, adjust the SteerAngle zero to zero the current steering angle. ex) Set it to 0.5 degrees Celsius for a straight line, and -0.5 degrees Celsius for -0.5 degrees Celsius."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaSteerAngleCorrection"));
    int value = str.toInt();
    value = value - 1;
    if (value <= -50) {
      value = -50;
    }
    QString values = QString::number(value);
    params.put("KisaSteerAngleCorrection", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaSteerAngleCorrection"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 50) {
      value = 50;
    }
    QString values = QString::number(value);
    params.put("KisaSteerAngleCorrection", values.toStdString());
    refresh();
  });
  refresh();
}

void SteerAngleCorrection::refresh() {
  auto strs = QString::fromStdString(params.get("KisaSteerAngleCorrection"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.1;
  QString valuefs = QString::number(valuef);
  label.setText(QString::fromStdString(valuefs.toStdString()));
}

SpeedLimitOffset::SpeedLimitOffset() : AbstractControl(tr("SpeedLimit Offset"), tr("During safetycam deceleration, it decelerates by compensating for the difference between GPS speed and real speed."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btn.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btn.setFixedSize(110, 100);
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btn);
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btn, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaSpeedLimitOffsetOption"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 4) {
      value = 0;
    }
    QString values = QString::number(value);
    params.put("KisaSpeedLimitOffsetOption", values.toStdString());
    refresh();
  });

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaSpeedLimitOffset"));
    int value = str.toInt();
    value = value - 1;
    if (value <= -30) {
      value = -30;
    }
    QString values = QString::number(value);
    //uiState()->speed_lim_off = value;
    params.put("KisaSpeedLimitOffset", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaSpeedLimitOffset"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 30) {
      value = 30;
    }
    QString values = QString::number(value);
    //uiState()->speed_lim_off = value;
    params.put("KisaSpeedLimitOffset", values.toStdString());
    refresh();
  });
  refresh();
}

void SpeedLimitOffset::refresh() {
  auto strs = QString::fromStdString(params.get("KisaSpeedLimitOffsetOption"));
  if (strs == "0") {
    btn.setText("%");
  } else if (strs == "1") {
    btn.setText("±");
  } else if (strs == "2") {
    btn.setText("C");
  } else {
    btn.setText("D");}
  label.setText(QString::fromStdString(params.get("KisaSpeedLimitOffset")));
}
RESChoice::RESChoice() : AbstractControl(tr("AutoRES Option"), tr("Sets the auto RES option. 1. Adjust the temporary cruise speed, 2. Adjust the set speed itself according to the presence or absence of a preceding car. 3. Adjust the cruise speed if there is a preceding car, and adjust the set speed if there is no preceding car. Please note that the automatic RES may not work well depending on the conditions."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("◀");
  btnplus.setText("▶");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("AutoResOption"));
    int value = str.toInt();
    value = value - 1;
    if (value <= -1) {
      value = 2;
    }
    QString values = QString::number(value);
    params.put("AutoResOption", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("AutoResOption"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 3) {
      value = 0;
    }
    QString values = QString::number(value);
    params.put("AutoResOption", values.toStdString());
    refresh();
  });
  refresh();
}

void RESChoice::refresh() {
  QString option = QString::fromStdString(params.get("AutoResOption"));
  if (option == "0") {
    label.setText(tr("CruiseSet"));
  } else if (option == "1") {
    label.setText(tr("MaxSpeedSet"));
  } else {
    label.setText(tr("AUTO(LeadCar)"));
  }
}

AutoResCondition::AutoResCondition() : AbstractControl(tr("AutoRES Condition"), tr("Sets the automatic RES condition. When the brake is released/operated when the accelerator pedal is operated."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("◀");
  btnplus.setText("▶");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("AutoResCondition"));
    int value = str.toInt();
    value = value - 1;
    if (value <= -1) {
      value = 1;
    }
    QString values = QString::number(value);
    params.put("AutoResCondition", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("AutoResCondition"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 2) {
      value = 0;
    }
    QString values = QString::number(value);
    params.put("AutoResCondition", values.toStdString());
    refresh();
  });
  refresh();
}

void AutoResCondition::refresh() {
  QString option = QString::fromStdString(params.get("AutoResCondition"));
  if (option == "0") {
    label.setText(tr("RelBrake"));
  } else {
    label.setText(tr("OnGas"));
  }
}

AutoResLimitTime::AutoResLimitTime() : AbstractControl(tr("AutoRES Allow(sec)"), tr("Adjust the automatic RES allowance time. Automatic RES operates only within the set time after the cruise is released."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("AutoResLimitTime"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 0) {
      value = 0;
    }
    QString values = QString::number(value);
    params.put("AutoResLimitTime", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("AutoResLimitTime"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 60) {
      value = 60;
    }
    QString values = QString::number(value);
    params.put("AutoResLimitTime", values.toStdString());
    refresh();
  });
  refresh();
}

void AutoResLimitTime::refresh() {
  QString option = QString::fromStdString(params.get("AutoResLimitTime"));
  if (option == "0") {
    label.setText(tr("NoLimit"));
  } else {
    label.setText(QString::fromStdString(params.get("AutoResLimitTime")));
  }
  btnminus.setText("-");
  btnplus.setText("+");
}

AutoEnableSpeed::AutoEnableSpeed() : AbstractControl(tr("Auto Engage Speed"), tr("Set the automatic engage speed."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("AutoEnableSpeed"));
    int value = str.toInt();
    value = value - 1;
    if (value <= -1) {
      value = -1;
    }
    QString values = QString::number(value);
    params.put("AutoEnableSpeed", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("AutoEnableSpeed"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 30) {
      value = 30;
    }
    QString values = QString::number(value);
    params.put("AutoEnableSpeed", values.toStdString());
    refresh();
  });
  refresh();
}

void AutoEnableSpeed::refresh() {
  QString option = QString::fromStdString(params.get("AutoEnableSpeed"));
  if (option == "-1") {
    label.setText(tr("atDGear"));
  } else if (option == "0") {
    label.setText(tr("atDepart"));
  } else {
    label.setText(QString::fromStdString(params.get("AutoEnableSpeed")));
  }
  btnminus.setText("-");
  btnplus.setText("+");
}

CamDecelDistAdd::CamDecelDistAdd() : AbstractControl(tr("SafetyCamDist Adj(%)"), tr("Reduce or increase the deceleration start distance during deceleration of the safety section (+ value: deceleration start from a long distance, -value: deceleration start at a short distance) = interpolation value X interpolation value X reduction/increase ratio according to difference between current speed and cam speed."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("SafetyCamDecelDistGain"));
    int value = str.toInt();
    value = value - 5;
    if (value <= -100) {
      value = -100;
    }
    QString values = QString::number(value);
    params.put("SafetyCamDecelDistGain", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("SafetyCamDecelDistGain"));
    int value = str.toInt();
    value = value + 5;
    if (value >= 100) {
      value = 100;
    }
    QString values = QString::number(value);
    params.put("SafetyCamDecelDistGain", values.toStdString());
    refresh();
  });
  refresh();
}

void CamDecelDistAdd::refresh() {
  QString option = QString::fromStdString(params.get("SafetyCamDecelDistGain"));
  if (option == "0") {
    label.setText(tr("Default"));
  } else {
    label.setText(QString::fromStdString(params.get("SafetyCamDecelDistGain")));
  }
  btnminus.setText("-");
  btnplus.setText("+");
}

//튜닝
CameraOffset::CameraOffset() : AbstractControl(tr("CameraOffset"), tr("Sets the CameraOffset value. (low value: Car to Move Left, high value: Car to Move Right)"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("CameraOffsetAdj"));
    int value = str.toInt();
    value = value - 5;
    if (value <= -1000) {
      value = -1000;
    }
    QString values = QString::number(value);
    params.put("CameraOffsetAdj", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("CameraOffsetAdj"));
    int value = str.toInt();
    value = value + 5;
    if (value >= 1000) {
      value = 1000;
    }
    QString values = QString::number(value);
    params.put("CameraOffsetAdj", values.toStdString());
    refresh();
  });
  refresh();
}

void CameraOffset::refresh() {
  auto strs = QString::fromStdString(params.get("CameraOffsetAdj"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.001;
  QString valuefs = QString::number(valuef);
  label.setText(QString::fromStdString(valuefs.toStdString()));
}

PathOffset::PathOffset() : AbstractControl(tr("PathOffset"), tr("Sets the PathOffset value. (low value: Car to Move Left, high value: Car to Move Right)"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("PathOffsetAdj"));
    int value = str.toInt();
    value = value - 5;
    if (value <= -1000) {
      value = -1000;
    }
    QString values = QString::number(value);
    params.put("PathOffsetAdj", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("PathOffsetAdj"));
    int value = str.toInt();
    value = value + 5;
    if (value >= 1000) {
      value = 1000;
    }
    QString values = QString::number(value);
    params.put("PathOffsetAdj", values.toStdString());
    refresh();
  });
  refresh();
}

void PathOffset::refresh() {
  auto strs = QString::fromStdString(params.get("PathOffsetAdj"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.001;
  QString valuefs = QString::number(valuef);
  label.setText(QString::fromStdString(valuefs.toStdString()));
}

SRBaseControl::SRBaseControl() : AbstractControl(tr("SteerRatio"), tr("Sets the SteerRatio default value."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btndigit.setStyleSheet(R"(
    QPushButton {
      padding: 0;
      border-radius: 50px;
      font-size: 35px;
      font-weight: 500;
      color: #E4E4E4;
      background-color: #393939;
    }
    QPushButton:pressed {
      background-color: #ababab;
    }
  )");
  btnminus.setStyleSheet(R"(
    QPushButton {
      padding: 0;
      border-radius: 50px;
      font-size: 35px;
      font-weight: 500;
      color: #E4E4E4;
      background-color: #393939;
    }
    QPushButton:pressed {
      background-color: #ababab;
    }
  )");
  btnplus.setStyleSheet(R"(
    QPushButton {
      padding: 0;
      border-radius: 50px;
      font-size: 35px;
      font-weight: 500;
      color: #E4E4E4;
      background-color: #393939;
    }
    QPushButton:pressed {
      background-color: #ababab;
    }
  )");
  btndigit.setFixedSize(150, 100);
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  hlayout->addWidget(&btndigit);
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);
  btndigit.setText("0.01");
  btnminus.setText("-");
  btnplus.setText("+");

  QObject::connect(&btndigit, &QPushButton::clicked, [=]() {
    digit = digit * 10;
    if (digit >= 11) {
      digit = 0.01;
    }
    QString level = QString::number(digit);
    btndigit.setText(level);
  });

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("SteerRatioAdj"));
    int value = str.toInt();
    value = value - (digit*100);
    if (value <= 800) {
      value = 800;
    }
    QString values = QString::number(value);
    params.put("SteerRatioAdj", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("SteerRatioAdj"));
    int value = str.toInt();
    value = value + (digit*100);
    if (value >= 2500) {
      value = 2500;
    }
    QString values = QString::number(value);
    params.put("SteerRatioAdj", values.toStdString());
    refresh();
  });
  refresh();
}

void SRBaseControl::refresh() {
  auto strs = QString::fromStdString(params.get("SteerRatioAdj"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.01;
  QString valuefs = QString::number(valuef);
  label.setText(QString::fromStdString(valuefs.toStdString()));
}

SteerActuatorDelay::SteerActuatorDelay() : AbstractControl(tr("SteerActuatorDelay"), tr("Adjust the SteerActuatorDelay value."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("SteerActuatorDelayAdj"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 1) {
      value = 1;
    }
    QString values = QString::number(value);
    params.put("SteerActuatorDelayAdj", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("SteerActuatorDelayAdj"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 100) {
      value = 100;
    }
    QString values = QString::number(value);
    params.put("SteerActuatorDelayAdj", values.toStdString());
    refresh();
  });
  refresh();
}

void SteerActuatorDelay::refresh() {
  auto strs = QString::fromStdString(params.get("SteerActuatorDelayAdj"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.01;
  QString valuefs = QString::number(valuef);
  label.setText(QString::fromStdString(valuefs.toStdString()));
}

// SteerRateCost::SteerRateCost() : AbstractControl(tr("SteerRateCost"), tr("Adjust the SteerRateCost value."), "../assets/offroad/icon_shell.png") {

//   label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
//   label.setStyleSheet("color: #e0e879");
//   hlayout->addWidget(&label);

//   btnminus.setStyleSheet(R"(
//     padding: 0;
//     border-radius: 50px;
//     font-size: 35px;
//     font-weight: 500;
//     color: #E4E4E4;
//     background-color: #393939;
//   )");
//   btnplus.setStyleSheet(R"(
//     padding: 0;
//     border-radius: 50px;
//     font-size: 35px;
//     font-weight: 500;
//     color: #E4E4E4;
//     background-color: #393939;
//   )");
//   btnminus.setFixedSize(150, 100);
//   btnplus.setFixedSize(150, 100);
//   btnminus.setText("－");
//   btnplus.setText("＋");
//   hlayout->addWidget(&btnminus);
//   hlayout->addWidget(&btnplus);

//   QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
//     auto str = QString::fromStdString(params.get("SteerRateCostAdj"));
//     int value = str.toInt();
//     value = value - 1;
//     if (value <= 1) {
//       value = 1;
//     }
//     QString values = QString::number(value);
//     params.put("SteerRateCostAdj", values.toStdString());
//     refresh();
//   });
  
//   QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
//     auto str = QString::fromStdString(params.get("SteerRateCostAdj"));
//     int value = str.toInt();
//     value = value + 1;
//     if (value >= 200) {
//       value = 200;
//     }
//     QString values = QString::number(value);
//     params.put("SteerRateCostAdj", values.toStdString());
//     refresh();
//   });
//   refresh();
// }

// void SteerRateCost::refresh() {
//   auto strs = QString::fromStdString(params.get("SteerRateCostAdj"));
//   int valuei = strs.toInt();
//   float valuef = valuei * 0.01;
//   QString valuefs = QString::number(valuef);
//   label.setText(QString::fromStdString(valuefs.toStdString()));
// }

SteerLimitTimer::SteerLimitTimer() : AbstractControl(tr("SteerLimitTimer"), tr("Adjust the SteerLimitTimer value."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("SteerLimitTimerAdj"));
    int value = str.toInt();
    value = value - 5;
    if (value <= 0) {
      value = 0;
    }
    QString values = QString::number(value);
    params.put("SteerLimitTimerAdj", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("SteerLimitTimerAdj"));
    int value = str.toInt();
    value = value + 5;
    if (value >= 300) {
      value = 300;
    }
    QString values = QString::number(value);
    params.put("SteerLimitTimerAdj", values.toStdString());
    refresh();
  });
  refresh();
}

void SteerLimitTimer::refresh() {
  auto strs = QString::fromStdString(params.get("SteerLimitTimerAdj"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.01;
  QString valuefs = QString::number(valuef);
  label.setText(QString::fromStdString(valuefs.toStdString()));
}

TireStiffnessFactor::TireStiffnessFactor() : AbstractControl(tr("TireStiffnessFactor"), tr("Adjust the TireStiffnessFactor value."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("TireStiffnessFactorAdj"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 1) {
      value = 1;
    }
    QString values = QString::number(value);
    params.put("TireStiffnessFactorAdj", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("TireStiffnessFactorAdj"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 200) {
      value = 200;
    }
    QString values = QString::number(value);
    params.put("TireStiffnessFactorAdj", values.toStdString());
    refresh();
  });
  refresh();
}

void TireStiffnessFactor::refresh() {
  auto strs = QString::fromStdString(params.get("TireStiffnessFactorAdj"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.01;
  QString valuefs = QString::number(valuef);
  label.setText(QString::fromStdString(valuefs.toStdString()));
}

SteerMax::SteerMax() : AbstractControl(tr("SteerMax"), tr("Adjust the SteerMax value."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);
  btnminus.setText("－");
  btnplus.setText("＋");

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("SteerMaxAdj"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 200) {
      value = 200;
    }
    QString values = QString::number(value);
    params.put("SteerMaxAdj", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str1 = QString::fromStdString(params.get("SteerMaxAdj"));
    int value = str1.toInt();
    value = value + 1;
    if (value >= 384) {
      value = 384;
    }
    QString values = QString::number(value);
    params.put("SteerMaxAdj", values.toStdString());
    refresh();
  });
  refresh();
}

void SteerMax::refresh() {
  label.setText(QString::fromStdString(params.get("SteerMaxAdj")));
}

SteerDeltaUp::SteerDeltaUp() : AbstractControl(tr("SteerDeltaUp"), tr("Adjust the SteerDeltaUp value."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);
  btnminus.setText("－");
  btnplus.setText("＋");

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("SteerDeltaUpAdj"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 1) {
      value = 1;
    }
    QString values = QString::number(value);
    params.put("SteerDeltaUpAdj", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str1 = QString::fromStdString(params.get("SteerDeltaUpAdj"));
    int value = str1.toInt();
    value = value + 1;
    if (value >= 3) {
      value = 3;
    }
    QString values = QString::number(value);
    params.put("SteerDeltaUpAdj", values.toStdString());
    refresh();
  });
  refresh();
}

void SteerDeltaUp::refresh() {
  label.setText(QString::fromStdString(params.get("SteerDeltaUpAdj")));
}

SteerDeltaDown::SteerDeltaDown() : AbstractControl(tr("SteerDeltaDown"), tr("Adjust the SteerDeltaDown value."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);
  btnminus.setText("－");
  btnplus.setText("＋");

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("SteerDeltaDownAdj"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 1) {
      value = 1;
    }
    QString values = QString::number(value);
    params.put("SteerDeltaDownAdj", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str1 = QString::fromStdString(params.get("SteerDeltaDownAdj"));
    int value = str1.toInt();
    value = value + 1;
    if (value >= 7) {
      value = 7;
    }
    QString values = QString::number(value);
    params.put("SteerDeltaDownAdj", values.toStdString());
    refresh();
  });
  refresh();
}

void SteerDeltaDown::refresh() {
  label.setText(QString::fromStdString(params.get("SteerDeltaDownAdj")));
}

SteerThreshold::SteerThreshold() : AbstractControl(tr("SteerThreshold"), tr("Adjust the SteerThreshold value."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("SteerThreshold"));
    int value = str.toInt();
    value = value - 10;
    if (value <= 50) {
      value = 50;
    }
    QString values = QString::number(value);
    params.put("SteerThreshold", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("SteerThreshold"));
    int value = str.toInt();
    value = value + 10;
    if (value >= 300) {
      value = 300;
    }
    QString values = QString::number(value);
    params.put("SteerThreshold", values.toStdString());
    refresh();
  });
  refresh();
}

void SteerThreshold::refresh() {
  label.setText(QString::fromStdString(params.get("SteerThreshold")));
}

//제어
LateralControl::LateralControl() : AbstractControl(tr("LatControl(Reboot)"), tr("Set the steering control method(PID/INDI/LQR/TORQUE). Reboot Required."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("◀");
  btnplus.setText("▶");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  auto str = QString::fromStdString(params.get("LateralControlMethod"));
  latcontrol = str.toInt();  

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    latcontrol--;
    if (latcontrol < 0)  
        latcontrol = 4;

    QString latcontrols = QString::number(latcontrol);
    params.put("LateralControlMethod", latcontrols.toStdString());
    refresh();
  });

  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    latcontrol++;

    if (latcontrol > 4) 
      latcontrol = 0;
    
    QString latcontrols = QString::number(latcontrol);
    params.put("LateralControlMethod", latcontrols.toStdString());
    refresh();
  });
  refresh();
}

void LateralControl::refresh() 
{

  QString  str;
  switch( latcontrol )
  {
    case 0 : str = "0.PID"; break;
    case 1 : str = "1.INDI";  break;
    case 2 : str = "2.LQR";  break;
    case 3 : str = "3.TORQUE";  break;
    case 4 : str = "4.MULTI";  break;
  }

  label.setText( str );
}

PidKp::PidKp() : AbstractControl(tr("Kp"), tr("Adjust Kp"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("PidKp"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 1) {
      value = 1;
    }
    QString values = QString::number(value);
    params.put("PidKp", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("PidKp"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 50) {
      value = 50;
    }
    QString values = QString::number(value);
    params.put("PidKp", values.toStdString());
    refresh();
  });
  refresh();
}

void PidKp::refresh() {
  auto strs = QString::fromStdString(params.get("PidKp"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.01;
  QString valuefs = QString::number(valuef);
  label.setText(QString::fromStdString(valuefs.toStdString()));
}

PidKi::PidKi() : AbstractControl(tr("Ki"), tr("Adjust Ki"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("PidKi"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 1) {
      value = 1;
    }
    QString values = QString::number(value);
    params.put("PidKi", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("PidKi"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 100) {
      value = 100;
    }
    QString values = QString::number(value);
    params.put("PidKi", values.toStdString());
    refresh();
  });
  refresh();
}

void PidKi::refresh() {
  auto strs = QString::fromStdString(params.get("PidKi"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.001;
  QString valuefs = QString::number(valuef);
  label.setText(QString::fromStdString(valuefs.toStdString()));
}

PidKd::PidKd() : AbstractControl(tr("Kd"), tr("Adjust Kd"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("PidKd"));
    int value = str.toInt();
    value = value - 5;
    if (value <= 0) {
      value = 0;
    }
    QString values = QString::number(value);
    params.put("PidKd", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("PidKd"));
    int value = str.toInt();
    value = value + 5;
    if (value >= 300) {
      value = 300;
    }
    QString values = QString::number(value);
    params.put("PidKd", values.toStdString());
    refresh();
  });
  refresh();
}

void PidKd::refresh() {
  auto strs = QString::fromStdString(params.get("PidKd"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.01;
  QString valuefs = QString::number(valuef);
  label.setText(QString::fromStdString(valuefs.toStdString()));
}

PidKf::PidKf() : AbstractControl(tr("Kf"), tr("Adjust Kf"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("PidKf"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 1) {
      value = 1;
    }
    QString values = QString::number(value);
    params.put("PidKf", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("PidKf"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 50) {
      value = 50;
    }
    QString values = QString::number(value);
    params.put("PidKf", values.toStdString());
    refresh();
  });
  refresh();
}

void PidKf::refresh() {
  auto strs = QString::fromStdString(params.get("PidKf"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.00001;
  QString valuefs = QString::number(valuef, 'f', 5);
  label.setText(QString::fromStdString(valuefs.toStdString()));
}

OuterLoopGain::OuterLoopGain() : AbstractControl(tr("OuterLoopGain"), tr("Adjust OuterLoopGain"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("OuterLoopGain"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 1) {
      value = 1;
    }
    QString values = QString::number(value);
    params.put("OuterLoopGain", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("OuterLoopGain"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 200) {
      value = 200;
    }
    QString values = QString::number(value);
    params.put("OuterLoopGain", values.toStdString());
    refresh();
  });
  refresh();
}

void OuterLoopGain::refresh() {
  auto strs = QString::fromStdString(params.get("OuterLoopGain"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.1;
  QString valuefs = QString::number(valuef);
  label.setText(QString::fromStdString(valuefs.toStdString()));
}

InnerLoopGain::InnerLoopGain() : AbstractControl(tr("InnerLoopGain"), tr("Adjust InnerLoopGain"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("InnerLoopGain"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 1) {
      value = 1;
    }
    QString values = QString::number(value);
    params.put("InnerLoopGain", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("InnerLoopGain"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 200) {
      value = 200;
    }
    QString values = QString::number(value);
    params.put("InnerLoopGain", values.toStdString());
    refresh();
  });
  refresh();
}

void InnerLoopGain::refresh() {
  auto strs = QString::fromStdString(params.get("InnerLoopGain"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.1;
  QString valuefs = QString::number(valuef);
  label.setText(QString::fromStdString(valuefs.toStdString()));
}

TimeConstant::TimeConstant() : AbstractControl(tr("TimeConstant"), tr("Adjust TimeConstant"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("TimeConstant"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 1) {
      value = 1;
    }
    QString values = QString::number(value);
    params.put("TimeConstant", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("TimeConstant"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 200) {
      value = 200;
    }
    QString values = QString::number(value);
    params.put("TimeConstant", values.toStdString());
    refresh();
  });
  refresh();
}

void TimeConstant::refresh() {
  auto strs = QString::fromStdString(params.get("TimeConstant"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.1;
  QString valuefs = QString::number(valuef);
  label.setText(QString::fromStdString(valuefs.toStdString()));
}

ActuatorEffectiveness::ActuatorEffectiveness() : AbstractControl(tr("ActuatorEffectiveness"), tr("Adjust ActuatorEffectiveness"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("ActuatorEffectiveness"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 1) {
      value = 1;
    }
    QString values = QString::number(value);
    params.put("ActuatorEffectiveness", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("ActuatorEffectiveness"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 200) {
      value = 200;
    }
    QString values = QString::number(value);
    params.put("ActuatorEffectiveness", values.toStdString());
    refresh();
  });
  refresh();
}

void ActuatorEffectiveness::refresh() {
  auto strs = QString::fromStdString(params.get("ActuatorEffectiveness"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.1;
  QString valuefs = QString::number(valuef);
  label.setText(QString::fromStdString(valuefs.toStdString()));
}

Scale::Scale() : AbstractControl(tr("Scale"), tr("Adjust Scale"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("Scale"));
    int value = str.toInt();
    value = value - 50;
    if (value <= 50) {
      value = 50;
    }
    QString values = QString::number(value);
    params.put("Scale", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("Scale"));
    int value = str.toInt();
    value = value + 50;
    if (value >= 5000) {
      value = 5000;
    }
    QString values = QString::number(value);
    params.put("Scale", values.toStdString());
    refresh();
  });
  refresh();
}

void Scale::refresh() {
  label.setText(QString::fromStdString(params.get("Scale")));
}

LqrKi::LqrKi() : AbstractControl(tr("LqrKi"), tr("Adjust ki"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("LqrKi"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 1) {
      value = 1;
    }
    QString values = QString::number(value);
    params.put("LqrKi", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("LqrKi"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 100) {
      value = 100;
    }
    QString values = QString::number(value);
    params.put("LqrKi", values.toStdString());
    refresh();
  });
  refresh();
}

void LqrKi::refresh() {
  auto strs = QString::fromStdString(params.get("LqrKi"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.001;
  QString valuefs = QString::number(valuef);
  label.setText(QString::fromStdString(valuefs.toStdString()));
}

DcGain::DcGain() : AbstractControl(tr("DcGain"), tr("Adjust DcGain"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("DcGain"));
    int value = str.toInt();
    value = value - 5;
    if (value <= 5) {
      value = 5;
    }
    QString values = QString::number(value);
    params.put("DcGain", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("DcGain"));
    int value = str.toInt();
    value = value + 5;
    if (value >= 500) {
      value = 500;
    }
    QString values = QString::number(value);
    params.put("DcGain", values.toStdString());
    refresh();
  });
  refresh();
}

void DcGain::refresh() {
  auto strs = QString::fromStdString(params.get("DcGain"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.00001;
  QString valuefs = QString::number(valuef);
  label.setText(QString::fromStdString(valuefs.toStdString()));
}

TorqueKp::TorqueKp() : AbstractControl(tr("Kp"), tr("Adjust Kp"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("TorqueKp"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 1) {
      value = 1;
    }
    QString values = QString::number(value);
    params.put("TorqueKp", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("TorqueKp"));
    auto str1 = QString::fromStdString(params.get("TorqueMaxLatAccel"));
    int value = str.toInt();
    int max_lat_accel = str1.toInt();
    value = value + 1;
    if (value >= max_lat_accel) {
      value = max_lat_accel;
    }
    QString values = QString::number(value);
    params.put("TorqueKp", values.toStdString());
    refresh();
  });
  refresh();
}

void TorqueKp::refresh() {
  auto strs = QString::fromStdString(params.get("TorqueKp"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.1;
  QString valuefs = QString::number(valuef);
  label.setText(QString::fromStdString(valuefs.toStdString()));
}

TorqueKf::TorqueKf() : AbstractControl(tr("Kf"), tr("Adjust Kf"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("TorqueKf"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 1) {
      value = 1;
    }
    QString values = QString::number(value);
    params.put("TorqueKf", values.toStdString());
    refresh();
  });
  
 QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("TorqueKf"));
    auto str1 = QString::fromStdString(params.get("TorqueMaxLatAccel"));
    int value = str.toInt();
    int max_lat_accel = str1.toInt();
    value = value + 1;
    if (value >= max_lat_accel) {
      value = max_lat_accel;
    }
    QString values = QString::number(value);
    params.put("TorqueKf", values.toStdString());
    refresh();
  });
  refresh();
}

void TorqueKf::refresh() {
  auto strs = QString::fromStdString(params.get("TorqueKf"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.1;
  QString valuefs = QString::number(valuef);
  label.setText(QString::fromStdString(valuefs.toStdString()));
}

TorqueKi::TorqueKi() : AbstractControl(tr("Ki"), tr("Adjust Ki"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("TorqueKi"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 1) {
      value = 1;
    }
    QString values = QString::number(value);
    params.put("TorqueKi", values.toStdString());
    refresh();
  });
  
 QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("TorqueKi"));
    auto str1 = QString::fromStdString(params.get("TorqueMaxLatAccel"));
    int value = str.toInt();
    int max_lat_accel = str1.toInt();
    value = value + 1;
    if (value >= max_lat_accel) {
      value = max_lat_accel;
    }
    QString values = QString::number(value);
    params.put("TorqueKi", values.toStdString());
    refresh();
  });
  refresh();
}

void TorqueKi::refresh() {
  auto strs = QString::fromStdString(params.get("TorqueKi"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.1;
  QString valuefs = QString::number(valuef);
  label.setText(QString::fromStdString(valuefs.toStdString()));
}

TorqueFriction::TorqueFriction() : AbstractControl(tr("Friction"), tr("Adjust Friction"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("TorqueFriction"));
    int value = str.toInt();
    value = value - 5;
    if (value <= 0) {
      value = 0;
    }
    QString values = QString::number(value);
    params.put("TorqueFriction", values.toStdString());
    refresh();
  });
  
 QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("TorqueFriction"));
    int value = str.toInt();
    value = value + 5;
    if (value >= 300) {
      value = 300;
    }
    QString values = QString::number(value);
    params.put("TorqueFriction", values.toStdString());
    refresh();
  });
  refresh();
}

void TorqueFriction::refresh() {
  auto strs = QString::fromStdString(params.get("TorqueFriction"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.001;
  QString valuefs = QString::number(valuef);
  label.setText(QString::fromStdString(valuefs.toStdString()));
}

TorqueMaxLatAccel::TorqueMaxLatAccel() : AbstractControl(tr("MaxLatAccel"), tr("Adjust MaxLatAccel"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("TorqueMaxLatAccel"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 1) {
      value = 1;
    }
    QString values = QString::number(value);
    params.put("TorqueMaxLatAccel", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("TorqueMaxLatAccel"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 50) {
      value = 50;
    }
    QString values = QString::number(value);
    params.put("TorqueMaxLatAccel", values.toStdString());
    refresh();
  });
  refresh();
}

void TorqueMaxLatAccel::refresh() {
  auto strs = QString::fromStdString(params.get("TorqueMaxLatAccel"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.1;
  QString valuefs = QString::number(valuef);
  label.setText(QString::fromStdString(valuefs.toStdString()));
}

TorqueAngDeadZone::TorqueAngDeadZone() : AbstractControl(tr("AngleDeadZone"), tr("Adjust TorqueAngDeadZone"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("TorqueAngDeadZone"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 0) {
      value = 0;
    }
    QString values = QString::number(value);
    params.put("TorqueAngDeadZone", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("TorqueAngDeadZone"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 50) {
      value = 50;
    }
    QString values = QString::number(value);
    params.put("TorqueAngDeadZone", values.toStdString());
    refresh();
  });
  refresh();
}

void TorqueAngDeadZone::refresh() {
  auto strs = QString::fromStdString(params.get("TorqueAngDeadZone"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.1;
  QString valuefs = QString::number(valuef);
  label.setText(QString::fromStdString(valuefs.toStdString()));
}

CruiseGapTR::CruiseGapTR() : AbstractControl(tr("CruiseGap"), tr("Adjust the inter-vehicle distance (TR) according to the cruise gap. TR refers to the time in seconds of collision with the car in front, and the larger it becomes, the farther it is from the car in front."), "") {
  QString dtr = QString::fromStdString(params.get("DynamicTRGap"));
  if (dtr == "0") {
    btn1.setStyleSheet(R"(
      padding: -10;
      border-radius: 35px;
      font-size: 30px;
      font-weight: 500;
      color: #E4E4E4;
      background-color: #393939;
    )");
    btn2.setStyleSheet(R"(
      padding: -10;
      border-radius: 35px;
      font-size: 30px;
      font-weight: 500;
      color: #E4E4E4;
      background-color: #393939;
    )");
    btn3.setStyleSheet(R"(
      padding: -10;
      border-radius: 35px;
      font-size: 30px;
      font-weight: 500;
      color: #E4E4E4;
      background-color: #393939;
    )");
    btn4.setStyleSheet(R"(
      padding: -10;
      border-radius: 35px;
      font-size: 30px;
      font-weight: 500;
      color: #E4E4E4;
      background-color: #393939;
    )");
  } else {
    btn1.setStyleSheet(R"(
      padding: 0;
      border-radius: 50px;
      font-size: 35px;
      font-weight: 500;
      color: #E4E4E4;
      background-color: #393939;
    )");
    btn2.setStyleSheet(R"(
      padding: 0;
      border-radius: 50px;
      font-size: 35px;
      font-weight: 500;
      color: #E4E4E4;
      background-color: #393939;
    )");
    btn3.setStyleSheet(R"(
      padding: 0;
      border-radius: 50px;
      font-size: 35px;
      font-weight: 500;
      color: #E4E4E4;
      background-color: #393939;
    )");
    btn4.setStyleSheet(R"(
      padding: 0;
      border-radius: 50px;
      font-size: 35px;
      font-weight: 500;
      color: #E4E4E4;
      background-color: #393939;
    )");
  }
  label1.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label1.setStyleSheet("color: #e0e879");
  label2.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label2.setStyleSheet("color: #e0e879");
  label3.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label3.setStyleSheet("color: #e0e879");
  label4.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label4.setStyleSheet("color: #e0e879");
  label1a.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label2a.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label3a.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label4a.setAlignment(Qt::AlignVCenter|Qt::AlignRight);

  if (dtr != "1") {
    hlayout->addWidget(&label1a);
    hlayout->addWidget(&label1);
    if (dtr == "0") {
      btn1.setFixedSize(110, 100);
      label1a.setText("1:");
    } else {
      btn1.setFixedSize(150, 100);
      label1a.setText("1S:");
    }
    hlayout->addWidget(&btn1);
  }
  if (dtr != "2") {
    hlayout->addWidget(&label2a);
    hlayout->addWidget(&label2);
    if (dtr == "0") {
      btn2.setFixedSize(110, 100);
      label2a.setText("2:");
    } else {
      btn2.setFixedSize(150, 100);
      label2a.setText("2S:");
    }
    hlayout->addWidget(&btn2);
  }
  if (dtr != "3") {
    hlayout->addWidget(&label3a);
    hlayout->addWidget(&label3);
    if (dtr == "0") {
      btn3.setFixedSize(110, 100);
      label3a.setText("3:");
    } else {
      btn3.setFixedSize(150, 100);
      label3a.setText("3S:");
    }
    hlayout->addWidget(&btn3);
  }
  if (dtr != "4") {
    hlayout->addWidget(&label4a);
    hlayout->addWidget(&label4);
    if (dtr == "0") {
      btn4.setFixedSize(110, 100);
      label4a.setText("4:");
    } else {
      btn4.setFixedSize(150, 100);
      label4a.setText("4S:");
    }
    hlayout->addWidget(&btn4);
  }

  QObject::connect(&btn1, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("CruiseGap1"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 13) {
      value = 7;
    }
    QString values = QString::number(value);
    params.put("CruiseGap1", values.toStdString());
    refresh1();
  });

  QObject::connect(&btn2, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("CruiseGap2"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 16) {
      value = 8;
    }
    QString values = QString::number(value);
    params.put("CruiseGap2", values.toStdString());
    refresh2();
  });
  
  QObject::connect(&btn3, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("CruiseGap3"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 20) {
      value = 10;
    }
    QString values = QString::number(value);
    params.put("CruiseGap3", values.toStdString());
    refresh3();
  });

  QObject::connect(&btn4, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("CruiseGap4"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 30) {
      value = 12;
    }
    QString values = QString::number(value);
    params.put("CruiseGap4", values.toStdString());
    refresh4();
  });

  refresh1();
  refresh2();
  refresh3();
  refresh4();
}

void CruiseGapTR::refresh1() {
  auto strs = QString::fromStdString(params.get("CruiseGap1"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.1;
  QString valuefs = QString::number(valuef);
  label1.setText(QString::fromStdString(valuefs.toStdString()));
  btn1.setText("▲");
}
void CruiseGapTR::refresh2() {
  auto strs = QString::fromStdString(params.get("CruiseGap2"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.1;
  QString valuefs = QString::number(valuef);
  label2.setText(QString::fromStdString(valuefs.toStdString()));
  btn2.setText("▲");
}
void CruiseGapTR::refresh3() {
  auto strs = QString::fromStdString(params.get("CruiseGap3"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.1;
  QString valuefs = QString::number(valuef);
  label3.setText(QString::fromStdString(valuefs.toStdString()));
  btn3.setText("▲");
}
void CruiseGapTR::refresh4() {
  auto strs = QString::fromStdString(params.get("CruiseGap4"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.1;
  QString valuefs = QString::number(valuef);
  label4.setText(QString::fromStdString(valuefs.toStdString()));
  btn4.setText("▲");
}

DynamicTRGap::DynamicTRGap() : AbstractControl(tr("Use DynamicTR"), tr("Use DynamicTR and assign it to the corresponding gap and adjust TR by speed below."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("◀");
  btnplus.setText("▶");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("DynamicTRGap"));
    int value = str.toInt();
    value = value - 1;
    if (value <= -1) {
      value = 4;
    }
    QString values = QString::number(value);
    params.put("DynamicTRGap", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("DynamicTRGap"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 5) {
      value = 0;
    }
    QString values = QString::number(value);
    params.put("DynamicTRGap", values.toStdString());
    refresh();
  });
  refresh();
}

void DynamicTRGap::refresh() {
  QString option = QString::fromStdString(params.get("DynamicTRGap"));
  if (option == "0") {
    label.setText(tr("UnUse"));
  } else if (option == "1") {
    label.setText(QString::fromStdString("■"));
  } else if (option == "2") {
    label.setText(QString::fromStdString("■■"));
  } else if (option == "3") {
    label.setText(QString::fromStdString("■■■"));
  } else {
    label.setText(QString::fromStdString("■■■■"));
  }
}

LCTimingFactor::LCTimingFactor() : AbstractControl("", "", "") {

  btn1.setStyleSheet(R"(
    padding: -10;
    border-radius: 35px;
    font-size: 30px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btn2.setStyleSheet(R"(
    padding: -10;
    border-radius: 35px;
    font-size: 30px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btn3.setStyleSheet(R"(
    padding: -10;
    border-radius: 35px;
    font-size: 30px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btn4.setStyleSheet(R"(
    padding: -10;
    border-radius: 35px;
    font-size: 30px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  label1.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  label1.setStyleSheet("color: #e0e879");
  label2.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  label2.setStyleSheet("color: #e0e879");
  label3.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  label3.setStyleSheet("color: #e0e879");
  label4.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  label4.setStyleSheet("color: #e0e879");
  label1a.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  label2a.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  label3a.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  label4a.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);

  hlayout->addWidget(&label1a);
  hlayout->addWidget(&label1);
  btn1.setFixedSize(145, 100);
  label1a.setText("30:");
  hlayout->addWidget(&btn1);
  hlayout->addWidget(&label2a);
  hlayout->addWidget(&label2);
  btn2.setFixedSize(145, 100);
  label2a.setText("60:");
  hlayout->addWidget(&btn2);
  hlayout->addWidget(&label3a);
  hlayout->addWidget(&label3);
  btn3.setFixedSize(145, 100);
  label3a.setText("80:");
  hlayout->addWidget(&btn3);
  hlayout->addWidget(&label4a);
  hlayout->addWidget(&label4);
  btn4.setFixedSize(145, 100);
  label4a.setText("110:");
  hlayout->addWidget(&btn4);

  QObject::connect(&btn1, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("LCTimingFactor30"));
    int value = str.toInt();
    auto str2 = QString::fromStdString(params.get("LCTimingFactor60"));
    int value2 = str2.toInt();
    auto str_ud = QString::fromStdString(params.get("LCTimingFactorUD"));
    if (str_ud == "1") {
      value = value + 5;
    } else {
      value = value - 5;
    }
    if (value >= value2) {
      value = value2;
    } else if (value <= 5) {
      value = 5;
    }
    QString values = QString::number(value);
    params.put("LCTimingFactor30", values.toStdString());
    refresh1();
  });

  QObject::connect(&btn2, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("LCTimingFactor60"));
    int value = str.toInt();
    auto str0 = QString::fromStdString(params.get("LCTimingFactor30"));
    int value0 = str0.toInt();
    auto str2 = QString::fromStdString(params.get("LCTimingFactor80"));
    int value2 = str2.toInt();
    auto str_ud = QString::fromStdString(params.get("LCTimingFactorUD"));
    if (str_ud == "1") {
      value = value + 5;
    } else {
      value = value - 5;
    }
    if (value >= value2) {
      value = value2;
    } else if (value <= value0) {
      value = value0;
    }
    QString values = QString::number(value);
    params.put("LCTimingFactor60", values.toStdString());
    refresh2();
  });
  
  QObject::connect(&btn3, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("LCTimingFactor80"));
    int value = str.toInt();
    auto str0 = QString::fromStdString(params.get("LCTimingFactor60"));
    int value0 = str0.toInt();
    auto str2 = QString::fromStdString(params.get("LCTimingFactor110"));
    int value2 = str2.toInt();
    auto str_ud = QString::fromStdString(params.get("LCTimingFactorUD"));
    if (str_ud == "1") {
      value = value + 5;
    } else {
      value = value - 5;
    }
    if (value >= value2) {
      value = value2;
    } else if (value <= value0) {
      value = value0;
    }
    QString values = QString::number(value);
    params.put("LCTimingFactor80", values.toStdString());
    refresh3();
  });

  QObject::connect(&btn4, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("LCTimingFactor110"));
    int value = str.toInt();
    auto str0 = QString::fromStdString(params.get("LCTimingFactor80"));
    int value0 = str0.toInt();
    auto str_ud = QString::fromStdString(params.get("LCTimingFactorUD"));
    if (str_ud == "1") {
      value = value + 5;
    } else {
      value = value - 5;
    }
    if (value <= value0) {
      value = value0;
    } else if (value >= 300) {
      value = 300;
    }
    QString values = QString::number(value);
    params.put("LCTimingFactor110", values.toStdString());
    refresh4();
  });

  refresh1();
  refresh2();
  refresh3();
  refresh4();
}

void LCTimingFactor::refresh1() {
  auto strs = QString::fromStdString(params.get("LCTimingFactor30"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.01;
  QString valuefs = QString::number(valuef);
  label1.setText(QString::fromStdString(valuefs.toStdString()));
  btn1.setText("↕");
}
void LCTimingFactor::refresh2() {
  auto strs = QString::fromStdString(params.get("LCTimingFactor60"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.01;
  QString valuefs = QString::number(valuef);
  label2.setText(QString::fromStdString(valuefs.toStdString()));
  btn2.setText("↕");
}
void LCTimingFactor::refresh3() {
  auto strs = QString::fromStdString(params.get("LCTimingFactor80"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.01;
  QString valuefs = QString::number(valuef);
  label3.setText(QString::fromStdString(valuefs.toStdString()));
  btn3.setText("↕");
}
void LCTimingFactor::refresh4() {
  auto strs = QString::fromStdString(params.get("LCTimingFactor110"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.01;
  QString valuefs = QString::number(valuef);
  label4.setText(QString::fromStdString(valuefs.toStdString()));
  btn4.setText("↕");
}

LCTimingFactorUD::LCTimingFactorUD() : AbstractControl(tr("LaneChange Time (km/h: value)"), tr("When changing lanes, adjust the timing of lane change for each speed. If you want a quick lane change, increase the value and lower the value if you want a slow lane change."), "../assets/offroad/icon_shell.png") {

  btn.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btn.setFixedSize(125, 100);
  btn2.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btn2.setFixedSize(150, 100);
  hlayout->addWidget(&btn2);
  hlayout->addWidget(&btn);

  QObject::connect(&btn2, &QPushButton::clicked, [=]() {
    bool stat = params.getBool("LCTimingFactorEnable");
    if (stat) {
      params.putBool("LCTimingFactorEnable", false);
    } else {
      params.putBool("LCTimingFactorEnable", true);
    }
    refresh2();
  });

  QObject::connect(&btn, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("LCTimingFactorUD"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 2) {
      value = 0;
    }
    QString values = QString::number(value);
    params.put("LCTimingFactorUD", values.toStdString());
    refresh();
  });
  refresh();
  refresh2();
}

void LCTimingFactorUD::refresh() {
  auto strs = QString::fromStdString(params.get("LCTimingFactorUD"));
  if (strs == "1") {
    btn.setText("↑");
  } else {
    btn.setText("↓");
  }
}

void LCTimingFactorUD::refresh2() {
  bool param = params.getBool("LCTimingFactorEnable");
  if (param) {
    btn2.setText("ON");
    btn2.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #00A12E;
    )");
  } else {
    btn2.setText("OFF");
    btn2.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
    )");
  }
}

LCTimingKeepFactor::LCTimingKeepFactor() : AbstractControl("", "", "") {
  btn1.setStyleSheet(R"(
    padding: -10;
    border-radius: 35px;
    font-size: 30px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btn2.setStyleSheet(R"(
    padding: -10;
    border-radius: 35px;
    font-size: 30px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btn3.setStyleSheet(R"(
    padding: -10;
    border-radius: 35px;
    font-size: 30px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btn4.setStyleSheet(R"(
    padding: -10;
    border-radius: 35px;
    font-size: 30px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  label1.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  label1.setStyleSheet("color: #e0e879");
  label2.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  label2.setStyleSheet("color: #e0e879");
  label1a.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  label2a.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  btn1.setText("－");
  btn2.setText("＋");
  btn3.setText("－");
  btn4.setText("＋");

  hlayout->addWidget(&label1a);
  btn1.setFixedSize(120, 100);
  btn2.setFixedSize(120, 100);
  label1a.setText("Left:");
  hlayout->addWidget(&label1);
  hlayout->addWidget(&btn1);
  hlayout->addWidget(&btn2);

  hlayout->addWidget(&label2a);
  btn3.setFixedSize(120, 100);
  btn4.setFixedSize(120, 100);
  label2a.setText("Right:");
  hlayout->addWidget(&label2);
  hlayout->addWidget(&btn3);
  hlayout->addWidget(&btn4);

  QObject::connect(&btn1, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("LCTimingKeepFactorLeft"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 1) {
      value = 1;
    }
    QString values = QString::number(value);
    params.put("LCTimingKeepFactorLeft", values.toStdString());
    refresh1();
  });
  
  QObject::connect(&btn2, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("LCTimingKeepFactorLeft"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 50) {
      value = 50;
    }
    QString values = QString::number(value);
    params.put("LCTimingKeepFactorLeft", values.toStdString());
    refresh1();
  });
  QObject::connect(&btn3, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("LCTimingKeepFactorRight"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 1) {
      value = 1;
    }
    QString values = QString::number(value);
    params.put("LCTimingKeepFactorRight", values.toStdString());
    refresh2();
  });
  
  QObject::connect(&btn4, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("LCTimingKeepFactorRight"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 50) {
      value = 50;
    }
    QString values = QString::number(value);
    params.put("LCTimingKeepFactorRight", values.toStdString());
    refresh2();
  });

  refresh1();
  refresh2();
}

void LCTimingKeepFactor::refresh1() {
  auto strs = QString::fromStdString(params.get("LCTimingKeepFactorLeft"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.001;
  QString valuefs = QString::number(valuef);
  label1.setText(QString::fromStdString(valuefs.toStdString()));
}
void LCTimingKeepFactor::refresh2() {
  auto strs = QString::fromStdString(params.get("LCTimingKeepFactorRight"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.001;
  QString valuefs = QString::number(valuef);
  label2.setText(QString::fromStdString(valuefs.toStdString()));
}


LCTimingKeepFactorUD::LCTimingKeepFactorUD() : AbstractControl(tr("LaneChange Keeping Time"), tr("Set a time to keep lane change. Low value makes changing time more so that your car move enough to target lane. If car is over to target lane too fast, increase a value."), "../assets/offroad/icon_shell.png") {

  btn.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btn.setFixedSize(150, 100);
  hlayout->addWidget(&btn);

  QObject::connect(&btn, &QPushButton::clicked, [=]() {
    bool stat = params.getBool("LCTimingKeepFactorEnable");
    if (stat) {
      params.putBool("LCTimingKeepFactorEnable", false);
    } else {
      params.putBool("LCTimingKeepFactorEnable", true);
    }
    refresh();
  });

  refresh();
}

void LCTimingKeepFactorUD::refresh() {
  bool param = params.getBool("LCTimingKeepFactorEnable");
  if (param) {
    btn.setText("ON");
    btn.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #00A12E;
    )");
  } else {
    btn.setText("OFF");
    btn.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  }
}

LiveSRPercent::LiveSRPercent() : AbstractControl(tr("LiveSR Adjust(%)"), tr("When using LiveSR, the learned value is arbitrarily adjusted (%) and used. -Value:Lower from learned value, +Value:Lower from learned value"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("LiveSteerRatioPercent"));
    int value = str.toInt();
    value = value - 1;
    if (value <= -50) {
      value = -50;
    }
    QString values = QString::number(value);
    params.put("LiveSteerRatioPercent", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("LiveSteerRatioPercent"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 50) {
      value = 50;
    }
    QString values = QString::number(value);
    params.put("LiveSteerRatioPercent", values.toStdString());
    refresh();
  });
  refresh();
}

void LiveSRPercent::refresh() {
  QString option = QString::fromStdString(params.get("LiveSteerRatioPercent"));
  if (option == "0") {
    label.setText(tr("Default"));
  } else {
    label.setText(QString::fromStdString(params.get("LiveSteerRatioPercent")));
  }
  btnminus.setText("-");
  btnplus.setText("+");
}

VCurvSpeedUD::VCurvSpeedUD() : AbstractControl(tr("VisionCurvDecel([CV] [TargetSpeed])"), tr("Adjust the curve deceleration speed according to the model speed(curvature). (interpolation and list value)"), "../assets/offroad/icon_shell.png") {
}

VCurvSpeed::VCurvSpeed() : AbstractControl("", "", "") {
  btn.setStyleSheet(R"(
    padding: -10;
    border-radius: 35px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  edit1.setStyleSheet(R"(
    background-color: grey;
    font-size: 55px;
    font-weight: 500;
    height: 120px;
  )");
  edit2.setStyleSheet(R"(
    background-color: grey;
    font-size: 55px;
    font-weight: 500;
    height: 120px;
  )");
  btn.setFixedSize(150, 100);
  edit1.setFixedSize(600, 100);
  edit2.setFixedSize(600, 100);
  edit1.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  edit2.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);

  hlayout->addWidget(&edit1);
  hlayout->addWidget(&edit2);
  hlayout->addWidget(&btn);

  QObject::connect(&btn, &QPushButton::clicked, [=]() {
    int list_count1 = 0;
    int list_count2 = 0;
    bool is_metric = params.getBool("IsMetric");
    if (is_metric) {
      QString targetvalue1 = InputDialog::getText(tr("Set CV values with comma"), this, tr("Values are kph"), false, 1, QString::fromStdString(params.get("VCurvSpeedC")));
      if (targetvalue1.length() > 0 && targetvalue1 != QString::fromStdString(params.get("VCurvSpeedC"))) {
        QStringList list1 = targetvalue1.split(",");
        list_count1 = list1.size();
        params.put("VCurvSpeedC", targetvalue1.toStdString());
        refresh();
      } else {
        QStringList list1 = QString::fromStdString(params.get("VCurvSpeedC")).split(",");
        list_count1 = list1.size();
      }
      QString targetvalue2 = InputDialog::getText(tr("Set TS values with comma"), this, "CV: " + QString::fromStdString(params.get("VCurvSpeedC")), false, 1, QString::fromStdString(params.get("VCurvSpeedT")));
      if (targetvalue2.length() > 0 && targetvalue2 != QString::fromStdString(params.get("VCurvSpeedT"))) {
        QStringList list2 = targetvalue2.split(",");
        list_count2 = list2.size();
        params.put("VCurvSpeedT", targetvalue2.toStdString());
        refresh();
      } else {
        QStringList list2 = QString::fromStdString(params.get("VCurvSpeedT")).split(",");
        list_count2 = list2.size();
      }
      if (list_count1 != list_count2) {
        ConfirmationDialog::alert(tr("Index count does not match. Check your input again."), this);
      }
    } else {
      QString targetvalue1 = InputDialog::getText(tr("Set CV values with comma"), this, tr("Values are mph"), false, 1, QString::fromStdString(params.get("VCurvSpeedCMPH")));
      if (targetvalue1.length() > 0 && targetvalue1 != QString::fromStdString(params.get("VCurvSpeedCMPH"))) {
        QStringList list1 = targetvalue1.split(",");
        list_count1 = list1.size();
        params.put("VCurvSpeedCMPH", targetvalue1.toStdString());
        refresh();
      } else {
        QStringList list1 = QString::fromStdString(params.get("VCurvSpeedCMPH")).split(",");
        list_count1 = list1.size();
      }
      QString targetvalue2 = InputDialog::getText(tr("Set TS values with comma"), this, "CV: " + QString::fromStdString(params.get("VCurvSpeedCMPH")), false, 1, QString::fromStdString(params.get("VCurvSpeedTMPH")));
      if (targetvalue2.length() > 0 && targetvalue2 != QString::fromStdString(params.get("VCurvSpeedTMPH"))) {
        QStringList list2 = targetvalue2.split(",");
        list_count2 = list2.size();
        params.put("VCurvSpeedTMPH", targetvalue2.toStdString());
        refresh();
      } else {
        QStringList list2 = QString::fromStdString(params.get("VCurvSpeedTMPH")).split(",");
        list_count2 = list2.size();
      }
      if (list_count1 != list_count2) {
        ConfirmationDialog::alert(tr("Index count does not match. Check your input again."), this);
      }
    }
  });
  refresh();
}

void VCurvSpeed::refresh() {
  bool is_metric = params.getBool("IsMetric");
  auto strs1 = QString::fromStdString(params.get("VCurvSpeedC"));
  auto strs2 = QString::fromStdString(params.get("VCurvSpeedT"));
  if (!is_metric) {
    strs1 = QString::fromStdString(params.get("VCurvSpeedCMPH"));
    strs2 = QString::fromStdString(params.get("VCurvSpeedTMPH"));
  }
  edit1.setText(QString::fromStdString(strs1.toStdString()));
  edit2.setText(QString::fromStdString(strs2.toStdString()));
  btn.setText(tr("EDIT"));
}

OCurvSpeedUD::OCurvSpeedUD() : AbstractControl(tr("OSMCurvDecel([TSL] [TargetSpeed])"), tr("Adjust the curve deceleration speed according to turn speed limit of OSM. (interpolation value)"), "../assets/offroad/icon_shell.png") {
}

OCurvSpeed::OCurvSpeed() : AbstractControl("", "", "") {
  btn.setStyleSheet(R"(
    padding: -10;
    border-radius: 35px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  edit1.setStyleSheet(R"(
    background-color: grey;
    font-size: 55px;
    font-weight: 500;
    height: 120px;
  )");
  edit2.setStyleSheet(R"(
    background-color: grey;
    font-size: 55px;
    font-weight: 500;
    height: 120px;
  )");
  btn.setFixedSize(150, 100);
  edit1.setFixedSize(600, 100);
  edit2.setFixedSize(600, 100);
  edit1.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  edit2.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);

  hlayout->addWidget(&edit1);
  hlayout->addWidget(&edit2);
  hlayout->addWidget(&btn);

  QObject::connect(&btn, &QPushButton::clicked, [=]() {
    int list_count1 = 0;
    int list_count2 = 0;
    QString targetvalue1 = InputDialog::getText(tr("Set TSL values with comma"), this, tr("Valus are TSL"), false, 1, QString::fromStdString(params.get("OCurvSpeedC")));
    if (targetvalue1.length() > 0 && targetvalue1 != QString::fromStdString(params.get("OCurvSpeedC"))) {
      QStringList list1 = targetvalue1.split(",");
      list_count1 = list1.size();
      params.put("OCurvSpeedC", targetvalue1.toStdString());
      refresh();
    } else {
      QStringList list1 = QString::fromStdString(params.get("OCurvSpeedC")).split(",");
      list_count1 = list1.size();
    }
    QString targetvalue2 = InputDialog::getText(tr("Set TS values with comma"), this, "TSL: " + QString::fromStdString(params.get("OCurvSpeedC")), false, 1, QString::fromStdString(params.get("OCurvSpeedT")));
    if (targetvalue2.length() > 0 && targetvalue2 != QString::fromStdString(params.get("OCurvSpeedT"))) {
      QStringList list2 = targetvalue2.split(",");
      list_count2 = list2.size();
      params.put("OCurvSpeedT", targetvalue2.toStdString());
      refresh();
    } else {
      QStringList list2 = QString::fromStdString(params.get("OCurvSpeedT")).split(",");
      list_count2 = list2.size();
    }
    if (list_count1 != list_count2) {
      ConfirmationDialog::alert(tr("Index count does not match. Check your input again."), this);
    }
  });
  refresh();
}

void OCurvSpeed::refresh() {
  auto strs1 = QString::fromStdString(params.get("OCurvSpeedC"));
  auto strs2 = QString::fromStdString(params.get("OCurvSpeedT"));
  edit1.setText(QString::fromStdString(strs1.toStdString()));
  edit2.setText(QString::fromStdString(strs2.toStdString()));
  btn.setText(tr("EDIT"));
}

KISANaviSelect::KISANaviSelect() : AbstractControl(tr("Navigation Select"), tr("Select the navigation you want to use.(None/TMap/Mappy/Waze) Refer to Readme.txt in the directory."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("◀");
  btnplus.setText("▶");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KISANaviSelect"));
    int value = str.toInt();
    value = value - 1;
    if (value <= -1) {
      value = 2;
    }
    QString values = QString::number(value);
    params.put("KISANaviSelect", values.toStdString());
    refresh();
  });
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KISANaviSelect"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 3) {
      value = 0;
    }
    QString values = QString::number(value);
    params.put("KISANaviSelect", values.toStdString());
    refresh();
  });
  refresh();
}

void KISANaviSelect::refresh() {
  QString option = QString::fromStdString(params.get("KISANaviSelect"));
  if (option == "0") {label.setText(tr("None"));
  } else if (option == "1") {label.setText(tr("TMap/Mappy"));
  } else if (option == "2") {label.setText(tr("Waze"));
  }
}

KISAMapboxStyle::KISAMapboxStyle() : AbstractControl(tr("Mapbox Style"), tr("Set the Mapbox sytle to Comma/KISA/User/Satellite"), "../assets/offroad/icon_shell.png") {
  btn1.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btn1.setFixedSize(200, 100);
  btn2.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btn2.setFixedSize(200, 100);
  btn3.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btn3.setFixedSize(200, 100);
  btn4.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btn4.setFixedSize(200, 100);
  hlayout->addWidget(&btn1);
  hlayout->addWidget(&btn2);
  hlayout->addWidget(&btn3);
  hlayout->addWidget(&btn4);
  btn1.setText(tr("Comma"));
  btn2.setText(tr("KISA"));
  btn3.setText(tr("User"));
  btn4.setText(tr("SATL"));

  QObject::connect(&btn1, &QPushButton::clicked, [=]() {
    params.put("KISAMapboxStyleSelect", "0");
    refresh();
  });
  QObject::connect(&btn2, &QPushButton::clicked, [=]() {
    params.put("KISAMapboxStyleSelect", "1");
    refresh();
  });
  QObject::connect(&btn3, &QPushButton::clicked, [=]() {
    params.put("KISAMapboxStyleSelect", "2");
    if (ConfirmationDialog::alert(tr("You've chosen own style. Please set your mapbox style to the param <MapboxStyleCustom>. File location: /data/params/d/MapboxStyleCustom"), this)) {}
    refresh();
  });
  QObject::connect(&btn4, &QPushButton::clicked, [=]() {
    params.put("KISAMapboxStyleSelect", "3");
    refresh();
  });
  refresh();
}

void KISAMapboxStyle::refresh() {
  QString option = QString::fromStdString(params.get("KISAMapboxStyleSelect"));
  if (option == "0") {
    btn1.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #00A12E;
    )");
    btn2.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
    )");
    btn3.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
    )");
    btn4.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
    )");
  } else if (option == "1") {
    btn1.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
    )");
    btn2.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #00A12E;
    )");
    btn3.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
    )");
    btn4.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
    )");
  } else if (option == "2") {
    btn1.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
    )");
    btn2.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
    )");
    btn3.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #00A12E;
    )");
    btn4.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
    )");
  } else {
    btn1.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
    )");
    btn2.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
    )");
    btn3.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
    )");
    btn4.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #00A12E;
    )");
  }
}

RESCountatStandstill::RESCountatStandstill() : AbstractControl(tr("RES Count at Standstill"), tr("Comma Default: 25, this value cannot be acceptable at some cars. So adjust the number if you want to. It generates RES CAN messages when leadcar is moving. If departure is failed, increase the number. In opposite, if CAN error occurs, decrease the number."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("RESCountatStandstill"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 1) {
      value = 1;
    }
    QString values = QString::number(value);
    params.put("RESCountatStandstill", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("RESCountatStandstill"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 50) {
      value = 50;
    }
    QString values = QString::number(value);
    params.put("RESCountatStandstill", values.toStdString());
    refresh();
  });
  refresh();
}

void RESCountatStandstill::refresh() {
  label.setText(QString::fromStdString(params.get("RESCountatStandstill")));
  btnminus.setText("-");
  btnplus.setText("+");
}

SpeedLimitSignType::SpeedLimitSignType() : AbstractControl(tr("SafetyCam SignType"), tr("Select SafetyCam SignType (Circle/Rectangle)"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("◀");
  btnplus.setText("▶");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaSpeedLimitSignType"));
    int value = str.toInt();
    value = value - 1;
    if (value <= -1) {
      value = 1;
    }
    QString values = QString::number(value);
    params.put("KisaSpeedLimitSignType", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaSpeedLimitSignType"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 2) {
      value = 0;
    }
    QString values = QString::number(value);
    params.put("KisaSpeedLimitSignType", values.toStdString());
    refresh();
  });
  refresh();
}

void SpeedLimitSignType::refresh() {
  QString option = QString::fromStdString(params.get("KisaSpeedLimitSignType"));
  if (option == "0") {
    label.setText(tr("Circle"));
  } else {
    label.setText(tr("Rectangle"));
  }
}

RadarLongHelperOption::RadarLongHelperOption() : AbstractControl(tr("Long Mode"), tr("Vision Only, Radar Only, KISA(Radar+Vision)"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("◀");
  btnplus.setText("▶");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("RadarLongHelper"));
    int value = str.toInt();
    value = value - 1;
    if (value <= -1) {
      value = 2;
    }
    QString values = QString::number(value);
    params.put("RadarLongHelper", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("RadarLongHelper"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 3) {
      value = 0;
    }
    QString values = QString::number(value);
    params.put("RadarLongHelper", values.toStdString());
    refresh();
  });
  refresh();
}

void RadarLongHelperOption::refresh() {
  QString option = QString::fromStdString(params.get("RadarLongHelper"));
  if (option == "0") {
    label.setText(tr("Vision Only"));
  } else if (option == "1") {
    label.setText(tr("Radar Only"));
  } else {
    label.setText(tr("KISA(Radar+Vision)"));
  }
}



CurvDecelSelect::CurvDecelSelect() : AbstractControl(tr("Curv Decel Option"), tr("None, Vision+OSM, Vision Only, OSM Only"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("◀");
  btnplus.setText("▶");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("CurvDecelOption"));
    int value = str.toInt();
    value = value - 1;
    if (value <= -1) {
      value = 3;
    }
    QString values = QString::number(value);
    params.put("CurvDecelOption", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("CurvDecelOption"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 4) {
      value = 0;
    }
    QString values = QString::number(value);
    params.put("CurvDecelOption", values.toStdString());
    refresh();
  });
  refresh();
}

void CurvDecelSelect::refresh() {
  QString option = QString::fromStdString(params.get("CurvDecelOption"));
  if (option == "0") {
    label.setText(tr("None"));
  } else if (option == "1") {
    label.setText(tr("Vision+OSM"));
  } else if (option == "2") {
    label.setText(tr("Vision Only"));
  } else {
    label.setText(tr("OSM Only"));
  }
}

AutoRESDelay::AutoRESDelay() : AbstractControl(tr("AutoRES Delay(sec)"), tr("Give delay time to trigger for AutoRES while driving."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("AutoRESDelay"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 0) {
      value = 0;
    }
    QString values = QString::number(value);
    params.put("AutoRESDelay", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("AutoRESDelay"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 20) {
      value = 20;
    }
    QString values = QString::number(value);
    params.put("AutoRESDelay", values.toStdString());
    refresh();
  });
  refresh();
}

void AutoRESDelay::refresh() {
  QString option = QString::fromStdString(params.get("AutoRESDelay"));
  if (option == "0") {
    label.setText(tr("No Delay"));
  } else {
    label.setText(QString::fromStdString(params.get("AutoRESDelay")));
  }
  btnminus.setText("-");
  btnplus.setText("+");
}

OSMCustomSpeedLimitUD::OSMCustomSpeedLimitUD() : AbstractControl(tr("CustomSpeedLimit([SL] [TargetSpeed])"), tr("Set the offset speed according to speed limit of OSM or Waze. (interpolation value)"), "../assets/offroad/icon_shell.png") {
}

OSMCustomSpeedLimit::OSMCustomSpeedLimit() : AbstractControl("", "", "") {
  btn.setStyleSheet(R"(
    padding: -10;
    border-radius: 35px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  edit1.setStyleSheet(R"(
    background-color: grey;
    font-size: 55px;
    font-weight: 500;
    height: 120px;
  )");
  edit2.setStyleSheet(R"(
    background-color: grey;
    font-size: 55px;
    font-weight: 500;
    height: 120px;
  )");
  btn.setFixedSize(150, 100);
  edit1.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  edit2.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  edit1.setFixedSize(600, 100);
  edit2.setFixedSize(600, 100);

  hlayout->addWidget(&edit1);
  hlayout->addWidget(&edit2);
  hlayout->addWidget(&btn);

  QObject::connect(&btn, &QPushButton::clicked, [=]() {
    int list_count1 = 0;
    int list_count2 = 0;
    QString targetvalue1 = InputDialog::getText(tr("Set SL values with comma"), this, tr("Values are kph or mph"), false, 1, QString::fromStdString(params.get("OSMCustomSpeedLimitC")));
    if (targetvalue1.length() > 0 && targetvalue1 != QString::fromStdString(params.get("OSMCustomSpeedLimitC"))) {
      QStringList list1 = targetvalue1.split(",");
      list_count1 = list1.size();
      params.put("OSMCustomSpeedLimitC", targetvalue1.toStdString());
      refresh();
    } else {
      QStringList list1 = QString::fromStdString(params.get("OSMCustomSpeedLimitC")).split(",");
      list_count1 = list1.size();
    }
    QString targetvalue2 = InputDialog::getText(tr("Set CTSL values with comma"), this, "SL: " + QString::fromStdString(params.get("OSMCustomSpeedLimitC")), false, 1, QString::fromStdString(params.get("OSMCustomSpeedLimitT")));
    if (targetvalue2.length() > 0 && targetvalue2 != QString::fromStdString(params.get("OSMCustomSpeedLimitT"))) {
      QStringList list2 = targetvalue2.split(",");
      list_count2 = list2.size();
      params.put("OSMCustomSpeedLimitT", targetvalue2.toStdString());
      refresh();
    } else {
      QStringList list2 = QString::fromStdString(params.get("OSMCustomSpeedLimitT")).split(",");
      list_count2 = list2.size();
    }
    if (list_count1 != list_count2) {
      ConfirmationDialog::alert(tr("Index count does not match. Check your input again."), this);
    }
  });
  refresh();
}

void OSMCustomSpeedLimit::refresh() {
  auto strs1 = QString::fromStdString(params.get("OSMCustomSpeedLimitC"));
  auto strs2 = QString::fromStdString(params.get("OSMCustomSpeedLimitT"));
  edit1.setText(QString::fromStdString(strs1.toStdString()));
  edit2.setText(QString::fromStdString(strs2.toStdString()));
  btn.setText(tr("EDIT"));
}

DesiredCurvatureLimit::DesiredCurvatureLimit() : AbstractControl(tr("DesiredCurvatureLimit"), tr("Adjust DisiredCurvatureLimit, Default is 0.05(DT_MDL), For HKG, maybe 0.2 is preferred from user's experience. If the steering digs into inside on intersection, upper the value. And then it will limit your scope of steering angle. In case of opposite situation, lower the value. this is multiplier of desired curvature rate not real limit value."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btndigit.setStyleSheet(R"(
    QPushButton {
      padding: 0;
      border-radius: 50px;
      font-size: 35px;
      font-weight: 500;
      color: #E4E4E4;
      background-color: #393939;
    }
    QPushButton:pressed {
      background-color: #ababab;
    }
  )");
  btnminus.setStyleSheet(R"(
    QPushButton {
      padding: 0;
      border-radius: 50px;
      font-size: 35px;
      font-weight: 500;
      color: #E4E4E4;
      background-color: #393939;
    }
    QPushButton:pressed {
      background-color: #ababab;
    }
  )");
  btnplus.setStyleSheet(R"(
    QPushButton {
      padding: 0;
      border-radius: 50px;
      font-size: 35px;
      font-weight: 500;
      color: #E4E4E4;
      background-color: #393939;
    }
    QPushButton:pressed {
      background-color: #ababab;
    }
  )");
  btndigit.setFixedSize(100, 100);
  btnminus.setFixedSize(100, 100);
  btnplus.setFixedSize(100, 100);
  hlayout->addWidget(&btndigit);
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);
  btndigit.setText("0.01");
  btnminus.setText("-");
  btnplus.setText("+");

  QObject::connect(&btndigit, &QPushButton::clicked, [=]() {
    digit = digit * 10;
    if (digit >= 2) {
      digit = 0.01;
    }
    QString level = QString::number(digit);
    btndigit.setText(level);
  });

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("DesiredCurvatureLimit"));
    int value = str.toInt();
    value = value - (digit*100);
    if (value <= 5) {
      value = 5;
    }
    QString values = QString::number(value);
    params.put("DesiredCurvatureLimit", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("DesiredCurvatureLimit"));
    int value = str.toInt();
    value = value + (digit*100);
    if (value >= 1000) {
      value = 1000;
    }
    QString values = QString::number(value);
    params.put("DesiredCurvatureLimit", values.toStdString());
    refresh();
  });
  refresh();
}

void DesiredCurvatureLimit::refresh() {
  auto strs = QString::fromStdString(params.get("DesiredCurvatureLimit"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.01;
  QString valuefs = QString::number(valuef);
  label.setText("＊ " + QString::fromStdString(valuefs.toStdString()));
}

DynamicTRUD::DynamicTRUD() : AbstractControl(tr("DynamicTR: [Speed] [TRs]"), tr("Set TR of each speeds. (Mid range is interpolation values)"), "../assets/offroad/icon_shell.png") {
}

DynamicTRBySpeed::DynamicTRBySpeed() : AbstractControl("", "", "") {
  btn.setStyleSheet(R"(
    padding: -10;
    border-radius: 35px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  edit1.setStyleSheet(R"(
    background-color: grey;
    font-size: 55px;
    font-weight: 500;
    height: 120px;
  )");
  edit2.setStyleSheet(R"(
    background-color: grey;
    font-size: 55px;
    font-weight: 500;
    height: 120px;
  )");
  btn.setFixedSize(150, 100);
  edit1.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  edit2.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);

  hlayout->addWidget(&edit1);
  hlayout->addWidget(&edit2);
  hlayout->addWidget(&btn);

  QObject::connect(&btn, &QPushButton::clicked, [=]() {
    int list_count1 = 0;
    int list_count2 = 0;
    QString targetvalue1 = InputDialog::getText(tr("Set Speed values with comma"), this, tr("Values are kph or mph"), false, 1, QString::fromStdString(params.get("DynamicTRSpd")));
    if (targetvalue1.length() > 0 && targetvalue1 != QString::fromStdString(params.get("DynamicTRSpd"))) {
      QStringList list1 = targetvalue1.split(",");
      list_count1 = list1.size();
      params.put("DynamicTRSpd", targetvalue1.toStdString());
      refresh();
    } else {
      QStringList list1 = QString::fromStdString(params.get("DynamicTRSpd")).split(",");
      list_count1 = list1.size();
    }
    QString targetvalue2 = InputDialog::getText(tr("Set TR values with comma"), this, "SPD: " + QString::fromStdString(params.get("DynamicTRSpd")), false, 1, QString::fromStdString(params.get("DynamicTRSet")));
    if (targetvalue2.length() > 0 && targetvalue2 != QString::fromStdString(params.get("DynamicTRSet"))) {
      QStringList list2 = targetvalue2.split(",");
      list_count2 = list2.size();
      params.put("DynamicTRSet", targetvalue2.toStdString());
      refresh();
    } else {
      QStringList list2 = QString::fromStdString(params.get("DynamicTRSet")).split(",");
      list_count2 = list2.size();
    }
    if (list_count1 != list_count2) {
      ConfirmationDialog::alert(tr("Index count does not match. Check your input again."), this);
    }
  });
  refresh();
}

void DynamicTRBySpeed::refresh() {
  auto strs1 = QString::fromStdString(params.get("DynamicTRSpd"));
  auto strs2 = QString::fromStdString(params.get("DynamicTRSet"));
  edit1.setText(QString::fromStdString(strs1.toStdString()));
  edit2.setText(QString::fromStdString(strs2.toStdString()));
  btn.setText(tr("EDIT"));
}

LaneWidth::LaneWidth() : AbstractControl(tr("Set LaneWidth"), tr("Set LaneWidth (default:3.7)"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("LaneWidth"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 23) {
      value = 23;
    }
    QString values = QString::number(value);
    params.put("LaneWidth", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("LaneWidth"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 40) {
      value = 40;
    }
    QString values = QString::number(value);
    params.put("LaneWidth", values.toStdString());
    refresh();
  });
  refresh();
}

void LaneWidth::refresh() {
  auto strs = QString::fromStdString(params.get("LaneWidth"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.1;
  QString valuefs = QString::number(valuef);
  label.setText(QString::fromStdString(valuefs.toStdString()));
}

SpeedLaneWidthUD::SpeedLaneWidthUD() : AbstractControl(tr("Speed LaneWidth: [Spd(m/s)] [LaneWidth]"), tr("Set LaneWidths by speed. Speed is m/s values not kph or mph. (Mid range is interpolation values)"), "../assets/offroad/icon_shell.png") {
}

SpeedLaneWidth::SpeedLaneWidth() : AbstractControl("", "", "") {
  btn.setStyleSheet(R"(
    padding: -10;
    border-radius: 35px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  edit1.setStyleSheet(R"(
    background-color: grey;
    font-size: 55px;
    font-weight: 500;
    height: 120px;
  )");
  edit2.setStyleSheet(R"(
    background-color: grey;
    font-size: 55px;
    font-weight: 500;
    height: 120px;
  )");
  btn.setFixedSize(150, 100);
  edit1.setFixedSize(600, 100);
  edit2.setFixedSize(600, 100);
  edit1.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  edit2.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);

  hlayout->addWidget(&edit1);
  hlayout->addWidget(&edit2);
  hlayout->addWidget(&btn);

  QObject::connect(&btn, &QPushButton::clicked, [=]() {
    int list_count1 = 0;
    int list_count2 = 0;
    QString targetvalue1 = InputDialog::getText(tr("Set Speed(m/s) values with comma"), this, tr("Values are m/s unit."), false, 1, QString::fromStdString(params.get("SpdLaneWidthSpd")));
    if (targetvalue1.length() > 0 && targetvalue1 != QString::fromStdString(params.get("SpdLaneWidthSpd"))) {
      QStringList list1 = targetvalue1.split(",");
      list_count1 = list1.size();
      params.put("SpdLaneWidthSpd", targetvalue1.toStdString());
      refresh();
    } else {
      QStringList list1 = QString::fromStdString(params.get("SpdLaneWidthSpd")).split(",");
      list_count1 = list1.size();
    }
    QString targetvalue2 = InputDialog::getText(tr("Set LW(m) values with comma"), this, "SPD: " + QString::fromStdString(params.get("SpdLaneWidthSpd")), false, 1, QString::fromStdString(params.get("SpdLaneWidthSet")));
    if (targetvalue2.length() > 0 && targetvalue2 != QString::fromStdString(params.get("SpdLaneWidthSet"))) {
      QStringList list2 = targetvalue2.split(",");
      list_count2 = list2.size();
      params.put("SpdLaneWidthSet", targetvalue2.toStdString());
      refresh();
    } else {
      QStringList list2 = QString::fromStdString(params.get("SpdLaneWidthSet")).split(",");
      list_count2 = list2.size();
    }
    if (list_count1 != list_count2) {
      ConfirmationDialog::alert(tr("Index count does not match. Check your input again."), this);
    }
  });
  refresh();
}

void SpeedLaneWidth::refresh() {
  auto strs1 = QString::fromStdString(params.get("SpdLaneWidthSpd"));
  auto strs2 = QString::fromStdString(params.get("SpdLaneWidthSet"));
  edit1.setText(QString::fromStdString(strs1.toStdString()));
  edit2.setText(QString::fromStdString(strs2.toStdString()));
  btn.setText(tr("EDIT"));
}

KISABottomTextView::KISABottomTextView() : AbstractControl(tr("Bottom Text View"), tr("Date/Time/StreetName"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("◀");
  btnplus.setText("▶");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("BottomTextView"));
    int value = str.toInt();
    value = value - 1;
    if (value <= -1) {
      value = 7;
    }
    QString values = QString::number(value);
    params.put("BottomTextView", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("BottomTextView"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 8) {
      value = 0;
    }
    QString values = QString::number(value);
    params.put("BottomTextView", values.toStdString());
    refresh();
  });
  refresh();
}

void KISABottomTextView::refresh() {
  QString option = QString::fromStdString(params.get("BottomTextView"));
  if (option == "0") {
    label.setText(tr("None"));
    uiState()->scene.bottom_text_view = 0;
  } else if (option == "1") {
    label.setText(tr("Date+Time"));
    uiState()->scene.bottom_text_view = 1;
  } else if (option == "2") {
    label.setText(tr("Date"));
    uiState()->scene.bottom_text_view = 2;
  } else if (option == "3") {
    label.setText(tr("Time"));
    uiState()->scene.bottom_text_view = 3;
  } else if (option == "4") {
    label.setText(tr("Date+Time+Str"));
    uiState()->scene.bottom_text_view = 4;
  } else if (option == "5") {
    label.setText(tr("Date+Str"));
    uiState()->scene.bottom_text_view = 5;
  } else if (option == "6") {
    label.setText(tr("Time+Str"));
    uiState()->scene.bottom_text_view = 6;
  } else {
    label.setText(tr("StreetName"));
    uiState()->scene.bottom_text_view = 7;
  }
}

KISAEdgeOffset::KISAEdgeOffset() : AbstractControl("", tr("low value to move car to left, high value to move car to right on each lane."), "") {

  labell1.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  labell1.setText(tr("LeftEdge: "));
  hlayout->addWidget(&labell1);
  labell.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  labell.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&labell);
  btnminusl.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplusl.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminusl.setFixedSize(150, 100);
  btnplusl.setFixedSize(150, 100);
  hlayout->addWidget(&btnminusl);
  hlayout->addWidget(&btnplusl);

  labelr1.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  labelr1.setText(tr("RightEdge: "));
  hlayout->addWidget(&labelr1);
  labelr.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  labelr.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&labelr);
  btnminusr.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplusr.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminusr.setFixedSize(150, 100);
  btnplusr.setFixedSize(150, 100);
  hlayout->addWidget(&btnminusr);
  hlayout->addWidget(&btnplusr);

  btnminusl.setText("－");
  btnplusl.setText("＋");
  btnminusr.setText("－");
  btnplusr.setText("＋");

  QObject::connect(&btnminusl, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("LeftEdgeOffset"));
    int value = str.toInt();
    value = value - 1;
    if (value <= -50) {
      value = -50;
    }
    QString values = QString::number(value);
    params.put("LeftEdgeOffset", values.toStdString());
    refreshl();
  });
  
  QObject::connect(&btnplusl, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("LeftEdgeOffset"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 50) {
      value = 50;
    }
    QString values = QString::number(value);
    params.put("LeftEdgeOffset", values.toStdString());
    refreshl();
  });
  QObject::connect(&btnminusr, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("RightEdgeOffset"));
    int value = str.toInt();
    value = value - 1;
    if (value <= -50) {
      value = -50;
    }
    QString values = QString::number(value);
    params.put("RightEdgeOffset", values.toStdString());
    refreshr();
  });
  
  QObject::connect(&btnplusr, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("RightEdgeOffset"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 50) {
      value = 50;
    }
    QString values = QString::number(value);
    params.put("RightEdgeOffset", values.toStdString());
    refreshr();
  });
  refreshl();
  refreshr();
}

void KISAEdgeOffset::refreshl() {
  auto strs = QString::fromStdString(params.get("LeftEdgeOffset"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.01;
  QString valuefs = QString::number(valuef);
  labell.setText(QString::fromStdString(valuefs.toStdString()));
}

void KISAEdgeOffset::refreshr() {
  auto strs = QString::fromStdString(params.get("RightEdgeOffset"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.01;
  QString valuefs = QString::number(valuef);
  labelr.setText(QString::fromStdString(valuefs.toStdString()));
}

ToAvoidLKASFault::ToAvoidLKASFault() : AbstractControl("", "", "") {

  labell1.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  labell1.setText(tr("MaxAngle: "));
  hlayout->addWidget(&labell1);
  labell.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  labell.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&labell);
  btnminusl.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplusl.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminusl.setFixedSize(150, 100);
  btnplusl.setFixedSize(150, 100);
  hlayout->addWidget(&btnminusl);
  hlayout->addWidget(&btnplusl);

  labelr1.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  labelr1.setText(tr("MaxFrame: "));
  hlayout->addWidget(&labelr1);
  labelr.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  labelr.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&labelr);
  btnminusr.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplusr.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminusr.setFixedSize(150, 100);
  btnplusr.setFixedSize(150, 100);
  hlayout->addWidget(&btnminusr);
  hlayout->addWidget(&btnplusr);

  btnminusl.setText("－");
  btnplusl.setText("＋");
  btnminusr.setText("－");
  btnplusr.setText("＋");

  QObject::connect(&btnminusl, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("AvoidLKASFaultMaxAngle"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 45) {
      value = 45;
    }
    QString values = QString::number(value);
    params.put("AvoidLKASFaultMaxAngle", values.toStdString());
    refreshl();
  });
  
  QObject::connect(&btnplusl, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("AvoidLKASFaultMaxAngle"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 99) {
      value = 99;
    }
    QString values = QString::number(value);
    params.put("AvoidLKASFaultMaxAngle", values.toStdString());
    refreshl();
  });

  QObject::connect(&btnminusr, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("AvoidLKASFaultMaxFrame"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 10) {
      value = 10;
    }
    QString values = QString::number(value);
    params.put("AvoidLKASFaultMaxFrame", values.toStdString());
    refreshr();
  });
  
  QObject::connect(&btnplusr, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("AvoidLKASFaultMaxFrame"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 150) {
      value = 150;
    }
    QString values = QString::number(value);
    params.put("AvoidLKASFaultMaxFrame", values.toStdString());
    refreshr();
  });
  refreshl();
  refreshr();
}

void ToAvoidLKASFault::refreshl() {
  labell.setText(QString::fromStdString(params.get("AvoidLKASFaultMaxAngle")));
}

void ToAvoidLKASFault::refreshr() {
  labelr.setText(QString::fromStdString(params.get("AvoidLKASFaultMaxFrame")));
}

RoutineDriveOption::RoutineDriveOption() : AbstractControl("", "", "") {

  btn0.setFixedSize(150, 100);
  btn1.setFixedSize(150, 100);
  btn0.setText("CO");
  btn1.setText("SL");
  hlayout->addWidget(&btn0);
  hlayout->addWidget(&btn1);

  QObject::connect(&btn0, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("RoutineDriveOption"));
    bool is_value = str.contains("0");
    if (is_value) {
      QString values = str.replace("0", "");
      params.put("RoutineDriveOption", values.toStdString());
    } else {
      QString values = str + "0";
      params.put("RoutineDriveOption", values.toStdString());
    }
    refresh();
  });
  QObject::connect(&btn1, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("RoutineDriveOption"));
    bool is_value = str.contains("1");
    if (is_value) {
      QString values = str.replace("1", "");
      params.put("RoutineDriveOption", values.toStdString());
    } else {
      QString values = str + "1";
      params.put("RoutineDriveOption", values.toStdString());
    }
    refresh();
  });
  refresh();
}

void RoutineDriveOption::refresh() {
  QString option = QString::fromStdString(params.get("RoutineDriveOption"));
  if (option.contains("0")) {
    btn0.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #00A12E;
    )");
  } else {
    btn0.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
    )");
  }
  if (option.contains("1")) {
    btn1.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #00A12E;
    )");
  } else {
    btn1.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
    )");
  }
}

RPMAnimatedMaxValue::RPMAnimatedMaxValue() : AbstractControl(tr("AnimatedRPM Max"), tr("Set Max RPM for animated rpm value."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("AnimatedRPMMax"));
    int value = str.toInt();
    value = value - 100;
    if (value <= 500) {
      value = 500;
    }
    QString values = QString::number(value);
    params.put("AnimatedRPMMax", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("AnimatedRPMMax"));
    int value = str.toInt();
    value = value + 100;
    if (value >= 8000) {
      value = 8000;
    }
    QString values = QString::number(value);
    params.put("AnimatedRPMMax", values.toStdString());
    refresh();
  });
  refresh();
}

void RPMAnimatedMaxValue::refresh() {
  label.setText(QString::fromStdString(params.get("AnimatedRPMMax")));
}

UserSpecificFeature::UserSpecificFeature() : AbstractControl(tr("FeatureNumber"), tr("User Specific Feature"), "") {
  btn.setStyleSheet(R"(
    padding: -10;
    border-radius: 35px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  edit.setStyleSheet(R"(
    background-color: grey;
    font-size: 55px;
    font-weight: 500;
    height: 120px;
  )");
  edit.setFixedSize(900, 100);
  btn.setFixedSize(200, 100);
  hlayout->addWidget(&edit);
  hlayout->addWidget(&btn);

  QObject::connect(&btn, &QPushButton::clicked, [=]() {
    QString targetvalue = InputDialog::getText(tr("User Specific Features"), this, tr("Put your number you know."), false, 1, QString::fromStdString(params.get("UserSpecificFeature")));
    if (targetvalue.length() > 0 && targetvalue != QString::fromStdString(params.get("UserSpecificFeature"))) {
      params.put("UserSpecificFeature", targetvalue.toStdString());
      refresh();
    }
   });
  refresh();
}

void UserSpecificFeature::refresh() {
  auto strs = QString::fromStdString(params.get("UserSpecificFeature"));
  edit.setText(QString::fromStdString(strs.toStdString()));
  btn.setText(tr("SET"));
}

MultipleLatSelect::MultipleLatSelect() : AbstractControl(tr("Multi LateralControl"), tr("Multiple Lateral Tune by Speed/Angle."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("◀");
  btnplus.setText("▶");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  auto str = QString::fromStdString(params.get("MultipleLateralUse"));
  m_nMethod = str.toInt();

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    m_nMethod -= 1;
    if (m_nMethod < 0) {
      m_nMethod = 3;
    }

    QString values = QString::number(m_nMethod);
    params.put("MultipleLateralUse", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
  
    m_nMethod += 1;
    if (m_nMethod > 3) {
      m_nMethod = 0;
    }
    QString values = QString::number(m_nMethod);
    params.put("MultipleLateralUse", values.toStdString());
    refresh();
  });
  refresh();
}

void MultipleLatSelect::refresh() {
  QString strMethod;

  switch( m_nMethod )
  {
    case 0 : strMethod = "Spd_Split"; break;
    case 1 : strMethod = "Ang_Split"; break;
    case 2 : strMethod = "Ang_Intrp"; break;
    case 3 : strMethod = "Spd_Intrp"; break;
    default :
      strMethod = "None"; 
      break;
  }


  label.setText( strMethod );
}

MultipleLateralSpeed::MultipleLateralSpeed() : AbstractControl("", "", "") {
  label1.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  label1.setText(tr("SPD: "));
  hlayout->addWidget(&label1);
  labell.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  labell.setStyleSheet("color: #e0e879");
  labelr.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  labelr.setStyleSheet("color: #e0e879");
  btn1.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 50px;
    font-weight: 500;
    color: #e0e879;
    background-color: #808080;
  )");
  btn2.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 50px;
    font-weight: 500;
    color: #e0e879;
    background-color: #808080;
  )");
  btn3.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 50px;
    font-weight: 500;
    color: #e0e879;
    background-color: #808080;
  )");
  btnminusl.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplusl.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminusr.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplusr.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminusl.setFixedSize(70, 100);
  btnplusl.setFixedSize(70, 100);
  btnminusr.setFixedSize(70, 100);
  btnplusr.setFixedSize(70, 100);
  btn1.setFixedSize(150, 100);
  btn2.setFixedSize(150, 100);
  btn3.setFixedSize(150, 100);

  hlayout->addWidget(&btn1);
  hlayout->addWidget(&btnminusl);
  hlayout->addWidget(&labell);
  hlayout->addWidget(&btnplusl);
  hlayout->addWidget(&btn2);
  hlayout->addWidget(&btnminusr);
  hlayout->addWidget(&labelr);
  hlayout->addWidget(&btnplusr);
  hlayout->addWidget(&btn3);

  btnminusl.setText("－");
  btnplusl.setText("＋");
  btnminusr.setText("－");
  btnplusr.setText("＋");

  QObject::connect(&btn1, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("MultipleLateralOpS")).split(",");
    int value = list[0].toInt();
    value = value + 1;
    if (value >= 4) {
      value = 0;
    }
    QString values = QString::number(value);
    list[0] = values;
    QString str = list.join(",");
    params.put("MultipleLateralOpS", str.toStdString());
    refresh1();
  });

  QObject::connect(&btn2, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("MultipleLateralOpS")).split(",");
    int value = list[1].toInt();
    value = value + 1;
    if (value >= 4) {
      value = 0;
    }
    QString values = QString::number(value);
    list[1] = values;
    QString str = list.join(",");
    params.put("MultipleLateralOpS", str.toStdString());
    refresh2();
  });

  QObject::connect(&btn3, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("MultipleLateralOpS")).split(",");
    int value = list[2].toInt();
    value = value + 1;
    if (value >= 4) {
      value = 0;
    }
    QString values = QString::number(value);
    list[2] = values;
    QString str = list.join(",");
    params.put("MultipleLateralOpS", str.toStdString());
    refresh3();
  });

  QObject::connect(&btnminusl, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("MultipleLateralSpd")).split(",");
    int value = list[0].toInt();
    value = value - 5;
    if (value <= 5) {
      value = 5;
    }
    QString values = QString::number(value);
    list[0] = values;
    QString str = list.join(",");
    params.put("MultipleLateralSpd", str.toStdString());
    refreshl();
  });

  QObject::connect(&btnplusl, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("MultipleLateralSpd")).split(",");
    int value = list[0].toInt();
    int valuem = list[1].toInt();
    value = value + 5;
    if (value >= (valuem - 5)) {
      value = valuem - 5;
    }
    QString values = QString::number(value);
    list[0] = values;
    QString str = list.join(",");
    params.put("MultipleLateralSpd", str.toStdString());
    refreshl();
  });

  QObject::connect(&btnminusr, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("MultipleLateralSpd")).split(",");
    int value = list[1].toInt();
    int valuem = list[0].toInt();
    value = value - 5;
    if (value <= (valuem + 5)) {
      value = valuem + 5;
    }
    QString values = QString::number(value);
    list[1] = values;
    QString str = list.join(",");
    params.put("MultipleLateralSpd", str.toStdString());
    refreshr();
  });

  QObject::connect(&btnplusr, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("MultipleLateralSpd")).split(",");
    int value = list[1].toInt();
    value = value + 5;
    if (value >= 110) {
      value = 110;
    }
    QString values = QString::number(value);
    list[1] = values;
    QString str = list.join(",");
    params.put("MultipleLateralSpd", str.toStdString());
    refreshr();
  });
  refresh1();
  refresh2();
  refresh3();
  refreshl();
  refreshr();
}

void MultipleLateralSpeed::refresh1() {
  QStringList list = QString::fromStdString(params.get("MultipleLateralOpS")).split(",");
  if (list[0] == "0") {
    btn1.setText("PID");
  } else if (list[0] == "1") {
    btn1.setText("IND");
  } else if (list[0] == "2") {
    btn1.setText("LQR");
  } else if (list[0] == "3") {
    btn1.setText("TOQ");
  }
}

void MultipleLateralSpeed::refresh2() {
  QStringList list = QString::fromStdString(params.get("MultipleLateralOpS")).split(",");
  if (list[1] == "0") {
    btn2.setText("PID");
  } else if (list[1] == "1") {
    btn2.setText("IND");
  } else if (list[1] == "2") {
    btn2.setText("LQR");
  } else if (list[1] == "3") {
    btn2.setText("TOQ");
  }
}

void MultipleLateralSpeed::refresh3() {
  QStringList list = QString::fromStdString(params.get("MultipleLateralOpS")).split(",");
  if (list[2] == "0") {
    btn3.setText("PID");
  } else if (list[2] == "1") {
    btn3.setText("IND");
  } else if (list[2] == "2") {
    btn3.setText("LQR");
  } else if (list[2] == "3") {
    btn3.setText("TOQ");
  }
}

void MultipleLateralSpeed::refreshl() {
  QStringList list = QString::fromStdString(params.get("MultipleLateralSpd")).split(",");
  labell.setText(list[0]);
}

void MultipleLateralSpeed::refreshr() {
  QStringList list = QString::fromStdString(params.get("MultipleLateralSpd")).split(",");
  labelr.setText(list[1]);
}

MultipleLateralAngle::MultipleLateralAngle() : AbstractControl("", "", "") {
  label1.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  label1.setText(tr("ANG: "));
  hlayout->addWidget(&label1);
  labell.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  labell.setStyleSheet("color: #e0e879");
  labelr.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  labelr.setStyleSheet("color: #e0e879");
  btn1.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 50px;
    font-weight: 500;
    color: #e0e879;
    background-color: #808080;
  )");
  btn2.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 50px;
    font-weight: 500;
    color: #e0e879;
    background-color: #808080;
  )");
  btn3.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 50px;
    font-weight: 500;
    color: #e0e879;
    background-color: #808080;
  )");
  btnminusl.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplusl.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminusr.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplusr.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminusl.setFixedSize(70, 100);
  btnplusl.setFixedSize(70, 100);
  btnminusr.setFixedSize(70, 100);
  btnplusr.setFixedSize(70, 100);
  btn1.setFixedSize(150, 100);
  btn2.setFixedSize(150, 100);
  btn3.setFixedSize(150, 100);

  hlayout->addWidget(&btn1);
  hlayout->addWidget(&btnminusl);
  hlayout->addWidget(&labell);
  hlayout->addWidget(&btnplusl);
  hlayout->addWidget(&btn2);
  hlayout->addWidget(&btnminusr);
  hlayout->addWidget(&labelr);
  hlayout->addWidget(&btnplusr);
  hlayout->addWidget(&btn3);

  btnminusl.setText("－");
  btnplusl.setText("＋");
  btnminusr.setText("－");
  btnplusr.setText("＋");

  QObject::connect(&btn1, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("MultipleLateralOpA")).split(",");
    int value = list[0].toInt();
    value = value + 1;
    if (value >= 4) {
      value = 0;
    }
    QString values = QString::number(value);
    list[0] = values;
    QString str = list.join(",");
    params.put("MultipleLateralOpA", str.toStdString());
    refresh1();
  });

  QObject::connect(&btn2, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("MultipleLateralOpA")).split(",");
    int value = list[1].toInt();
    value = value + 1;
    if (value >= 4) {
      value = 0;
    }
    QString values = QString::number(value);
    list[1] = values;
    QString str = list.join(",");
    params.put("MultipleLateralOpA", str.toStdString());
    refresh2();
  });

  QObject::connect(&btn3, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("MultipleLateralOpA")).split(",");
    int value = list[2].toInt();
    value = value + 1;
    if (value >= 4) {
      value = 0;
    }
    QString values = QString::number(value);
    list[2] = values;
    QString str = list.join(",");
    params.put("MultipleLateralOpA", str.toStdString());
    refresh3();
  });

  QObject::connect(&btnminusl, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("MultipleLateralAng")).split(",");
    int value = list[0].toInt();
    value = value - 5;
    if (value <= 5) {
      value = 5;
    }
    QString values = QString::number(value);
    list[0] = values;
    QString str = list.join(",");
    params.put("MultipleLateralAng", str.toStdString());
    refreshl();
  });

  QObject::connect(&btnplusl, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("MultipleLateralAng")).split(",");
    int value = list[0].toInt();
    int valuem = list[1].toInt();
    value = value + 5;
    if (value >= (valuem - 5)) {
      value = valuem - 5;
    }
    QString values = QString::number(value);
    list[0] = values;
    QString str = list.join(",");
    params.put("MultipleLateralAng", str.toStdString());
    refreshl();
  });

  QObject::connect(&btnminusr, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("MultipleLateralAng")).split(",");
    int value = list[1].toInt();
    int valuem = list[0].toInt();
    value = value - 5;
    if (value <= (valuem + 5)) {
      value = valuem + 5;
    }
    QString values = QString::number(value);
    list[1] = values;
    QString str = list.join(",");
    params.put("MultipleLateralAng", str.toStdString());
    refreshr();
  });

  QObject::connect(&btnplusr, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("MultipleLateralAng")).split(",");
    int value = list[1].toInt();
    value = value + 5;
    if (value >= 90) {
      value = 90;
    }
    QString values = QString::number(value);
    list[1] = values;
    QString str = list.join(",");
    params.put("MultipleLateralAng", str.toStdString());
    refreshr();
  });
  refresh1();
  refresh2();
  refresh3();
  refreshl();
  refreshr();
}

void MultipleLateralAngle::refresh1() {
  QStringList list = QString::fromStdString(params.get("MultipleLateralOpA")).split(",");
  if (list[0] == "0") {
    btn1.setText("PID");
  } else if (list[0] == "1") {
    btn1.setText("IND");
  } else if (list[0] == "2") {
    btn1.setText("LQR");
  } else if (list[0] == "3") {
    btn1.setText("TOQ");
  }
}

void MultipleLateralAngle::refresh2() {
  QStringList list = QString::fromStdString(params.get("MultipleLateralOpA")).split(",");
  if (list[1] == "0") {
    btn2.setText("PID");
  } else if (list[1] == "1") {
    btn2.setText("IND");
  } else if (list[1] == "2") {
    btn2.setText("LQR");
  } else if (list[1] == "3") {
    btn2.setText("TOQ");
  }
}

void MultipleLateralAngle::refresh3() {
  QStringList list = QString::fromStdString(params.get("MultipleLateralOpA")).split(",");
  if (list[2] == "0") {
    btn3.setText("PID");
  } else if (list[2] == "1") {
    btn3.setText("IND");
  } else if (list[2] == "2") {
    btn3.setText("LQR");
  } else if (list[2] == "3") {
    btn3.setText("TOQ");
  }
}

void MultipleLateralAngle::refreshl() {
  QStringList list = QString::fromStdString(params.get("MultipleLateralAng")).split(",");
  labell.setText(list[0]);
}

void MultipleLateralAngle::refreshr() {
  QStringList list = QString::fromStdString(params.get("MultipleLateralAng")).split(",");
  labelr.setText(list[1]);
}

StoppingDist::StoppingDist() : AbstractControl(tr("Stopping Distance(m)"), tr("Car starts to stop under the value."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("StoppingDist"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 30) {
      value = 30;
    }
    QString values = QString::number(value);
    params.put("StoppingDist", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("StoppingDist"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 60) {
      value = 60;
    }
    QString values = QString::number(value);
    params.put("StoppingDist", values.toStdString());
    refresh();
  });
  refresh();
}

void StoppingDist::refresh() {
  auto strs = QString::fromStdString(params.get("StoppingDist"));
  int valuei = strs.toInt();
  float valuef = valuei * 0.1;
  QString valuefs = QString::number(valuef);
  label.setText(QString::fromStdString(valuefs.toStdString()));
}

VariableCruiseLevel::VariableCruiseLevel() : AbstractControl(tr("Button Spamming Level"), tr("High values make early stopping and starting, but might be not comfortable. Low values are the opposite."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("VarCruiseSpeedFactor"));
    int value = str.toInt();
    value = value - 1;
    if (value <= -1) {
      value = 16;
    }
    QString values = QString::number(value);
    params.put("VarCruiseSpeedFactor", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("VarCruiseSpeedFactor"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 17) {
      value = 0;
    }
    QString values = QString::number(value);
    params.put("VarCruiseSpeedFactor", values.toStdString());
    refresh();
  });
  refresh();
}

void VariableCruiseLevel::refresh() {
  label.setText(QString::fromStdString(params.get("VarCruiseSpeedFactor")));
}

ExternalDeviceIP::ExternalDeviceIP() : AbstractControl(tr("ExternalDevIP"), tr("Set Your External Device IP to get useful data. ex. a ip:192.168.0.1 / two or more: 192.168.0.1,192.168.0.2 put comma btw IPs / range:192.168.0.1-10  192.168.0-10.254 use dash(-)"), "") {
  btna.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btn.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  edit.setStyleSheet(R"(
    background-color: grey;
    font-size: 55px;
    font-weight: 500;
    height: 120px;
  )");

  edit.setFixedSize(720, 100);
  btn.setFixedSize(150, 100);
  btna.setFixedSize(250, 100);
  edit.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);

  hlayout->addWidget(&btna);
  hlayout->addWidget(&edit);
  hlayout->addWidget(&btn);

  QObject::connect(&btna, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("ExternalDeviceIPAuto"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 3) {
      value = 0;
    }
    QString values = QString::number(value);
    params.put("ExternalDeviceIPAuto", values.toStdString());
    refresh();
  });

  QObject::connect(&btn, &QPushButton::clicked, [=]() {
    QString eip_address = InputDialog::getText(tr("Input Your External Dev IP"), this, tr("See description for more detail how to set up."), false, 1, QString::fromStdString(params.get("ExternalDeviceIP")));
    if (eip_address.length() > 0) {
      params.put("ExternalDeviceIP", eip_address.toStdString());
    }
    refresh();
  });
  refresh();
}

void ExternalDeviceIP::refresh() {
  QString option = QString::fromStdString(params.get("ExternalDeviceIPAuto"));
  if (option == "0") {
    auto strs = QString::fromStdString(params.get("ExternalDeviceIP"));
    edit.setText(QString::fromStdString(strs.toStdString()));
    btn.setText(tr("SET"));
    btna.setText(tr("ManualInput"));
    btna.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
    )");
    btn.setEnabled(true);
    edit.setEnabled(true);
  } else if (option == "1") {
    btna.setText(tr("AutoDetect"));
    btna.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #00A12E;
    )");
    edit.setText("");
    btn.setText("");
    btn.setEnabled(false);
    edit.setEnabled(false);
  } else {
    btna.setText(tr("GatewayIP"));
    btna.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #00A1FF;
    )");
    edit.setText("");
    btn.setText("");
    btn.setEnabled(false);
    edit.setEnabled(false);
  }
}

DoNotDisturbMode::DoNotDisturbMode() : AbstractControl(tr("DoNotDisturb Mode"), tr("Off Event notification, Screen and Sound of Device. You can enable this touching Left-Top Box like a button on onroad screen."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("◀");
  btnplus.setText("▶");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("DoNotDisturbMode"));
    int value = str.toInt();
    value = value - 1;
    if (value <= -1) {
      value = 3;
    }
    QString values = QString::number(value);
    params.put("DoNotDisturbMode", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("DoNotDisturbMode"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 4) {
      value = 0;
    }
    QString values = QString::number(value);
    params.put("DoNotDisturbMode", values.toStdString());
    refresh();
  });
  refresh();
}

void DoNotDisturbMode::refresh() {
  QString option = QString::fromStdString(params.get("DoNotDisturbMode"));
  if (option == "0") {
    label.setText(tr("NotUse"));
    uiState()->scene.do_not_disturb_mode = 0;
  } else if (option == "1") {
    label.setText(tr("SCROffOnly"));
    uiState()->scene.do_not_disturb_mode = 1;
  } else if (option == "2") {
    label.setText(tr("SNDOffOnly"));
    uiState()->scene.do_not_disturb_mode = 2;
  } else {
    label.setText(tr("BothOff"));
    uiState()->scene.do_not_disturb_mode = 3;
  }
}

CruiseGapBySpd::CruiseGapBySpd() : AbstractControl("", "", "") {
  label1.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  label1.setStyleSheet("color: #e0e879");
  label2.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  label2.setStyleSheet("color: #e0e879");
  label3.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  label3.setStyleSheet("color: #e0e879");
  btn1.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 50px;
    font-weight: 500;
    color: #e0e879;
    background-color: #808080;
  )");
  btn2.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 50px;
    font-weight: 500;
    color: #e0e879;
    background-color: #808080;
  )");
  btn3.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 50px;
    font-weight: 500;
    color: #e0e879;
    background-color: #808080;
  )");
  btn4.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 50px;
    font-weight: 500;
    color: #e0e879;
    background-color: #808080;
  )");
  btnminus1.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus1.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus2.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus2.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus3.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus3.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus1.setFixedSize(90, 100);
  btnplus1.setFixedSize(90, 100);
  btnminus2.setFixedSize(90, 100);
  btnplus2.setFixedSize(90, 100);
  btnminus3.setFixedSize(90, 100);
  btnplus3.setFixedSize(90, 100);
  btn1.setFixedSize(110, 100);
  btn2.setFixedSize(110, 100);
  btn3.setFixedSize(110, 100);
  btn4.setFixedSize(110, 100);

  hlayout->addWidget(&btn1);
  hlayout->addWidget(&btnminus1);
  hlayout->addWidget(&label1);
  hlayout->addWidget(&btnplus1);
  hlayout->addWidget(&btn2);
  hlayout->addWidget(&btnminus2);
  hlayout->addWidget(&label2);
  hlayout->addWidget(&btnplus2);
  hlayout->addWidget(&btn3);
  hlayout->addWidget(&btnminus3);
  hlayout->addWidget(&label3);
  hlayout->addWidget(&btnplus3);
  hlayout->addWidget(&btn4);

  btnminus1.setText("－");
  btnplus1.setText("＋");
  btnminus2.setText("－");
  btnplus2.setText("＋");
  btnminus3.setText("－");
  btnplus3.setText("＋");

  QObject::connect(&btn1, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("CruiseGapBySpdGap")).split(",");
    int value = list[0].toInt();
    value = value + 1;
    if (value >= 5) {
      value = 1;
    }
    QString values = QString::number(value);
    list[0] = values;
    QString str = list.join(",");
    params.put("CruiseGapBySpdGap", str.toStdString());
    refresh1();
  });

  QObject::connect(&btn2, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("CruiseGapBySpdGap")).split(",");
    int value = list[1].toInt();
    value = value + 1;
    if (value >= 5) {
      value = 1;
    }
    QString values = QString::number(value);
    list[1] = values;
    QString str = list.join(",");
    params.put("CruiseGapBySpdGap", str.toStdString());
    refresh2();
  });

  QObject::connect(&btn3, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("CruiseGapBySpdGap")).split(",");
    int value = list[2].toInt();
    value = value + 1;
    if (value >= 5) {
      value = 1;
    }
    QString values = QString::number(value);
    list[2] = values;
    QString str = list.join(",");
    params.put("CruiseGapBySpdGap", str.toStdString());
    refresh3();
  });

  QObject::connect(&btn4, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("CruiseGapBySpdGap")).split(",");
    int value = list[3].toInt();
    value = value + 1;
    if (value >= 5) {
      value = 1;
    }
    QString values = QString::number(value);
    list[3] = values;
    QString str = list.join(",");
    params.put("CruiseGapBySpdGap", str.toStdString());
    refresh4();
  });

  QObject::connect(&btnminus1, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("CruiseGapBySpdSpd")).split(",");
    int value = list[0].toInt();
    value = value - 5;
    if (value <= 0) {
      value = 0;
    }
    QString values = QString::number(value);
    list[0] = values;
    QString str = list.join(",");
    params.put("CruiseGapBySpdSpd", str.toStdString());
    refresh5();
  });

  QObject::connect(&btnplus1, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("CruiseGapBySpdSpd")).split(",");
    int value = list[0].toInt();
    int valuem = list[1].toInt();
    value = value + 5;
    if (value >= (valuem - 5)) {
      value = valuem - 5;
    }
    QString values = QString::number(value);
    list[0] = values;
    QString str = list.join(",");
    params.put("CruiseGapBySpdSpd", str.toStdString());
    refresh5();
  });

  QObject::connect(&btnminus2, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("CruiseGapBySpdSpd")).split(",");
    int value = list[1].toInt();
    int valuem = list[0].toInt();
    value = value - 5;
    if (value <= (valuem + 5)) {
      value = valuem + 5;
    }
    QString values = QString::number(value);
    list[1] = values;
    QString str = list.join(",");
    params.put("CruiseGapBySpdSpd", str.toStdString());
    refresh6();
  });

  QObject::connect(&btnplus2, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("CruiseGapBySpdSpd")).split(",");
    int value = list[1].toInt();
    int valuem = list[2].toInt();
    value = value + 5;
    if (value >= (valuem - 5)) {
      value = valuem - 5;
    }
    QString values = QString::number(value);
    list[1] = values;
    QString str = list.join(",");
    params.put("CruiseGapBySpdSpd", str.toStdString());
    refresh6();
  });

  QObject::connect(&btnminus3, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("CruiseGapBySpdSpd")).split(",");
    int value = list[2].toInt();
    int valuem = list[1].toInt();
    value = value - 5;
    if (value <= (valuem + 5)) {
      value = valuem + 5;
    }
    QString values = QString::number(value);
    list[2] = values;
    QString str = list.join(",");
    params.put("CruiseGapBySpdSpd", str.toStdString());
    refresh7();
  });

  QObject::connect(&btnplus3, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("CruiseGapBySpdSpd")).split(",");
    int value = list[2].toInt();
    value = value + 5;
    if (value >= 160) {
      value = 160;
    }
    QString values = QString::number(value);
    list[2] = values;
    QString str = list.join(",");
    params.put("CruiseGapBySpdSpd", str.toStdString());
    refresh7();
  });

  refresh1();
  refresh2();
  refresh3();
  refresh4();
  refresh5();
  refresh6();
  refresh7();
}

void CruiseGapBySpd::refresh1() {
  QStringList list = QString::fromStdString(params.get("CruiseGapBySpdGap")).split(",");
  btn1.setText(list[0]);
}

void CruiseGapBySpd::refresh2() {
  QStringList list = QString::fromStdString(params.get("CruiseGapBySpdGap")).split(",");
  btn2.setText(list[1]);
}

void CruiseGapBySpd::refresh3() {
  QStringList list = QString::fromStdString(params.get("CruiseGapBySpdGap")).split(",");
  btn3.setText(list[2]);
}

void CruiseGapBySpd::refresh4() {
  QStringList list = QString::fromStdString(params.get("CruiseGapBySpdGap")).split(",");
  btn4.setText(list[3]);
}

void CruiseGapBySpd::refresh5() {
  QStringList list = QString::fromStdString(params.get("CruiseGapBySpdSpd")).split(",");
  label1.setText(list[0]);
}

void CruiseGapBySpd::refresh6() {
  QStringList list = QString::fromStdString(params.get("CruiseGapBySpdSpd")).split(",");
  label2.setText(list[1]);
}

void CruiseGapBySpd::refresh7() {
  QStringList list = QString::fromStdString(params.get("CruiseGapBySpdSpd")).split(",");
  label3.setText(list[2]);
}

CruiseSetwithRoadLimitSpeedOffset::CruiseSetwithRoadLimitSpeedOffset() : AbstractControl(tr("CruiseSet RoadLimitSpd Ofs"), tr("CruiseSet with RoadLimitSpeed Offset Value. This will add offset to navi road limit speed."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("CruiseSetwithRoadLimitSpeedOffset"));
    int value = str.toInt();
    value = value - 1;
    if (value <= -1) {
      value = 0;
    }
    QString values = QString::number(value);
    params.put("CruiseSetwithRoadLimitSpeedOffset", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("CruiseSetwithRoadLimitSpeedOffset"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 50) {
      value = 50;
    }
    QString values = QString::number(value);
    params.put("CruiseSetwithRoadLimitSpeedOffset", values.toStdString());
    refresh();
  });
  refresh();
}

void CruiseSetwithRoadLimitSpeedOffset::refresh() {
  label.setText(QString::fromStdString(params.get("CruiseSetwithRoadLimitSpeedOffset")));
}

LongAlternative::LongAlternative() : AbstractControl(tr("Long for BUS2"), tr("Long for Bus 2. If your radar is on bus2, choose mode 1 or mode 2."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KISALongAlt"));
    int value = str.toInt();
    value = value - 1;
    if (value <= -1) {
      value = 2;
    }
    QString values = QString::number(value);
    params.put("KISALongAlt", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KISALongAlt"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 3) {
      value = 0;
    }
    QString values = QString::number(value);
    params.put("KISALongAlt", values.toStdString());
    refresh();
  });
  refresh();
}

void LongAlternative::refresh() {
  QString option = QString::fromStdString(params.get("KISALongAlt"));
  if (option == "1") {
    label.setText(tr("Mode 1"));
  } else if (option == "2") {
    label.setText(tr("Mode 2"));
  } else {
    label.setText(tr("None"));
  }
}

MapboxToken::MapboxToken() : AbstractControl(tr("MapboxToken"), tr("MapboxToken"), "") {
  btn.setStyleSheet(R"(
    padding: -10;
    border-radius: 35px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  edit.setStyleSheet(R"(
    background-color: grey;
    font-size: 55px;
    font-weight: 500;
    height: 120px;
  )");
  edit.setFixedSize(900, 100);
  btn.setFixedSize(200, 100);
  hlayout->addWidget(&edit);
  hlayout->addWidget(&btn);

  QObject::connect(&btn, &QPushButton::clicked, [=]() {
    QString targetvalue = InputDialog::getText(tr("MapboxToken"), this, tr("Put your MapboxToken starting with pk."), false, 1, QString::fromStdString(params.get("MapboxToken")));
    if (targetvalue.length() > 0 && targetvalue != QString::fromStdString(params.get("MapboxToken"))) {
      params.put("MapboxToken", targetvalue.toStdString());
      refresh();
    }
   });
  refresh();
}

void MapboxToken::refresh() {
  auto strs = QString::fromStdString(params.get("MapboxToken"));
  edit.setText(QString::fromStdString(strs.toStdString()));
  btn.setText(tr("SET"));
}


CruiseSpammingLevel::CruiseSpammingLevel() : AbstractControl("", "", "") {
  label1.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  label1.setStyleSheet("color: #e0e879");
  label2.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  label2.setStyleSheet("color: #e0e879");
  label3.setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
  label3.setStyleSheet("color: #e0e879");
  btn1.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 50px;
    font-weight: 500;
    color: #e0e879;
    background-color: #808080;
  )");
  btn2.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 50px;
    font-weight: 500;
    color: #e0e879;
    background-color: #808080;
  )");
  btn3.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 50px;
    font-weight: 500;
    color: #e0e879;
    background-color: #808080;
  )");
  btn4.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 50px;
    font-weight: 500;
    color: #e0e879;
    background-color: #808080;
  )");
  btnminus1.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus1.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus2.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus2.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus3.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus3.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus1.setFixedSize(90, 100);
  btnplus1.setFixedSize(90, 100);
  btnminus2.setFixedSize(90, 100);
  btnplus2.setFixedSize(90, 100);
  btnminus3.setFixedSize(90, 100);
  btnplus3.setFixedSize(90, 100);
  btn1.setFixedSize(110, 100);
  btn2.setFixedSize(110, 100);
  btn3.setFixedSize(110, 100);
  btn4.setFixedSize(110, 100);

  hlayout->addWidget(&btn1);
  hlayout->addWidget(&btnminus1);
  hlayout->addWidget(&label1);
  hlayout->addWidget(&btnplus1);
  hlayout->addWidget(&btn2);
  hlayout->addWidget(&btnminus2);
  hlayout->addWidget(&label2);
  hlayout->addWidget(&btnplus2);
  hlayout->addWidget(&btn3);
  hlayout->addWidget(&btnminus3);
  hlayout->addWidget(&label3);
  hlayout->addWidget(&btnplus3);
  hlayout->addWidget(&btn4);

  btnminus1.setText("－");
  btnplus1.setText("＋");
  btnminus2.setText("－");
  btnplus2.setText("＋");
  btnminus3.setText("－");
  btnplus3.setText("＋");

  QObject::connect(&btn1, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("CruiseSpammingLevel")).split(",");
    int value = list[0].toInt();
    value = value + 1;
    if (value >= 17) {
      value = 0;
    }
    QString values = QString::number(value);
    list[0] = values;
    QString str = list.join(",");
    params.put("CruiseSpammingLevel", str.toStdString());
    refresh1();
  });

  QObject::connect(&btn2, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("CruiseSpammingLevel")).split(",");
    int value = list[1].toInt();
    value = value + 1;
    if (value >= 17) {
      value = 0;
    }
    QString values = QString::number(value);
    list[1] = values;
    QString str = list.join(",");
    params.put("CruiseSpammingLevel", str.toStdString());
    refresh2();
  });

  QObject::connect(&btn3, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("CruiseSpammingLevel")).split(",");
    int value = list[2].toInt();
    value = value + 1;
    if (value >= 17) {
      value = 0;
    }
    QString values = QString::number(value);
    list[2] = values;
    QString str = list.join(",");
    params.put("CruiseSpammingLevel", str.toStdString());
    refresh3();
  });

  QObject::connect(&btn4, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("CruiseSpammingLevel")).split(",");
    int value = list[3].toInt();
    value = value + 1;
    if (value >= 17) {
      value = 0;
    }
    QString values = QString::number(value);
    list[3] = values;
    QString str = list.join(",");
    params.put("CruiseSpammingLevel", str.toStdString());
    refresh4();
  });

  QObject::connect(&btnminus1, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("CruiseSpammingSpd")).split(",");
    int value = list[0].toInt();
    value = value - 5;
    if (value <= 0) {
      value = 0;
    }
    QString values = QString::number(value);
    list[0] = values;
    QString str = list.join(",");
    params.put("CruiseSpammingSpd", str.toStdString());
    refresh5();
  });

  QObject::connect(&btnplus1, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("CruiseSpammingSpd")).split(",");
    int value = list[0].toInt();
    int valuem = list[1].toInt();
    value = value + 5;
    if (value >= (valuem - 5)) {
      value = valuem - 5;
    }
    QString values = QString::number(value);
    list[0] = values;
    QString str = list.join(",");
    params.put("CruiseSpammingSpd", str.toStdString());
    refresh5();
  });

  QObject::connect(&btnminus2, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("CruiseSpammingSpd")).split(",");
    int value = list[1].toInt();
    int valuem = list[0].toInt();
    value = value - 5;
    if (value <= (valuem + 5)) {
      value = valuem + 5;
    }
    QString values = QString::number(value);
    list[1] = values;
    QString str = list.join(",");
    params.put("CruiseSpammingSpd", str.toStdString());
    refresh6();
  });

  QObject::connect(&btnplus2, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("CruiseSpammingSpd")).split(",");
    int value = list[1].toInt();
    int valuem = list[2].toInt();
    value = value + 5;
    if (value >= (valuem - 5)) {
      value = valuem - 5;
    }
    QString values = QString::number(value);
    list[1] = values;
    QString str = list.join(",");
    params.put("CruiseSpammingSpd", str.toStdString());
    refresh6();
  });

  QObject::connect(&btnminus3, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("CruiseSpammingSpd")).split(",");
    int value = list[2].toInt();
    int valuem = list[1].toInt();
    value = value - 5;
    if (value <= (valuem + 5)) {
      value = valuem + 5;
    }
    QString values = QString::number(value);
    list[2] = values;
    QString str = list.join(",");
    params.put("CruiseSpammingSpd", str.toStdString());
    refresh7();
  });

  QObject::connect(&btnplus3, &QPushButton::clicked, [=]() {
    QStringList list = QString::fromStdString(params.get("CruiseSpammingSpd")).split(",");
    int value = list[2].toInt();
    value = value + 5;
    if (value >= 160) {
      value = 160;
    }
    QString values = QString::number(value);
    list[2] = values;
    QString str = list.join(",");
    params.put("CruiseSpammingSpd", str.toStdString());
    refresh7();
  });

  refresh1();
  refresh2();
  refresh3();
  refresh4();
  refresh5();
  refresh6();
  refresh7();
}

void CruiseSpammingLevel::refresh1() {
  QStringList list = QString::fromStdString(params.get("CruiseSpammingLevel")).split(",");
  btn1.setText(list[0]);
}

void CruiseSpammingLevel::refresh2() {
  QStringList list = QString::fromStdString(params.get("CruiseSpammingLevel")).split(",");
  btn2.setText(list[1]);
}

void CruiseSpammingLevel::refresh3() {
  QStringList list = QString::fromStdString(params.get("CruiseSpammingLevel")).split(",");
  btn3.setText(list[2]);
}

void CruiseSpammingLevel::refresh4() {
  QStringList list = QString::fromStdString(params.get("CruiseSpammingLevel")).split(",");
  btn4.setText(list[3]);
}

void CruiseSpammingLevel::refresh5() {
  QStringList list = QString::fromStdString(params.get("CruiseSpammingSpd")).split(",");
  label1.setText(list[0]);
}

void CruiseSpammingLevel::refresh6() {
  QStringList list = QString::fromStdString(params.get("CruiseSpammingSpd")).split(",");
  label2.setText(list[1]);
}

void CruiseSpammingLevel::refresh7() {
  QStringList list = QString::fromStdString(params.get("CruiseSpammingSpd")).split(",");
  label3.setText(list[2]);
}

KISACruiseGapSet::KISACruiseGapSet() : AbstractControl(tr("Cruise Gap(Init)"), tr("Set initial Cruise Gap for OP Long"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("◀");
  btnplus.setText("▶");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaCruiseGapSet"));
    int value = str.toInt();
    value = value - 1;
    if (value <= -1) {
      value = 4;
    }
    QString values = QString::number(value);
    params.put("KisaCruiseGapSet", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KisaCruiseGapSet"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 5) {
      value = 1;
    }
    QString values = QString::number(value);
    params.put("KisaCruiseGapSet", values.toStdString());
    refresh();
  });
  refresh();
}

void KISACruiseGapSet::refresh() {
  QString option = QString::fromStdString(params.get("KisaCruiseGapSet"));
  if (option == "1") {
    label.setText(QString::fromStdString("■"));
  } else if (option == "2") {
    label.setText(QString::fromStdString("■■"));
  } else if (option == "3") {
    label.setText(QString::fromStdString("■■■"));
  } else {
    label.setText(QString::fromStdString("■■■■"));
  }
}

UseLegacyLaneModel::UseLegacyLaneModel() : AbstractControl(tr("Lateral Plan Mode"), tr("1.Model(latest model path), 2.MPC(mpc path from post processing of model, 3.Mix(Model(high curvature), MPC(low curvature), interpolation value)"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("◀");
  btnplus.setText("▶");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("UseLegacyLaneModel"));
    int value = str.toInt();
    value = value - 1;
    if (value <= -1) {
      value = 2;
    }
    QString values = QString::number(value);
    params.put("UseLegacyLaneModel", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("UseLegacyLaneModel"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 3) {
      value = 0;
    }
    QString values = QString::number(value);
    params.put("UseLegacyLaneModel", values.toStdString());
    refresh();
  });
  refresh();
}

void UseLegacyLaneModel::refresh() {
  QString option = QString::fromStdString(params.get("UseLegacyLaneModel"));
  if (option == "0") {
    label.setText(tr("Model"));
  } else if (option == "1") {
    label.setText(tr("MPC"));
  } else {
    label.setText(tr("Mix"));
  }
}

KISACruiseSpammingInterval::KISACruiseSpammingInterval() : AbstractControl(tr("Cruise Spamming Interval"), tr("Adjust Cruise Spamming Interval if SCC SetSpeed is not changed appropriately. Low values can make SetSpeed quickly, but could make cluster(CAN) error. Default Value: 7"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KISACruiseSpammingInterval"));
    int value = str.toInt();
    value = value - 1;
    if (value <= -1) {
      value = 20;
    }
    QString values = QString::number(value);
    params.put("KISACruiseSpammingInterval", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KISACruiseSpammingInterval"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 21) {
      value = 0;
    }
    QString values = QString::number(value);
    params.put("KISACruiseSpammingInterval", values.toStdString());
    refresh();
  });
  refresh();
}

void KISACruiseSpammingInterval::refresh() {
  label.setText(QString::fromStdString(params.get("KISACruiseSpammingInterval")));
}

KISACruiseSpammingBtnCount::KISACruiseSpammingBtnCount() : AbstractControl(tr("Cruise Spamming Btn Count"), tr("Increase Count if SCC SetSpeed is not changed appropriately, but could make cluster(CAN) error. Default Value: 2"), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KISACruiseSpammingBtnCount"));
    int value = str.toInt();
    value = value - 1;
    if (value <= 0) {
      value = 25;
    }
    QString values = QString::number(value);
    params.put("KISACruiseSpammingBtnCount", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("KISACruiseSpammingBtnCount"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 26) {
      value = 1;
    }
    QString values = QString::number(value);
    params.put("KISACruiseSpammingBtnCount", values.toStdString());
    refresh();
  });
  refresh();
}

void KISACruiseSpammingBtnCount::refresh() {
  label.setText(QString::fromStdString(params.get("KISACruiseSpammingBtnCount")));
}

RegenBrakeFeature::RegenBrakeFeature() : AbstractControl("", "", "") {

  btn1.setFixedSize(150, 100);
  btn2.setFixedSize(150, 100);
  btn3.setFixedSize(150, 100);
  btn1.setText("ST");
  btn2.setText("AT");
  btn3.setText("EE");
  hlayout->addWidget(&btn1);
  hlayout->addWidget(&btn2);
  hlayout->addWidget(&btn3);

  QObject::connect(&btn1, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("RegenBrakeFeature"));
    bool is_value = str.contains("1");
    if (is_value) {
      QString values = str.replace("1", "");
      params.put("RegenBrakeFeature", values.toStdString());
    } else {
      QString values = str + "1";
      params.put("RegenBrakeFeature", values.toStdString());
    }
    refresh();
  });
  QObject::connect(&btn2, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("RegenBrakeFeature"));
    bool is_value = str.contains("2");
    if (is_value) {
      QString values = str.replace("2", "");
      params.put("RegenBrakeFeature", values.toStdString());
    } else {
      QString values = str + "2";
      params.put("RegenBrakeFeature", values.toStdString());
    }
    refresh();
  });
  QObject::connect(&btn3, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("RegenBrakeFeature"));
    bool is_value = str.contains("3");
    if (is_value) {
      QString values = str.replace("3", "");
      params.put("RegenBrakeFeature", values.toStdString());
    } else {
      QString values = str + "3";
      params.put("RegenBrakeFeature", values.toStdString());
    }
    refresh();
  });
  refresh();
}

void RegenBrakeFeature::refresh() {
  QString option = QString::fromStdString(params.get("RegenBrakeFeature"));
  if (option.contains("1")) {
    btn1.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #00A12E;
    )");
  } else {
    btn1.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
    )");
  }
  if (option.contains("2")) {
    btn2.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #00A12E;
    )");
  } else {
    btn2.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
    )");
  }
  if (option.contains("3")) {
    btn3.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #00A12E;
    )");
  } else {
    btn3.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
    )");
  }
}

SetSpeedPlus::SetSpeedPlus() : AbstractControl(tr("SetSpeed Changed by Num"), tr("MAX Speed can be adjusted by number. Cruise Set Speed will be set as same with MAX quickly."), "../assets/offroad/icon_shell.png") {

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnminus.setFixedSize(150, 100);
  btnplus.setFixedSize(150, 100);
  btnminus.setText("-");
  btnplus.setText("+");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("SetSpeedPlus"));
    int value = str.toInt();
    value = value - 1;
    if (value <= -1) {
      value = 20;
    }
    QString values = QString::number(value);
    params.put("SetSpeedPlus", values.toStdString());
    refresh();
  });
  
  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    auto str = QString::fromStdString(params.get("SetSpeedPlus"));
    int value = str.toInt();
    value = value + 1;
    if (value >= 21) {
      value = 0;
    }
    QString values = QString::number(value);
    params.put("SetSpeedPlus", values.toStdString());
    refresh();
  });
  refresh();
}

void SetSpeedPlus::refresh() {
  QString option = QString::fromStdString(params.get("SetSpeedPlus"));
  if (option == "0") {
    label.setText(tr("NotUse"));
  } else {
    label.setText(QString::fromStdString(params.get("SetSpeedPlus")));
  }
}