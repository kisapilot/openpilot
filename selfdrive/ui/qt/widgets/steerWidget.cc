
#include <QDialog>
#include <QDateTime>
#include <QHBoxLayout>
#include <QMouseEvent>
#include <QVBoxLayout>

#include "selfdrive/ui/qt/widgets/steerWidget.h"



CSteerWidget::CSteerWidget(QWidget *parent) : QFrame(parent) 
{
  m_bShow = 0;

  auto str = QString::fromStdString( params.get("KisaSteerMethod") );
  int value = str.toInt();
  m_nSelect = value; 

  main_layout = new QVBoxLayout(this);
  main_layout->setMargin(0);


  hlayout = new QHBoxLayout;
  hlayout->setMargin(0);
  hlayout->setSpacing(20);

  // left icon 
  pix_plus =  QPixmap( "../assets/offroad/icon_plus.png" ).scaledToWidth(80, Qt::SmoothTransformation);
  pix_minus =  QPixmap( "../assets/offroad/icon_minus.png" ).scaledToWidth(80, Qt::SmoothTransformation);


  icon_label = new QLabel();
  icon_label->setPixmap(pix_plus );
  icon_label->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
  hlayout->addWidget(icon_label);

  // title
  QString  title = tr("Steer control Method");
  title_label = new QPushButton(title);
  title_label->setFixedHeight(120);
  title_label->setStyleSheet("font-size: 50px; font-weight: 400; text-align: left");
  hlayout->addWidget(title_label);

  connect(title_label, &QPushButton::clicked, [=]() {

    if( m_bShow )  m_bShow = 0;
    else   m_bShow = 1;
    refresh();
  });

  // label
  method_label = new QPushButton(tr("method")); // .setAlignment(Qt::AlignVCenter|Qt::AlignHCenter);
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
    m_nSelect += 1;
    if( m_nSelect > 1 )
      m_nSelect = 0;

    QString values = QString::number(m_nSelect);
    params.put("KisaSteerMethod", values.toStdString());      
    refresh();
  });

  main_layout->addLayout(hlayout);


  FrameSmooth( parent );
  FrameNormal( parent );


  main_layout->addStretch();
  refresh();
}

CSteerWidget::~CSteerWidget()
{

}


void CSteerWidget::FrameSmooth(QWidget *parent) 
{
 // 1. layer#1 menu
  m_pChildFrame1 = new QFrame(); 
  m_pChildFrame1->setContentsMargins(40, 10, 40, 50);
  m_pChildFrame1->setStyleSheet(R"(
    * {
      padding: 0;
      border-radius: 50px;
      font-size: 35px;
      font-weight: 500;
      color: #E4E4E4;
      background-color: black;
    } 
  )");
  
  main_layout->addWidget(m_pChildFrame1);
  QVBoxLayout *menu_layout = new QVBoxLayout(m_pChildFrame1);
 // menu_layout->setContentsMargins(32, 5, 32, 32);


  MenuControl *pMenu1 = new MenuControl( 
    "KisaMaxSteeringAngle",
    tr("Driver to Steer Angle"),
    tr("Improve the edge between the driver and the openpilot."),
    "../assets/offroad/icon_shell.png"    
    );
  pMenu1->SetControl( 10, 180, 5 );
  menu_layout->addWidget( pMenu1 );

  
   MenuControl *pMenu2 = new MenuControl( 
    "KisaMaxDriverAngleWait",
    tr("Driver to Steer"),
    tr("Controls smooth torque by the driver  From KisaMaxSteeringAngle. def:0.002(5sec)"),
    "../assets/offroad/icon_shell.png"    
    );
  pMenu2->SetControl( 0, 1, 0.001 );
  pMenu2->SetString( 0, "Not");
  menu_layout->addWidget( pMenu2 ); 


   MenuControl *pMenu3 = new MenuControl( 
    "KisaMaxSteerAngleWait" ,
    tr("Steer Angle"),
    tr("Controls torque by steering angle From KisaMaxSteeringAngle. def:0.001(10sec)"),
    "../assets/offroad/icon_shell.png"    
    );
  pMenu3->SetControl( 0, 1, 0.001 );
  pMenu3->SetString( 0, "Not");
  menu_layout->addWidget( pMenu3 ); 

   MenuControl *pMenu4 = new MenuControl( 
    "KisaDriverAngleWait" ,
    tr("Normal driver to Steer"),
    tr("Controls torque limitation due to normal driver handle intervention. def:0.001(10sec)"),
    "../assets/offroad/icon_shell.png"
    );
  pMenu4->SetControl( 0, 1, 0.001 );
  pMenu4->SetString( 0, "Not");
  menu_layout->addWidget( pMenu4 ); 

  // Update + Cancel buttons
  /*
  QHBoxLayout *btn_layout = new QHBoxLayout();
  btn_layout->setSpacing(30);
  menu_layout->addLayout(btn_layout);

    QPushButton* confirm_btn = new QPushButton("confirm");
   // confirm_btn->setFixedHeight(120);
    confirm_btn->setStyleSheet(R"(
      QPushButton {
        height: 120px;
        border-radius: 15px;
        background-color: gray;
      }
      * {
        font-size: 50px; 
        font-weight: 400; 
        text-align: left;
      }
    )");  
    btn_layout->addWidget(confirm_btn);
    QObject::connect(confirm_btn, &QPushButton::clicked, [=]() {
     // pmyWidget->setVisible(false);
     //m_pChildFrame1->hide();
     m_bShow = 0;
     refresh();
    });    



    QPushButton* cancel_btn = new QPushButton("cancel");
   // cancel_btn->setFixedHeight(120);
    cancel_btn->setStyleSheet(R"(
      QPushButton {
        height: 120px;
        border-radius: 15px;
        background-color: gray;
      }
      * {
        font-size: 50px; 
        font-weight: 400; 
        text-align: left;
      }
    )");  
    btn_layout->addWidget(cancel_btn);
    QObject::connect(cancel_btn, &QPushButton::clicked, [=]() {
     // pmyWidget->setVisible(false);
     //m_pChildFrame1->hide();
     m_bShow = 0;
     refresh();
    });
    */
}

void CSteerWidget::FrameNormal(QWidget *parent) 
{
 // 1. layer#2 menu
  m_pChildFrame2 = new QFrame(); 
  m_pChildFrame2->setContentsMargins(40, 10, 40, 50);
  m_pChildFrame2->setStyleSheet(R"(
    * {
      padding: 0;
      border-radius: 50px;
      font-size: 35px;
      font-weight: 500;
      color: #E4E4E4;
      background-color: black;
    } 
  )");
  
  main_layout->addWidget(m_pChildFrame2);
  QVBoxLayout *menu_layout = new QVBoxLayout(m_pChildFrame2);  
  MenuControl *pMenu1 = new MenuControl( 
    "KisaMaxAngleLimit",
    tr("Max Steering Angle"),
    tr("Set the maximum steering angle of the handle where the openpilot is possible. Please note that some vehicles may experience errors if the angle is set above 90 degrees.")
    //"../assets/offroad/icon_chevron_right.png"    
    );
  pMenu1->SetControl( 80, 360, 10 );
  pMenu1->SetString( 80, "NoLimit");
  menu_layout->addWidget( pMenu1 );
}


void CSteerWidget::showEvent(QShowEvent *event) 
{
  refresh();
}

void CSteerWidget::hideEvent(QHideEvent *event) 
{
  m_bShow = 0;
  refresh();
}

void CSteerWidget::refresh() 
{
  QString str;

  switch( m_nSelect )
  {
    case 0 : str = tr("0.Normal"); break;
    case 1 : str = tr("1.Smooth"); break;
    default: str = tr("2.Empty");  break;
  }


  method_label->setText( str );


  if(  m_bShow == 0 )
  {
    // pmyWidget->setVisible(false);
    m_pChildFrame1->hide();
    m_pChildFrame2->hide();
    icon_label->setPixmap(pix_plus);
  }
  else
  {
     if( m_nSelect == 0 )
     {
       m_pChildFrame2->show();
        m_pChildFrame1->hide();
     }
     else
     {
       m_pChildFrame1->show();
       m_pChildFrame2->hide();
     }

    
    icon_label->setPixmap(pix_minus);
    //pmyWidget->setVisible(true);
  }

}



