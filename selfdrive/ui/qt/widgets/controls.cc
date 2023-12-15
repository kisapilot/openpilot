#include "selfdrive/ui/qt/widgets/controls.h"

#include <QPainter>
#include <QStyleOption>

QFrame *horizontal_line(QWidget *parent) {
  QFrame *line = new QFrame(parent);
  line->setFrameShape(QFrame::StyledPanel);
  line->setStyleSheet(R"(
    margin-left: 40px;
    margin-right: 40px;
    border-width: 1px;
    border-bottom-style: solid;
    border-color: gray;
  )");
  line->setFixedHeight(2);
  return line;
}

AbstractControl::AbstractControl(const QString &title, const QString &desc, const QString &icon, QWidget *parent) : QFrame(parent) {
  QVBoxLayout *main_layout = new QVBoxLayout(this);
  main_layout->setMargin(0);

  hlayout = new QHBoxLayout;
  hlayout->setMargin(0);
  hlayout->setSpacing(20);

  // left icon
  icon_label = new QLabel(this);
  hlayout->addWidget(icon_label);
  if (!icon.isEmpty()) {
    icon_pixmap = QPixmap(icon).scaledToWidth(80, Qt::SmoothTransformation);
    icon_label->setPixmap(icon_pixmap);
    icon_label->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
  }
  icon_label->setVisible(!icon.isEmpty());

  // title
  title_label = new QPushButton(title);
  title_label->setFixedHeight(120);
  title_label->setStyleSheet("font-size: 50px; font-weight: 400; text-align: left; border: none;");
  // kisapilot
  if (!title.isEmpty()) {
    hlayout->addWidget(title_label, 1);
  }

  // value next to control button
  value = new ElidedLabel();
  value->setAlignment(Qt::AlignVCenter);
  value->setStyleSheet("color: #aaaaaa");
  hlayout->addWidget(value);

  main_layout->addLayout(hlayout);

  // description
  if (!desc.isEmpty()) {
    description = new QLabel(desc);
    description->setContentsMargins(40, 20, 40, 20);
    description->setStyleSheet("font-size: 40px; color: grey");
    description->setWordWrap(true);
    description->setVisible(false);
    main_layout->addWidget(description);

    connect(title_label, &QPushButton::clicked, [=]() {
      if (!description->isVisible()) {
        emit showDescriptionEvent();
      }

      if (!description->text().isEmpty()) {
        description->setVisible(!description->isVisible());
      }
    });
  }
  main_layout->addStretch();
}

void AbstractControl::hideEvent(QHideEvent *e) {
  if (description != nullptr) {
    description->hide();
  }
}

//  MenuControl

MenuControl::MenuControl( const QString &str_param, const QString &title, const QString &desc, const QString &icon, QWidget *parent ) 
  : AbstractControl( title, desc, icon, parent )
{
  m_nDelta = 10;
  m_nMax = 100;
  m_nMin = 0;
  m_nValue = 0;

  label.setAlignment(Qt::AlignVCenter|Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  auto str = QString::fromStdString( params.get( str_param.toStdString() ) );
  float value = str.toDouble();
  m_dValue = value;

  btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");

  btnminus.setFixedSize(150, 100);
  btnminus.setText("-");
  hlayout->addWidget(&btnminus);
  
  QObject::connect(&btnminus, &QPushButton::clicked, [=]() {
    m_dValue -= m_nDelta;
    if (m_dValue < m_nMin) {
      m_dValue = m_nMin;
    }
    QString values = QString::number(m_dValue);
    params.put( str_param.toStdString() , values.toStdString());

    refresh();
  });


  btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
  btnplus.setFixedSize(150, 100);
  btnplus.setText("+");
  hlayout->addWidget(&btnplus);

  QObject::connect(&btnplus, &QPushButton::clicked, [=]() {
    m_dValue += m_nDelta;
    if (m_dValue > m_nMax) {
      m_dValue = m_nMax;
    }
    QString values = QString::number(m_dValue);
    params.put( str_param.toStdString(), values.toStdString());
    refresh();
  });
  refresh();
}

void MenuControl::refresh() 
{
  auto str = QString::fromStdString( params.get("KisaSteerMethod") );
  int valuem = str.toInt();

  QString values = QString::number( m_dValue );

  int count = m_strList.size();
  if( count > 0 )
  {
    int  nMenu = m_dValue;
    
    if( 0 <= nMenu && nMenu <= count )
      values = m_strList[nMenu];
  }
  else if( !m_strValue.isEmpty() )
  {
   // float  fDelta = std::abs(m_nValue - m_dValue);
   // printf( "MenuControl %f  %f \n", m_nValue, fDelta);
    if( m_nValue == m_dValue  )
        values = m_strValue;
  }
  else if( valuem == 0 && m_dValue == 80 )
  {
	  values = "NoLimit";
  }
  else if( valuem == 1 && m_dValue == 0 )
  {
	  values = "Not";
  }


  label.setText( values );
}

void MenuControl::SetString( const QString strList )
{
  m_strList = strList.split(",");
}

void MenuControl::SetString( float nValue, const QString str )
{
  m_nValue = nValue;
  m_strValue = str;
}

// controls

ButtonControl::ButtonControl(const QString &title, const QString &text, const QString &desc, QWidget *parent) : AbstractControl(title, desc, "", parent) {
  btn.setText(text);
  btn.setStyleSheet(R"(
    QPushButton {
      padding: 0;
      border-radius: 50px;
      font-size: 35px;
      font-weight: 500;
      color: #E4E4E4;
      background-color: #393939;
    }
    QPushButton:pressed {
      background-color: #4a4a4a;
    }
    QPushButton:disabled {
      color: #33E4E4E4;
    }
  )");
  btn.setFixedSize(250, 100);
  QObject::connect(&btn, &QPushButton::clicked, this, &ButtonControl::clicked);
  hlayout->addWidget(&btn);
}

// ElidedLabel

ElidedLabel::ElidedLabel(QWidget *parent) : ElidedLabel({}, parent) {}

ElidedLabel::ElidedLabel(const QString &text, QWidget *parent) : QLabel(text.trimmed(), parent) {
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  setMinimumWidth(1);
}

void ElidedLabel::resizeEvent(QResizeEvent* event) {
  QLabel::resizeEvent(event);
  lastText_ = elidedText_ = "";
}

void ElidedLabel::paintEvent(QPaintEvent *event) {
  const QString curText = text();
  if (curText != lastText_) {
    elidedText_ = fontMetrics().elidedText(curText, Qt::ElideRight, contentsRect().width());
    lastText_ = curText;
  }

  QPainter painter(this);
  drawFrame(&painter);
  QStyleOption opt;
  opt.initFrom(this);
  style()->drawItemText(&painter, contentsRect(), alignment(), opt.palette, isEnabled(), elidedText_, foregroundRole());
}

// ParamControl

ParamControl::ParamControl(const QString &param, const QString &title, const QString &desc, const QString &icon, QWidget *parent)
    : ToggleControl(title, desc, icon, false, parent) {
  key = param.toStdString();
  QObject::connect(this, &ParamControl::toggleFlipped, this, &ParamControl::toggleClicked);
}

void ParamControl::toggleClicked(bool state) {
  auto do_confirm = [this]() {
    QString content("<body><h2 style=\"text-align: center;\">" + title_label->text() + "</h2><br>"
                    "<p style=\"text-align: center; margin: 0 128px; font-size: 50px;\">" + getDescription() + "</p></body>");
    return ConfirmationDialog(content, tr("Enable"), tr("Cancel"), true, this).exec();
  };

  bool confirmed = store_confirm && params.getBool(key + "Confirmed");
  if (!confirm || confirmed || !state || do_confirm()) {
    if (store_confirm && state) params.putBool(key + "Confirmed", true);
    params.putBool(key, state);
    setIcon(state);
  } else {
    toggle.togglePosition();
  }
}
