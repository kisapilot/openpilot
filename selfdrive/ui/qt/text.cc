#include <QApplication>
#include <QLabel>
#include <QPushButton>
#include <QScrollBar>
#include <QVBoxLayout>
#include <QWidget>
#include <QProcess>
#include <QHostAddress>
#include <QNetworkInterface>
#include <QAbstractSocket>

#include "system/hardware/hw.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/qt_window.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"
#include "common/params.h"

int main(int argc, char *argv[]) {
  initApp(argc, argv);
  QApplication a(argc, argv);
  QWidget window;
  setMainWindow(&window);

  QGridLayout *main_layout = new QGridLayout(&window);
  main_layout->setMargin(50);

  QLabel *label = new QLabel(argv[1]);
  label->setWordWrap(true);
  label->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);
  ScrollView *scroll = new ScrollView(label);
  scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  main_layout->addWidget(scroll, 0, 0, 1, 5, Qt::AlignTop);

  // Scroll to the bottom
  QObject::connect(scroll->verticalScrollBar(), &QAbstractSlider::rangeChanged, [=]() {
    scroll->verticalScrollBar()->setValue(scroll->verticalScrollBar()->maximum());
  });

  QPushButton *btn = new QPushButton();
#ifdef __aarch64__
  QPushButton *btn2 = new QPushButton();
  QPushButton *btn3 = new QPushButton();
  QPushButton *btn4 = new QPushButton();
  QPushButton *btn5 = new QPushButton();
  QLabel *label2 = new QLabel();
  QString device_ip = "---";
  const QHostAddress &localhost = QHostAddress(QHostAddress::LocalHost);
  for (const QHostAddress &address: QNetworkInterface::allAddresses()) {
    if (address.protocol() == QAbstractSocket::IPv4Protocol && address != localhost)
      device_ip = address.toString();
  }
  label2->setText(device_ip);
  label2->setStyleSheet("color: #e0e879");
  main_layout->addWidget(label2, 0, 0, 1, 5, Qt::AlignRight | Qt::AlignTop);
  btn->setText(QObject::tr("Reboot"));
  btn2->setText(QObject::tr("Update"));
  btn3->setText(QObject::tr("Rollback"));
  btn4->setText(QObject::tr("Reset"));
  btn5->setText(QObject::tr("InitParam"));
  QObject::connect(btn, &QPushButton::clicked, [=]() {
    Hardware::reboot();
  });
  QObject::connect(btn2, &QPushButton::clicked, [=]() {
    btn2->setEnabled(false);
    QProcess::execute("touch /data/kisa_compiling");
    QProcess::execute("/data/openpilot/selfdrive/assets/addon/script/gitpull.sh");
    Hardware::reboot();
  });
  QObject::connect(btn3, &QPushButton::clicked, [=]() {
    btn3->setEnabled(false);
    QProcess::execute("touch /data/kisa_compiling");
    QString cmd = "git reset --hard " + QString::fromStdString(Params().get("GitCommit"));
    QProcess::execute(cmd);
    Hardware::reboot();
  });
  QObject::connect(btn4, &QPushButton::clicked, [=]() {
    btn4->setEnabled(false);
    QProcess::execute("touch /data/kisa_compiling");
    QProcess::execute("/data/openpilot/selfdrive/assets/addon/script/git_reset.sh");
    Hardware::reboot();
  });
  QObject::connect(btn5, &QPushButton::clicked, [=]() {
    btn5->setEnabled(false);
    QProcess::execute("touch /data/kisa_compiling");
    QProcess::execute("/data/openpilot/selfdrive/assets/addon/script/init_param.sh");
    Hardware::reboot();
  });
  btn2->setFixedSize(350, 150);
  btn3->setFixedSize(350, 150);
  btn4->setFixedSize(350, 150);
  btn5->setFixedSize(350, 150);
  main_layout->addWidget(btn2, 1, 0, 1, 1, Qt::AlignCenter | Qt::AlignBottom);
  main_layout->addWidget(btn3, 1, 1, 1, 1, Qt::AlignCenter | Qt::AlignBottom);
  main_layout->addWidget(btn4, 1, 2, 1, 1, Qt::AlignCenter | Qt::AlignBottom);
  main_layout->addWidget(btn5, 1, 3, 1, 1, Qt::AlignCenter | Qt::AlignBottom);
#else
  btn->setText(QObject::tr("Exit"));
  QObject::connect(btn, &QPushButton::clicked, &a, &QApplication::quit);
#endif
  btn->setFixedSize(350, 150);
  main_layout->addWidget(btn, 1, 4, 1, 1, Qt::AlignCenter | Qt::AlignBottom);

  window.setStyleSheet(R"(
    * {
      outline: none;
      color: white;
      background-color: black;
      font-size: 50px;
    }
    QPushButton {
      padding: 30px;
      padding-right: 30px;
      padding-left: 30px;
      border: 2px solid white;
      border-radius: 20px;
      margin-right: 30px;
    }
  )");

  return a.exec();
}
