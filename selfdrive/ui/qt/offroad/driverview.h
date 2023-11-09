#pragma once

#include <QStackedLayout>

#include "selfdrive/ui/qt/widgets/cameraview.h"
#include <QTimer>
#include "selfdrive/ui/qt/screenrecorder/screenrecorder.h"

class DriverViewScene : public QWidget {
  Q_OBJECT

public:
  explicit DriverViewScene(QWidget *parent);

public slots:
  void frameUpdated();

protected:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;
  void paintEvent(QPaintEvent *event) override;

private:
  Params params;
  QPixmap face_img;
  bool is_rhd = false;
  bool frame_updated = false;

  // neokii screen recorder. thx for sharing your source. 
  ScreenRecoder* recorder;
  std::shared_ptr<QTimer> record_timer;
};

class DriverViewWindow : public QWidget {
  Q_OBJECT

public:
  explicit DriverViewWindow(QWidget *parent);

signals:
  void done();

protected:
  void mousePressEvent(QMouseEvent* e) override;
  void closeView();

  CameraWidget *cameraView;
  DriverViewScene *scene;
  QStackedLayout *layout;
  const QRect d_rec_btn = QRect(1800, 885, 140, 140);
};
