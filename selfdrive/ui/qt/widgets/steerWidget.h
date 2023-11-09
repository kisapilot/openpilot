

#pragma once

#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <QStackedLayout>
#include <QTimer>
#include <QWidget>

#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/ui.h"



class CSteerWidget : public QFrame 
{
  Q_OBJECT

public:
  explicit CSteerWidget(QWidget *parent = 0);
  ~CSteerWidget();

private:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;

public slots:  
  void refresh();


private:
  void  FrameSmooth(QWidget *parent);
  void  FrameNormal(QWidget *parent);

 private:
  Params params; 
  QLabel *icon_label;
  QPixmap  pix_plus;
  QPixmap  pix_minus;


  QVBoxLayout *main_layout;
  QPushButton *title_label;
  QHBoxLayout *hlayout;
  QLabel *description = nullptr;  


  QPushButton  *method_label;
  int    m_nSelect;
  int    m_bShow;

   QFrame *m_pChildFrame1;
   QFrame *m_pChildFrame2;
};

