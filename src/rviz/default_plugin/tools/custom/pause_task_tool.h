#ifndef PAUSE_TASK_H
#define PAUSE_TASK_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <QObject>
# include "rviz/tool.h"
#endif

namespace rviz {
class PauseTaskTool : public Tool
{
Q_OBJECT
public:
  virtual void onInitialize();

  virtual void activate();

  virtual void deactivate() {}
};
}

#endif // PAUSE_TASK_H
