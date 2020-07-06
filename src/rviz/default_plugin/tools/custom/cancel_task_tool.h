#ifndef CANCEL_TASK_TOOL_H
#define CANCEL_TASK_TOOL_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <QObject>
# include "rviz/tool.h"
#endif

namespace rviz {
class CancelTaskTool : public Tool
{
Q_OBJECT
public:
  virtual void onInitialize();

  virtual void activate();

  virtual void deactivate() {}
};
}

#endif // CANCEL_TASK_TOOL_H
