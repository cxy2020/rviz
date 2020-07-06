#ifndef SEND_EXE_PATHTASK_H
#define SEND_EXE_PATHTASK_H

#include "generate_clean_path_tool.h"

namespace rviz {
class SendExePathTaskTool : public GenerateCleanPathTool
{
Q_OBJECT
public:
  SendExePathTaskTool() {}

  virtual void onInitialize();

  virtual void deactivate();
};
}

#endif // SEND_EXE_PATHTASK_H
