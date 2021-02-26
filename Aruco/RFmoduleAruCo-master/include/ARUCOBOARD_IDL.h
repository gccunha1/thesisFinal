// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_ARUCOBOARD_IDL
#define YARP_THRIFT_GENERATOR_ARUCOBOARD_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class ARUCOBOARD_IDL;


/**
 * struct Bottle { }
 * (
 * yarp.name = "yarp::os::Bottle"
 * yarp.includefile="yarp/os/Bottle.h"
 * )
 */
class ARUCOBOARD_IDL : public yarp::os::Wire {
public:
  ARUCOBOARD_IDL();
  /**
   * Update the state text file.
   * @return true/false on success/failure
   */
  virtual bool update();
  /**
   * Quit the module.
   * @return true/false on success/failure
   */
  virtual bool quit();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
