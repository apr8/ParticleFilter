/*

Contains the required attributes to read a log file.
args: log file to read
returns : the line in the log file whenever quried

*/

#ifndef PARTICLE_FILTER_READ_LOG_DATA_
#define PARTICLE_FILTER_READ_LOG_DATA_

#include <fstream>

using namespace std;

class ReadLogFile {
  
  public:
    ReadLogFile (ifstream file);
    virtual ~ReadLogFile();
    int readLine(); 

    // sensor data
    LaserData l_read;
    OdomData o_read;
    enum LOG_TYPE {
    L = 0,
    O = 1
    };
 
  private:
    ifstream in_log;
};


#endif

