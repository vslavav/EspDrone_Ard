#include "CmdHandler.h"
#include <vector>
//#include "MotorCtrl.h"
//#include "TopMgr.h"

using namespace std;

vector<string> split (string sLine, char cDelimiter);

void CmdHandler_SetNewCommand(string sCmd)
{
    vector<string> res = split(sCmd, ':') ;
    if(res.size() == 0)
    {
        return;
    }

    if(res[0].find("MotorCtrl") != string::npos)
    {
        //MotorCtrl_SetNewCommand(res[1]);
    }
    else if(res[0].find("Servo_1_Angle") != string::npos)
    {
        //ServoCtrl_Servo_1_Angle(res[1]);
    }
    else if(res[0].find("Servo_2_Angle") != string::npos)
    {
        //ServoCtrl_Servo_2_Angle(res[1]);
    }
    if(res[0].find("TopMgr") != string::npos)
    {
        //TopMgr_SetNewCommand(res[1]);
    }


}
