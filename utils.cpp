#include <vector>
#include <string>

using namespace std;

vector<string> split (string sLine, char cDelimiter)
{
    vector<string> res;
    string sTemp = "";
    for(int ii = 0; ii < sLine.length(); ii++)
    {
        if(sLine[ii] != cDelimiter)
        {
            sTemp += sLine[ii];
            
        }
        else
        {
            res.push_back(sTemp);
            sTemp = "";
        }
    }

    if(sTemp.size() != 0)
    {
        res.push_back(sTemp);
    }

    return res;
}