#include <iostream>
#include <string>
#include "rm.h"

using namespace std;

int main(int argc,char *argv[])
{
    string str = argv[1];
    int times = atoi(argv[2]);
    int maxtime= atoi(argv[3]);
    int cnt=0;
    while(cnt<times)
    {
        rm a(str,maxtime);
        a.exe();
        ++cnt;
    }
    return 0;
}
