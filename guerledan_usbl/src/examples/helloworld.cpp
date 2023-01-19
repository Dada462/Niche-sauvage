#include <iostream>
#include <ctime>
#include <fstream>
#include <chrono>
#include <thread>
#include <string>

using namespace std;
std::string current_time()
{
   time_t now = time(0);
   
   // convert now to string form
   char* dt = ctime(&now);

   cout << "The local date and time is: " << dt << endl;

   // convert now to tm struct for UTC
//    tm *gmtm = gmtime(&now);
//    dt = asctime(gmtm);
   char* test= new char[8];
   for (int i=0;i<8;i++)
   {
      test[i]=dt[i+11];
   }
   std::string s(test,8);
   return s;
}
#include <time.h>
int main() 
{
std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
begin = std::chrono::steady_clock::now();

std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "[ns]" << std::endl;
   return 0;
}