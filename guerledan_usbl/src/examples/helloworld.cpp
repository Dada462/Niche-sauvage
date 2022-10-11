#include <iostream>
#include <ctime>
#include <fstream>

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

int main() {
   ofstream log;
   log.open("src/guerledan_usbl/logs/October_test_"+current_time()+".dat");
   log<<"LOG: northing, easting, depth, azimith, elevation, range, Local depth"<<endl;
   log<<"BLABLA"<<endl;
   log.close();
}