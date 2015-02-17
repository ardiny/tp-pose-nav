#include <algorithm>
#include <iostream>
int main(){
std::cout<<"f("<<std::endl;
system("gedit");
FILE* file = popen("ls", "r");
// use fscanf to read:
char buffer[100];
fscanf(file, "%s", buffer);
pclose(file);
return 0;
};
