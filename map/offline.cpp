#include <algorithm>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <vector>
#include "rec.h" 
#include "../argos/controllers/marXbot/Consts.h"
using namespace std;
struct Arena{
float lon;
float width;
float height;
float res;
}arena;
//float rec[20][4];
int coni2x(float i,float resolution){
    float dif=(i+1.0*arenalon/2.0);
    int res=(((int)(dif*100)) % ((int)(resolution*100)));
    if (res==0){
        return (int)((dif+0.0001)/resolution+1);
    }
    else{
        return (int)((dif+0.0001)/resolution+1);
    }
}
int conj2y(float j,float resolution){
    float dif=(1.0*arenawidth/2.0)+j;
    int res=(((int)(dif*100.0)) % ((int)(resolution*100)));
    if (res==0){
        return (int)((dif+0.0001)/resolution+1);
    }
    else{
        return (int)((dif+0.0001)/resolution+1.0);
    }
}
int rect(float rectb[4],float pointx,float pointy,float reso,float accra,int ratestep){
float x1,x2,y1,y2;
float recta[4];
//rectb[0] h
//rectb[1] w
//rectb[2] x
//rectb[3] y
//------
//---x
//---|
//y--|
recta[0]=-rectb[3]+rectb[1]/2;
recta[1]=-rectb[3]-rectb[1]/2;
recta[2]=rectb[2]+rectb[0]/2;
recta[3]=rectb[2]-rectb[0]/2;
    if (recta[0]>recta[1])
    {
    x1=recta[1];
    x2=recta[0];
    }
    else
    {
    x1=recta[0];
    x2=recta[1];
    };
    if (recta[2]>recta[3])
    {
    y1=recta[3];
    y2=recta[2];
    }
    else
    {
    y1=recta[2];
    y2=recta[3];
    };
//if ((x1<=pointx<=x2)&&(y1<=pointy<=y2))
//return true;
//pointx+=reso/2;
//pointy+=reso/2;
//cout<<pointx-accra<<" "<<pointx+accra<<" "<<pointy-accra<<" "<<pointy+accra<<endl;
//cout<<x1<<" "<<x2<<" "<<y1<<" "<<y2<<endl;
float accr=accra;
for(float accr=0;accr<=accra;accr+=accra/ratestep){
if ((x1<=(pointx+accr))&&(x2>=(pointx+accr))&&(y1<=(pointy+accr))&&(y2>=(pointy+accr)))
return true;
if ((x1<=(pointx-accr))&&(x2>=(pointx-accr))&&(y1<=(pointy+accr))&&(y2>=(pointy+accr)))
return true;
if ((x1<=(pointx-accr))&&(x2>=(pointx-accr))&&(y1<=(pointy-accr))&&(y2>=(pointy-accr)))
return true;
if ((x1<=(pointx+accr))&&(x2>=(pointx+accr))&&(y1<=(pointy-accr))&&(y2>=(pointy-accr)))
return true;
if ((x1>=(pointx-accr))&&(x1<=(pointx+accr))&&(y1>=(pointy-accr))&&(y1<=(pointy+accr)))
return true;
if ((x1>=(pointx-accr))&&(x1<=(pointx+accr))&&(y2>=(pointy-accr))&&(y2<=(pointy+accr)))
return true;
if ((x2>=(pointx-accr))&&(x2<=(pointx+accr))&&(y2>=(pointy-accr))&&(y2<=(pointy+accr)))
return true;
if ((x2>=(pointx-accr))&&(x2<=(pointx+accr))&&(y1>=(pointy-accr))&&(y1<=(pointy+accr)))
return true;
//false0,true1
}
return false;
};
int main(int argc, char *argv[]){


//iniate
arena.lon=arenalon;
arena.width=arenawidth;
arena.height=arenaheight;
arena.res=arenares;
float accra1(0.07);
int ratestep1(4);
if (argc>1){
arena.res=atof(argv[1]);
}
if (argc>2){
accra1=atof(argv[2]);
}
if (argc>3){
ratestep1=(int)(atof(argv[3]));
}
int count=0;
int stat=0;
string line,rr;
ofstream finalfile("offline.txt");
std::cout<<"-Running Argos, please wait..."<< std::endl;
for (float i=-1*arena.lon/2;i<=1*arena.lon/2;i=i+arena.res)
 for (float j=-1*arena.width/2;j<=1*arena.width/2;j=j+arena.res){
 ifstream infile ("../argos/xml/crude.txt");
 ofstream outfile ("../argos/xml/map.xml");
 ifstream footbotfile("footbot.txt");
 //ifstream outargosfile("outargos.txt");
// std::cout<<"Start"<<std::endl;
    if (infile.is_open())
    {
        if (outfile.is_open())
         {
                while ( infile.good() )
                {
                getline (infile,line);
            outfile << line << endl;
            }

        }
    outfile << "<foot-bot id=\"fb_0\" position=\""<<j<<","<<-i<<","<<0.0<<"\" orientation=\"0,0,0\" controller=\"fdc\" />" << endl;
    outfile << "</arena>" << endl;
    outfile << "</argos-configuration>" << endl;
    }
    else cout << "Unable to open file";
 infile.close();
 outfile.close();
system("launch_argos -c ../argos/xml/map.xml");
    if (footbotfile.is_open())
    {
                while ( footbotfile.good() )
                {
                getline (footbotfile,line);
            int pre=0;
            stat=3;
                  /* */         for(int k=0;k<numberfield;k++){

                    if (rect(rec[k],i,j,arena.res,accra1,ratestep1))
                    {
                    stat=rec[k][4];
                    pre++;

                    };

                };
            if (pre>1) stat=0;
                        if((i<=-1*arena.lon/2)||(i>=+1*arena.lon/2)||(j<=-1*arena.width/2)||(j>=+1*arena.width/2)) stat=1;
            finalfile <<count<<","<<coni2x(i,arena.res)<<","<<conj2y(j,arena.res)<<","<<0<<","<<stat<<","<<line<<endl;
            }

        }
count++;
 }
return 0;
};
