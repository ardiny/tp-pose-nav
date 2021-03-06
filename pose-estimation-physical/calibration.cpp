#include "calibration.h"

calibration::calibration():
/*sensor0LT { 3120, 3045, 2606, 2293, 1997, 1807, 1617, 1491, 1335, 1267, 1159, 1059, 1016, 947, 926, 858, 815, 745, 724, 676, 676, 674, 628, 605, 607, 561, 541, 561, 560, 533, 561, 536, 510, 589, 519, 517, 516, 518, 535, 518, 516 },
sensor1LT{ 3101, 3068, 2707, 2357, 2095, 1900, 1667, 1513, 1365, 1270, 1161, 1092, 1027, 938, 894, 839, 795, 749, 748, 681, 656, 635, 612, 590, 561, 588, 560, 536, 514, 539, 490, 515, 514, 489, 483, 490, 515, 490, 467, 466, 489 },
// 20 to 150 cm included + 0 to 20 cm in another table
sensor2LT { 2655, 2554, 2491, 2409, 2332, 2281, 2203, 2174, 2089, 2054, 1969, 1929, 1897, 1835, 1794, 1767, 1708, 1705, 1634, 1578, 1595, 1550, 1526, 1472, 1443, 1439, 1378, 1395, 1355, 1331, 1330, 1291, 1263, 1244, 1263, 1243, 1244, 1221, 1204, 1200, 1169, 1180, 1155, 1136, 1155, 1112, 1067, 1108, 1069, 1043, 1030, 1020, 1019, 1023, 1021, 979, 998, 978, 956, 975, 934, 953, 912, 932, 886, 889, 909, 863, 910, 879, 889, 836, 817, 859, 859, 815, 790, 794, 813, 797, 792, 793, 772, 790, 772, 744, 750, 745, 735, 725, 768, 703, 744, 724, 723, 700, 725, 700, 680, 678, 690, 679, 721, 702, 700, 699, 655, 677, 680, 676, 657, 676, 670, 655, 676, 657, 653, 632, 654, 654, 633, 654, 611, 632, 631, 631, 607, 610, 628, 610, 629 },
sensor2NearLT  { 1838, 1948, 1990, 2157, 2282, 2534, 2779, 2971, 3040, 3087, 3082, 3086, 3072, 3040, 3021, 2935, 2873, 2777, 2721, 2655 },
sensor3LT { 2634, 2513, 2471, 2404, 2286, 2204, 2115, 2114, 2010, 1989, 1901, 1838, 1817, 1728, 1726, 1677, 1661, 1590, 1531, 1510, 1486, 1462, 1441, 1375, 1366, 1357, 1312, 1290, 1288, 1282, 1257, 1231, 1174, 1195, 1148, 1197, 1147, 1102, 1084, 1060, 1039, 1038, 1057, 1036, 1035, 973, 971, 950, 970, 968, 947, 899, 876, 901, 920, 873, 873, 872, 840, 812, 809, 854, 806, 807, 805, 847, 809, 741, 739, 806, 784, 738, 719, 740, 696, 766, 737, 670, 704, 696, 695, 651, 721, 693, 692, 649, 701, 626, 626, 669, 670, 647, 626, 649, 603, 603, 623, 648, 623, 605, 603, 553, 601, 601, 601, 547, 626, 529, 527, 601, 648, 578, 549, 581, 551, 527, 578, 549, 521, 551, 550, 551, 500, 605, 541, 509, 436, 479, 481, 452, 499 },
sensor3NearLT{ 1941, 1966, 2015, 2119, 2371, 2594, 2818, 3056, 3079, 3086, 3087, 3084, 3084, 3069, 3019, 2956, 2860, 2744, 2678, 2634 }
{

 }*/
sensor0LT { 3044.91,2971.94,2535.05,2276.61,1974.5,1719.11,1521.82,1384.48,1254.67,1276.17,1060.98,983.37,936.87,900.9,868.66,845.28,1029.27,827.74,812.44,1114.61,822.06,818.12,1007.87,814.04,980.99,816.61,466.13,405.34,459.15,376.62,442.86,358.75,471.19,470.05 },
sensor1LT{ 3066.24,3019.51,2570.81,2211.32,1958.25,1707.42,1498.39,1352.74,1217.78,1100.71,1006.31,926.38,856.84,808.06,764.18,747.56,718.45,708.62,692.66,698.2,700.68,694.76,689.79,692.75,693.73,694.89,301.69,284.49,275.94,237.43,229.09,215.96,204.37,191.45},
// 20 to 150 cm included + 0 to 20 cm in another table
sensor2LT { 2460.27,2390.28,2306.58,2232.78,2162.17,2094.46,2050.4,1986.51,1939.46,1838.8,1781.92,1744.59,1696.54,1656.65,1613,1592.22,1546.47,1516.65,1480.51,1437.94,1407.98,1387.31,1348.71,1325.79,1305.8,1278.17,1250.98,1228.67,1191.96,1188.89,1164.61,1150,1139.56,1123.13,1112.38,1101.64,1093.48,1086.04,1080.61,1076.26,1056.34,1045.31,1028.62,1023.37,1010.68,991.84,980,973.53,962.72,956.45,941.33,936,922.29,920.15,913.68,899.72,893.11,880.56,878.8,858.59,852.03,842.01,832.95,837.99,827.06,819.85,812.88,799.94,797.87,786.79,777.08,776.18,770.5,759.43,755.13,752.53,739.42,732.04,731.4,721.34,715.97,709.84,709.71,700.36,694.4,688,674,660.18,680.23,673,670,666.04,661.65,655,649.47,647.93,644.06,637.45,636.46,632.4 },
sensor2NearLT  { 1730.81,1718.96,1860.48,2046.59,2232.87,2494.24,2763.49,2956.73,3058.2,3082.29,3083.71,3077,3060.07,3012.79,2947.42,2878.39,2803.61,2711.86,2634.84,2555.54},
sensor3LT { 2448.11,2368.92,2285.09,2206.4,2142.62,2076.87,2024.22,1989.74,1918.68,1821.12,1781.95,1732.34,1695.68,1657.8,1592.48,1588.35,1545.47,1499.25,1492.23,1428.43,1403.68,1381.72,1340.84,1326.61,1295.23,1270.2,1243.35,1220.5,1218.08,1207.39,1183.49,1168.34,1141.75,1116.16,1100.53,1094.02,1084.95,1075.62,1065.87,1062.01,1043.65,1030.56,1030.1,1014.31,1010.43,1000,991.31,963.17,976.73,967.62,944.95,936.15,925.84,919.75,914.23,909,902.02,896.29,889.2,865,860,858.84,858.83,855.1,837.25,833.83,820.56,813.89,804.91,786.2,780,776.23,769.64,768.65,761.67,760.3,753.63,740.21,737.9,734.15,730.38,727.82,721.15,714.09,717.2,709.78,705.19,696.42,696.52,695.61,692.12,691.05,688.46,679.94,678.82,670.02,660.62,660.81,658,656.58 },
sensor3NearLT{ 1733.34,1721.86,1823.39,1991.13,2187.43,2419.31,2675.81,2879.36,3014.77,3071.55,3072.89,3065.39,3045.51,3003.12,2948.73,2880.18,2799.1,2694.65,2610.12,2530.01}
{

 }


void calibration::tets(){
   // qDebug() <<sensor0LT[3];
}
float calibration::getPos(const int* table, const int tableLength, const int value)
{
    for (int i = 0; i < tableLength; i++)
    {
        if (value > table[i])
        {
            if (i == 0)
                return 0.f;

            float delta = (float)(value - table[i]) / (float)(table[i-1] - table[i]);
            return (float)i + 1.f - delta;
        }
    }
    return (float)tableLength;
}
float calibration::getNearest(float* tab, int angle) {
    // Maximum +-3° lookup
    for(int i = 0; i <= 3; i++) {
        int r,l;

        r = (angle + i) % 360;
        l = (angle - i + 360) % 360;
        if (tab[r] != -1)
            return tab[r];
        if (tab[l] != -1)
            return tab[l];
    }
    return -1;
}
float calibration::recalculateDist(int angle)
{
    float far;
    float near;
    float farraw;
    float nearraw;
    //assert(angle >= 0);
   // assert(angle < 360);

    // Look for the closest angle which has the missing value
    far = scannerDataFar[angle];
    near = scannerDataClose[angle];
    farraw = scannerDataFarRaw[angle];
    nearraw = scannerDataCloseRaw[angle];
    if(far == -1 && near == -1)
        // Tell that we have a missing point
        return -1;
    if(far == -1){
        far = getNearest(scannerDataFar, angle);
        farraw = getNearest(scannerDataFarRaw, angle);}
    if(near == -1){
        near = getNearest(scannerDataClose, angle);
        nearraw = getNearest(scannerDataCloseRaw, angle);
    }

    // Check if we found it
    if(near == -1 || far == -1)
        return -1;
    if ((nearraw>600)&&(farraw>1500))
        if(near > 0)
            return near+8.5-7;//8.5 diameter of robot and -7 because of simulation
        else
            return -1;

    else
        if(far > 0)
            return far + 20 ;
        else
            return -1;


}
double calibration::getDist(int angle){

                if (parameters.calibration_state==parameters.just_robot_distance)
                            return dist_robot[angle];
                if (parameters.calibration_state==parameters.just_recalculate_distance)
                           return recalculateDist(angle);//always parameters.updatestate=parameters.update
                if (parameters.calibration_state==parameters.both){
                    if (dist_robot[angle]<=0)
                        return recalculateDist(angle);
                    else if (dist_robot[angle]>80)
                        return recalculateDist(angle);
                    else
                          return dist_robot[angle];
               }
    }
double calibration::filter1(double distvalue,int angle){
    if(parameters.histangle[angle]>0){
        parameters.histweight[angle]++;
        return (distvalue+parameters.histangle[angle]*parameters.histweight[angle])/(parameters.histweight[angle]+1);
    }else
        return distvalue;
}
