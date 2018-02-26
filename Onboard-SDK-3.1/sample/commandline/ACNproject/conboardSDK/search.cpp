#include "search.h"
#define _PI_ 3.14159265358979323846
double latitude(const Flight *flight)
{
    return flight->getPosition().latitude;
}
double longitude(const Flight *flight)
{
    //cout << "YOYO" <<  setprecision(20) << flight->getPosition().longitude << endl;
    return flight->getPosition().longitude;
}
double altitude(const Flight *flight)
{
    return flight->getPosition().altitude;
}
double flight_height(const Flight *flight)
{
    return flight->getPosition().height;
}
double getYaw(const Flight *flight)
{
    QuaternionData q = flight->getQuaternion();
    float yaw=atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2), - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1));
    return yaw;
}
void control(VirtualRC *vrc,int pitch,int yaw)
{
    VirtualRCData vdata;
    vdata.yaw = 1024+yaw;
    vdata.throttle = 1024;
    vdata.pitch = 1024+ pitch;
    vdata.roll = 1024; //+ roll;
    vrc->sendData(vdata);
}
void takeOff(VirtualRC *vrc)
{
    VirtualRCData vdata;
    vdata.roll = 1024-660;
    vdata.pitch = 1024-660;
    vdata.throttle = 1024-660;
    vdata.yaw = 1024+660;
    vrc->sendData(vdata);
    sleep(5);

    VirtualRCData vdata2;
    vdata2.throttle = 1024+330;
    vrc->sendData(vdata2);
    sleep(5);
}

void *collectRSSI(void *ptr)
{
    CollectThreadParams *params = (CollectThreadParams*)ptr;
    vector<PointData> *record =  params->record;
    time_t startTime = time(0);

    while(params->isFlying)
    {
        usleep(100*1000);
        PointData p;
        p.latitude = latitude(params->flight);
        p.longitude = longitude(params->flight);
        p.altitude = altitude(params->flight);
        p.ctimeStamp = clock();
        struct filtering_result
        {
            double result;
        } filter;


        
        double collect[1000];
        int i=0;
        while(i<1000)
        {
            //collect[i] = getFakeRSSI(params->flight,0.43648972424, 2.12120737396, 0, p.startSearch); //Hard Cord
            collect[i] = getFakeRSSI(params->flight,0.43648975113, 2.12119927893, 0, p.startSearch); //Hard Cord
            
            //collect[i] = getFakeRSSI(params->flight,0.393449,1.988961, 0, p.startSearch); //Hard Cord
            i++;
        }
        //cout << kalman_filter(collect) <<endl;
        p.RSSI = kalman_filter(collect);

        record->push_back(p);
        return 0;
    }

/*
 
        iwrange range;
        int sock;
        wireless_info info;
        sock = iw_sockets_open();
        double collect[50];
        double fCollect[50];


        if(iw_get_range_info(sock,"wlan0",&range)<0){
            printf("Error\n");
            exit(2);
        }

        int i=0;
        int count=0;
        while(i<1000){
            iw_get_stats(sock,"wlan0",&(info.stats),&range, 1);
            int r = (int8_t)info.stats.qual.level;
            if(i%20==0){
                fCollect[count] = (r/2) - 100;
                //printf("%d\n",(int8_t)info.stats.qual.level);
                //printf("%lf\n",fCollect[count]);
                count++;
            }
            i++;
        }

        p.RSSI = kalman_filter(fCollect);

        record->push_back(p);
        return 0;
    }

*/

}

vector<PointData> planPath(CoreAPI *api)
{

    /////////////////////
    // extern variable //
    /////////////////////

    int turnCase = 5;
    int preturn = 1;
    int descision = 1;  //0:left, 1:mid, 2:right
    int bool_predecision = 0;  //Previous decision is correct or not
    double distPercent=1.0;
    int predictCount = 0;



    const Flight *flight = new Flight(api);
    double curYaw = 0.0;
    double startX = 0.0, startY=0.0;  //Starting Point
    double currX = 0.0, currY=0.0, preX=0.0, preY=0.0;  //X and Y, in XY coordinate system
    double currLat=0.0, currLon=0.0, preLat=latitude(flight), preLon=longitude(flight); //Latitude and Longitude, in Geographic coordinate system
    double predictLon=0.0, predictLat=0.0, predictX=0.0, predictY=0.0;  //guess Target position, need to transform
    double Xa=0.0, Ya=0.0, Xb=0.0, Yb=0.0, Xt=0.0, Yt=0.0, LatA=0.0, LonA=0.0, LatB=0.0, LonB=0.0; //For CoordinateChanger
    double tol_moveDist = 0.0, err_dist=0.0;
    double moveDistance =0.0, cur_radius=0.0, pre_radius=0.0;
    
    vector<double> vec_predictX;  //predictX vector
    vector<double> vec_predictY;  //predictY vector
    
    
    vector<double> r_matrix;  //rotation matrix
    
    vector<double> x_matrix;  //history x coordinate
    vector<double> y_matrix;  //history y coordinate
    vector<double> lat_matrix, lon_matrix;  //history lat, lon coordinate

    // Start Searching, first fly straight for a little while
    vector<PointData> record;  //main record, recording all record from searchRecord
    vector<PointData> searchRecord = goFind(api,"./preFlight.txt", distPercent); // each turn record
    GuessPosition guess;


    currLat = searchRecord[searchRecord.size()-1].latitude;
    currLon = searchRecord[searchRecord.size()-1].longitude;
    /*
    moveDistance = earth_distance(currLat, currLon, preLat, preLon, 'K') * 1000; //km to m
    flightMove(&currX, &currY, &preX, &preY, descision, moveDistance, curYaw);
    tol_moveDist += moveDistance;
    */

    cur_radius = rssiToDist(searchRecord[searchRecord.size()-1].RSSI, searchRecord[searchRecord.size()-1].altitude);
    pre_radius = 999.0;

    currX=out4in5(cur_radius*1.5); currY=0.0; preX=currX; preY=-10.0; startX=currX; startY=currY;

    // 2D Array malloc, Create a weight distribution map(WDM) //
    printf("Height:%10lf\n", currX);
    int height = currX;  //Hard Cord
    int width = height*2;

    // 生成一維指標陣列
    int count=0;
    int i=0, j=0;
    double **map_weight = new double*[width];
    int **map_count = new int*[width];

    // 每個指標陣列再生成整數陣列
    for(i=0; i<width; i++)
    {
        map_count[i] = new int[height];
        map_weight[i] = new double[height];
    }
    // write
    for(i=0; i<width; i++)
    {
        for(j=0; j<height; j++)
        {
            map_count[i][j] = 0;
            map_weight[i][j] = 0.0;
        }
    }

    /////////////////////
    // extern variable //
    /////////////////////

    //Add First Weight//
    double d=0.0;
    for(i=0; i<width; i++)
    {
        for(j=0; j<height; j++)
        {
            d = distance(i, j, currX, currY);
            if (cur_radius >= d)
            {
                if (d <= cur_radius)
                {
                    map_weight[i][j] += d / pow((cur_radius), 2);
                    map_count[i][j] += 1;
                }
                else
                {
                    map_weight[i][j] += (cur_radius * 2 - d) / pow((cur_radius), 2);
                    map_count[i][j] += 1;
                }
            }
        }
    }


    record.insert( record.end(), searchRecord.begin(), searchRecord.end() );


    //Get current Yaw
    curYaw = getYaw(flight);
    curYaw = curYaw * -1;
    if(curYaw<0)
    {
        curYaw = 2*_PI_ + curYaw;
    }
    r_matrix.clear();
    rotation_matrix(currX, currY, preX, preY, r_matrix, curYaw);  //r_matrix在function宣告時將帶入address

    //Start Dynamic Path Planning(DPP)
    double recount_threshode = height/1.5;
    do
    {
        count+=1;

        guess = predictPos(map_weight, map_count, cur_radius, currX, currY, height);  //得出最高權重的(X,Y)點

        cout << "guessX:" << guess.xt << endl;
        cout << "guessY:" << guess.yt << endl;
        if(guess.xt == guess.xt){
            vec_predictX.push_back(guess.xt);
            vec_predictY.push_back(guess.yt);
        }

        if(vec_predictX.size()>=10 && vec_predictY.size()>=10)  //Hard Cord
        {
            if(vec_predictX.size()==11){
                vec_predictX.erase(vec_predictX.begin());
            }
            if(vec_predictY.size()==11){
                vec_predictY.erase(vec_predictY.begin());
            }
            predictX = gaussFilter(vec_predictX);
            predictY = gaussFilter(vec_predictY);
            //coordinateChanger(predictX, predictY, x_matrix, y_matrix, lat_matrix, lon_matrix);  //坐標系轉換
            cout << "predictX:" << predictX << endl;
            cout << "predictY:" << predictY << endl;
            

            predictLon = leastSquare(predictX, x_matrix, lon_matrix);
            predictLat = leastSquare(predictY, y_matrix, lat_matrix);
            err_dist = earth_distance(0.43648975113, 2.12119927893, predictLat, predictLon, 'K') * 1000;
            
            record[record.size()-1].error_dist = err_dist;
            record[record.size()-1].guessLatitude = predictLat;
            record[record.size()-1].guessLongitude = predictLon;
            
            printf("predictLat:%.10lf, predictLon:%.10lf, err_dist:%.10lf\n ", predictLat, predictLon, err_dist);
            x_matrix.erase(x_matrix.begin());
            y_matrix.erase(y_matrix.begin());
            lon_matrix.erase(lon_matrix.begin());
            lat_matrix.erase(lat_matrix.begin());
            
        }

        

        descision = turnDecision(startX, startY, currX, currY, preX, preY, &preturn, &bool_predecision, &turnCase, cur_radius, pre_radius, map_weight, map_count, r_matrix, height);

        //////////////////////////////////
        ///// Next out4in5 of Flight /////
        //////////////////////////////////
        distPercent = cur_radius/(height/1.6);
        if(distPercent>1 || distPercent<0)
        {
            distPercent = 1;
        }

        searchRecord.clear();
        if(descision==0)
        {
            searchRecord = goFind(api,"./moveLeft.txt", distPercent);
        }
        else if(descision==1)
        {
            searchRecord = goFind(api,"./moveStraight.txt", distPercent);
        }
        else if(descision==2)
        {
            searchRecord = goFind(api,"./moveRight.txt", distPercent);
        }

        //Get current Yaw
        curYaw = getYaw(flight);
        curYaw = curYaw * -1;
        if(curYaw<0)
        {
            curYaw = 2*_PI_ + curYaw;
        }
        flightMove(&currX, &currY, &preX, &preY, descision, moveDistance, curYaw);
        ////////////////////////////////

        pre_radius = cur_radius;
        cur_radius = rssiToDist(searchRecord[floor(searchRecord.size()/2)].RSSI, searchRecord[floor(searchRecord.size()/2)].altitude);

        r_matrix.clear();
        rotation_matrix(currX, currY, preX, preY, r_matrix, curYaw);  //r_matrix在function宣告時將帶入address
        //map_weight, map_count將導入address
        addWeight(startX, startY, currX, currY, preX, preY, cur_radius, pre_radius, map_weight, map_count, r_matrix, height);


        //若接近到原始距離的一半，權重重新分配
        
        if(cur_radius < (recount_threshode/3))   //Hard Cord
        {
            for(i=0; i<width; i++)
            {
                for(j=0; j<height; j++)
                {
                    map_count[i][j] = map_count[i][j]/2;
                }
            }
            recount_threshode = cur_radius;
        }
        
        

        preLat = currLat;
        preLon = currLon;
        currLat = searchRecord[floor(searchRecord.size()/2)].latitude;
        currLon = searchRecord[floor(searchRecord.size()/2)].longitude;
        x_matrix.push_back(preX);
        y_matrix.push_back(preY);
        lat_matrix.push_back(preLat);
        lon_matrix.push_back(preLon);

        moveDistance = earth_distance(currLat, currLon, preLat, preLon, 'K') * 1000;
        tol_moveDist += moveDistance;
        searchRecord[searchRecord.size()-1].total_moveDist = tol_moveDist;

        record.insert( record.end(), searchRecord.begin(), searchRecord.end() );

        //printf("moveDistance: %lf\n", moveDistance);
        printf("CurR:%.10lf, PreR:%.10lf\n", cur_radius, pre_radius);
        printf("CurX:%.10lf, CurY:%.10lf\n", currX, currY);
        printf("CurLon:%.10lf, CurLat:%.10lf\n", currLon, currLat);
        //printf("preX:%.10lf, preY:%.10lf\n", preX, preY);
        //printf("preLon:%.10lf, preLat:%.10lf\n", preLon, preLat);

        cout<< "------------------------------------\n" <<endl;

    }
    while(distPercent>0.35);  //Hard Cord


    for(int i=0; i<record.size(); i++)
    {
        record[i].ctimeStamp = (record[i].ctimeStamp - record[0].ctimeStamp) / CLOCKS_PER_SEC;
    }

    return record;
}

vector<PointData> goFind(CoreAPI *api,const char *pathFile, double distPercent)
{

    VirtualRC vrc(api);
    vrc.setControl(true,VirtualRC::CutOff::CutOff_ToRealRC);


    FILE * fp = fopen(pathFile,"r");
    int pitch=0, roll=0, yaw=0;

    vector<PointData> record;

    pthread_t collectThr;
    CollectThreadParams params;
    params.record = &record;
    params.flight = new Flight(api);


    /* ---- start flight ---- */
    pthread_create(&collectThr,NULL,collectRSSI,&params);
    fscanf(fp,"%d %d",&pitch,&yaw);

    do
    {
        control(&vrc, pitch*distPercent, yaw);  //Pitch Hard Code
        usleep(100 * 1000);
        fscanf(fp,"%d %d",&pitch,&yaw);

    }
    while(!feof(fp));

    params.isFlying = false;
    pthread_join(collectThr,NULL);
    cout<< "Record point: " << record.size() <<endl;


    return record;
}

double getFakeRSSI(const Flight *flight,double la,double lo,double al, int startSearch)
{
    double lat = latitude(flight);
    double lon = longitude(flight);
    double alt = altitude(flight);
    double dist = 1000 * earth_distance(la,lo,lat,lon,'K'); //meters
    dist = sqrt((dist*dist)+(alt-al)*(alt-al));
    double n=2, A= -50; //n:path-loss exponent, A:RSSI per unit
    double noise = 0.0;
    if(dist>=30)
    {
        noise = 2 * normalDistribution();
    }
    else if(30>dist>=20)
    {
        noise = 1.5 * normalDistribution();
    }
    else if(20>dist>=15)
    {
        noise = 1 * normalDistribution();
    }
    else if(15>dist>=10)
    {
        noise = 0.7 * normalDistribution();
    }
    else if(10>dist>=5)
    {
        noise = 0.4 * normalDistribution();
    }
    else if(5>dist)
    {
        noise = 0.1*normalDistribution();
    }
    double rssi = A - 10*n*log10(dist) + noise;
    /*
    if(dist>300 && startSearch==0){
    	rssi = 1;
    }
    */
    return rssi;
}




double deg2rad(double deg)
{
    return (deg * _PI_ / 180);
}
double rad2deg(double rad)
{
    return (rad * 180 / _PI_);
}

double earth_distance(double lat1, double lon1, double lat2, double lon2, char unit)
{
    double theta, dist;
    theta = lon1 - lon2;
    dist = sin(lat1) * sin(lat2) + cos(lat1) * cos(lat2) * cos(theta);
    dist = acos(dist);
    dist = rad2deg(dist);
    dist = dist * 60 * 1.1515;
    dist = dist * 1.609344;
    return (dist);
}

void coordinateChanger(double xt, double yt, vector<double> x_matrix, vector<double> y_matrix, vector<double> lat_matrix, vector<double> lon_matrix)   //XY coordinate to Lon,Lat coordinate
{

    



}

double rssiToDist(double rssi, double altitude)
{
    double dist=0;
    if(rssi<0)
    {
        double n=2, A=-50; //n:path-loss exponent, A:RSSI per unit
        double log=10, exp= (A-rssi) / (10*n);
        dist = pow(log, exp);
        dist = sqrt(pow(dist,n) + pow(altitude, n));
        return dist;
    }
    else
    {
        double dist=-1;
        return dist;
    }
}

double normalDistribution()
{
    srand(time(NULL));
    double u = rand() / (double)RAND_MAX;
    double v = rand() / (double)RAND_MAX;
    double pn = (rand() % 1);
    if(pn==0){
        pn = -1.0;
    }
    double x = sqrt(-2 * log(u)) * cos (2 * _PI_ * v);
    return (x*pn);

}


/**

Dynamic Path Planning Function

**/


void rotation_matrix(double currX, double currY, double preX, double preY, vector<double> &r_matrix, double curYaw)
{

    r_matrix.push_back(cos(curYaw));
    r_matrix.push_back(sin(curYaw));
    r_matrix.push_back(-sin(curYaw));
    r_matrix.push_back(cos(curYaw));

}

double moveDistance_to_speed(double move_distance)
{
    double speed;
    if(move_distance>=5)
    {
        speed = 5;
    }
    else
    {
        speed = move_distance;
    }

    return speed;
}

double distance(int x1, int y1, int x2, int y2)
{
    double d = pow(pow((x1 - x2), 2) + pow((y1 - y2), 2), 0.5);
    return d;
}

void addWeight(double startX, double startY, double currX, double currY, double preX, double preY, double cur_radius, double pre_radius, double** map_weight, int** map_count, vector<double> r_matrix, int height)
{

    int weight_i=0;
    int weight_j=0;


    if (cur_radius - pre_radius > 0)    //飛機遠離目標
    {
        for (int i = currX - cur_radius - 5; i <= currX + cur_radius + 5; i++)    //Hard Cord
        {
            for (int j = currY; j >= currY - cur_radius - 5; j--)
            {
                weight_i = out4in5((i - currX) * r_matrix[0] + (j - currY) * r_matrix[1] + currX);
                weight_j = out4in5((i - currX) * r_matrix[2] + (j - currY) * r_matrix[3] + currY);


                if (startX+height-1 > weight_i && weight_i>=0 && startY < weight_j && weight_j<startY+height-1)
                {
                    if (map_weight[weight_i][weight_j] >= 0)
                    {
                        double dist = distance(weight_i, weight_j, currX, currY);
                        if(cur_radius >= dist)
                        {
                            map_weight[weight_i][weight_j] += dist / pow((cur_radius), 2);
                            map_count[weight_i][weight_j] += 1;
                        }
                        else
                        {
                            map_weight[weight_i][weight_j] += (cur_radius * 2 - dist) / pow((cur_radius), 2);
                            map_count[weight_i][weight_j] += 1;
                        }
                    }
                }
            }
            for (int j = currY; j <= currY + cur_radius+5; j++)    //Hard Cord
            {
                weight_i = out4in5((i - currX) * r_matrix[0] + (j - currY) * r_matrix[1] + currX);
                weight_j = out4in5((i - currX) * r_matrix[2] + (j - currY) * r_matrix[3] + currY);


                if (startX+height-1 > weight_i && weight_i>=0 && startY < weight_j && weight_j<startY+height-1)
                {
                    if (map_weight[weight_i][weight_j] >= 0)
                    {
                        double dist = distance(weight_i, weight_j, currX, currY);
                        if(cur_radius >= dist)
                        {
                            map_weight[weight_i][weight_j] += dist / pow((cur_radius), 2)*0.9;
                            map_count[weight_i][weight_j] += 1;
                        }
                        else
                        {
                            map_weight[weight_i][weight_j] += (cur_radius * 2 - dist) / pow((cur_radius), 2)*0.9;
                            map_count[weight_i][weight_j] += 1;
                        }
                    }
                }
            }
        }
    }
    else if (cur_radius - pre_radius < 0)     //飛機靠近目標
    {
        for (int i = currX - cur_radius - 5; i <= currX + cur_radius + 5; i++)   //Hard Cord
        {
            for (int j = currY; j <= currY + cur_radius + 5; j++)
            {
                weight_i = out4in5((i - currX) * r_matrix[0] + (j - currY) * r_matrix[1] + currX);
                weight_j = out4in5((i - currX) * r_matrix[2] + (j - currY) * r_matrix[3] + currY);

                if (startX+height-1 > weight_i && weight_i>=0 && startY < weight_j && weight_j<startY+height-1)
                {
                    if (map_weight[weight_i][weight_j] >= 0)
                    {
                        double dist = distance(weight_i, weight_j, currX, currY);
                        if(cur_radius >= dist)
                        {
                            map_weight[weight_i][weight_j] += dist / pow((cur_radius), 2);
                            map_count[weight_i][weight_j] += 1;
                        }
                        else
                        {
                            map_weight[weight_i][weight_j] += (cur_radius * 2 - dist) / pow((cur_radius), 2);
                            map_count[weight_i][weight_j] += 1;
                        }
                    }
                }
            }
            for (int j = currY; j >= currY - cur_radius-5; j--)   //Hard Cord
            {
                weight_i = out4in5((i - currX) * r_matrix[0] + (j - currY) * r_matrix[1] + currX);
                weight_j = out4in5((i - currX) * r_matrix[2] + (j - currY) * r_matrix[3] + currY);

                if (startX+height-1 > weight_i && weight_i>=0 && startY < weight_j && weight_j<startY+height-1)
                {
                    if (map_weight[weight_i][weight_j] >= 0)
                    {
                        double dist = distance(weight_i, weight_j, currX, currY);
                        if(cur_radius >= dist)
                        {
                            map_weight[weight_i][weight_j] += dist / pow((cur_radius), 2)*0.9;
                            map_count[weight_i][weight_j] += 1;
                        }
                        else
                        {
                            map_weight[weight_i][weight_j] += (cur_radius * 2 - dist) / pow((cur_radius), 2)*0.9;
                            map_count[weight_i][weight_j] += 1;
                        }
                    }
                }
            }
        }
    }
}

double calConstant(double x, double y, double m)
{

    double b = y - (m * x);
    return b;
}

int turnDecision(double startX, double startY, double currX, double currY, double preX, double preY, int* preturn, int* bool_predecision, int* turnCase, double cur_radius, double pre_radius, double** map_weight, int** map_count, vector<double> r_matrix, int height)
{
    double turn_matrix[3] = {0.0, 0.0, 0.0};
    
    double slope=0.0;
    double split_line1=0.0;
    double split_line2=0.0;
    
    split_line1 = (1 / sqrt(3));
    split_line2 = (-1 / sqrt(3));
    double constant1 = calConstant(currX, currY, split_line1);
    double constant2 = calConstant(currX, currY, split_line2);
    
    int deltaX = currX - preX;
    int deltaY = currY - preY;
    if (deltaX > 5 && deltaY > 5)
    {
        *turnCase = 1;
    }
    else if (deltaX < -5 && deltaY > 5)
    {
        *turnCase = 2;
    }
    else if (deltaX < -5 && deltaY < -5)
    {
        *turnCase = 3;
    }
    else if (deltaX > 5 && deltaY < -5)
    {
        *turnCase = 4;
    }
    else if (5 >= deltaX >= -5 && deltaY >= 5)
    {
        *turnCase = 5;
    }
    else if (deltaX < -5 && -5 <= deltaY <= 5)
    {
        *turnCase = 6;
    }
    else if (-5 <= deltaX <= 5 && deltaY < -5)
    {
        *turnCase = 7;
    }
    else if (deltaX > 5 && -5 <= deltaY <= 5)
    {
        *turnCase = 8;
    }
    
    int wi=0;
    int wj=0;
    
    for (int i = currX - cur_radius - 5; i <= currX + cur_radius + 5; i++)
    {
        
        
        for (int j = currY; j <= currY + cur_radius + 5; j++)
        {
            
            wi = out4in5((i - currX) * (r_matrix[0]) + (j - currY) * (-r_matrix[1]) + currX);
            wj = out4in5((i - currX) * (-r_matrix[2]) + (j - currY) * (r_matrix[3]) + currY);
            
            if (startX+height-1 > wi && wi>=0 && startY <= wj && wj<startY+height-1)
            {
                if (map_weight[wi][wj] > 0)
                {
                    if (pow(wi - currX, 2) + pow(wj - currY, 2) <= cur_radius)
                    {
                        if(wj<currY){
                            if(wi>currX){
                                turn_matrix[2] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            else if(wi<currX){
                                turn_matrix[0] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                        }
                        else if(wj>currY){
                            if (wi > currX) {
                                if (wj < (split_line1 * wi + constant1)) {
                                    turn_matrix[1] += map_weight[wi][wj] / map_count[wi][wj];
                                } else if (wj > (split_line1 * wi + constant1)){
                                    turn_matrix[2] += map_weight[wi][wj] / map_count[wi][wj];
                                }
                            } else if((wi < currX) ) {
                                if (wj < (split_line2 * wi + constant2)) {
                                    turn_matrix[1] += map_weight[wi][wj] / map_count[wi][wj];
                                } else if (wj > (split_line2 * wi + constant2)){
                                    turn_matrix[0] += map_weight[wi][wj] / map_count[wi][wj];
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    /*
    if(turn_matrix[1]>turn_matrix[0]&&turn_matrix[1]>turn_matrix[2]){
        return 1;
    }
    else if(turn_matrix[0]>=turn_matrix[1]&&turn_matrix[0]>turn_matrix[2]){
        return 0;
    }
    else if(turn_matrix[2]>=turn_matrix[1]&&turn_matrix[2]>turn_matrix[0]){
        return 2;
    }
    else{
        return 1;
    }
  */
    
    
    
/*
    double turn_matrix[3] = {0.0, 0.0, 0.0};

    double slope=0.0;
    double split_line1=0.0;
    double split_line2=0.0;

    if (currX - preX == 0)
    {
        slope = 0;
        split_line1 = (slope - tan( _PI_ / 3)) / (tan( _PI_ / 3) * slope + 1);
        split_line2 = (slope + tan( _PI_ / 3)) / (1 - tan( _PI_ / 3) * slope);
    }
    else
    {
        slope = (currY - preY) / (currX - preX);
        split_line1 = (slope - tan( _PI_ / 6)) / (tan( _PI_ / 6) * slope + 1);
        split_line2 = (slope + tan( _PI_ / 6)) / (1 - tan( _PI_ / 6) * slope);
    }

    if (split_line2 > split_line1)
    {
        split_line1 = split_line1 + split_line2;
        split_line2 = split_line1 - split_line2;
        split_line1 = split_line1 - split_line2;
    }
    double constant1 = calConstant(currX, currY, split_line1);
    double constant2 = calConstant(currX, currY, split_line2);



    int deltaX = currX - preX;
    int deltaY = currY - preY;



    if (deltaX > 5 && deltaY > 5)
    {
        *turnCase = 1;
    }
    else if (deltaX < -5 && deltaY > 5)
    {
        *turnCase = 2;
    }
    else if (deltaX < -5 && deltaY < -5)
    {
        *turnCase = 3;
    }
    else if (deltaX > 5 && deltaY < -5)
    {
        *turnCase = 4;
    }
    else if (5 >= deltaX >= -5 && deltaY >= 5)
    {
        *turnCase = 5;
    }
    else if (deltaX < -5 && -5 <= deltaY <= 5)
    {
        *turnCase = 6;
    }
    else if (-5 <= deltaX <= 5 && deltaY < -5)
    {
        *turnCase = 7;
    }
    else if (deltaX > 5 && -5 <= deltaY <= 5)
    {
        *turnCase = 8;
    }

    int wi=0;
    int wj=0;


    for (int i = currX - cur_radius - 5; i <= currX + cur_radius + 5; i++)
    {


        for (int j = currY; j <= currY + cur_radius + 5; j++)
        {

            wi = out4in5((i - currX) * r_matrix[0] + (j - currY) * r_matrix[1] + currX);
            wj = out4in5((i - currX) * r_matrix[2] + (j - currY) * r_matrix[3] + currY);

            if (startX+height-1 > wi && wi>=0 && startY <= wj && wj<startY+height-1)
            {
                if (map_weight[wi][wj] > 0)
                {

                    if (pow(wi - currX, 2) + pow(wj - currY, 2) <= cur_radius)
                    {
                        switch (*turnCase)
                        {
                        case 1:
                            if (wi * split_line1 + constant1 <= wj)
                            {
                                turn_matrix[0] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line1 + constant1 > wj && wi * split_line2 + constant2 < wj)
                            {
                                turn_matrix[1] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line2 + constant2 >= wj)
                            {
                                turn_matrix[2] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            break;

                        case 2:
                            if (wi * split_line1 + constant1 <= wj)
                            {
                                turn_matrix[0] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line1 + constant1 > wj && wi * split_line2 + constant2 < wj)
                            {
                                turn_matrix[1] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line2 + constant2 >= wj)
                            {
                                turn_matrix[2] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            break;

                        case 3:
                            if (wi * split_line1 + constant1 >= wj)
                            {
                                turn_matrix[0] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line1 + constant1 < wj && wi * split_line2 + constant2 > wj)
                            {
                                turn_matrix[1] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line2 + constant2 <= wj)
                            {
                                turn_matrix[2] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            break;

                        case 4:
                            if (wi * split_line1 + constant1 <= wj)
                            {
                                turn_matrix[0] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line1 + constant1 > wj && wi * split_line2 + constant2 < wj)
                            {
                                turn_matrix[1] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line1 + constant1 >= wj)
                            {
                                turn_matrix[2] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            break;

                        case 5:
                            if (wi * split_line2 + constant2 >= wj)
                            {
                                turn_matrix[0] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line1 + constant1 < wj && wi * split_line2 + constant2 < wj)
                            {
                                turn_matrix[1] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line1 + constant1 >= wj)
                            {
                                turn_matrix[2] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            break;

                        case 6:
                            if (wi * split_line1 + constant1 >= wj)
                            {
                                turn_matrix[0] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line1 + constant1 < wj && wi * split_line2 + constant2 > wj)
                            {
                                turn_matrix[1] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line2 + constant2 <= wj)
                            {
                                turn_matrix[2] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            break;

                        case 7:
                            if (wi * split_line2 + constant2 <= wj)
                            {
                                turn_matrix[0] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line1 + constant1 > wj && wi * split_line2 + constant2 > wj)
                            {
                                turn_matrix[1] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line1 + constant1 <= wj)
                            {
                                turn_matrix[2] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            break;

                        case 8:
                            if (wi * split_line1 + constant1 <= wj)
                            {
                                turn_matrix[0] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line1 + constant1 > wj && wi * split_line2 + constant2 < wj)
                            {
                                turn_matrix[1] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line2 + constant2 >= wj)
                            {
                                turn_matrix[2] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            break;

                        default:
                            printf("Error");
                        }
                    }
                }
            }
        }

        for (int j = currY; j >= currY - cur_radius - 5; j--)
        {
            wi = out4in5((i - currX) * r_matrix[0] + (j - currY) * r_matrix[1] + currX);
            wj = out4in5((i - currX) * r_matrix[2] + (j - currY) * r_matrix[3] + currY);
            if (startX+height-1 > wi && wi>=0 && startY < wj && wj<startY+height-1)
            {
                if (map_weight[wi][wj] > 0)
                {
                    if (pow(wi - currX, 2) + pow(wj - currY, 2) <= cur_radius)
                    {
                        switch (*turnCase)
                        {
                        case 1:
                            if (wi * split_line2 + constant2 <= wj)
                            {
                                turn_matrix[2] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line1 + constant1 < wj && wi * split_line2 + constant2 > wj)
                            {
                                turn_matrix[1] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line1 + constant1 >= wj)
                            {
                                turn_matrix[0] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            break;

                        case 2:
                            if (wi * split_line2 + constant2 >= wj)
                            {
                                turn_matrix[2] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line1 + constant1 > wj && wi * split_line2 + constant2 < wj)
                            {
                                turn_matrix[1] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line1 + constant1 <= wj)
                            {
                                turn_matrix[0] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            break;

                        case 3:
                            if (wi * split_line2 + constant2 >= wj)
                            {
                                turn_matrix[2] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line1 + constant1 > wj && wi * split_line2 + constant2 < wj)
                            {
                                turn_matrix[1] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line1 + constant1 <= wj)
                            {
                                turn_matrix[0] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            break;

                        case 4:
                            if (wi * split_line2 + constant2 <= wj)
                            {
                                turn_matrix[2] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line1 + constant1 < wj && wi * split_line2 + constant2 > wj)
                            {
                                turn_matrix[1] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line1 + constant1 >= wj)
                            {
                                turn_matrix[0] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            break;

                        case 5:
                            if (wi * split_line1 + constant1 <= wj)
                            {
                                turn_matrix[2] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line1 + constant1 > wj && wi * split_line2 + constant2 > wj)
                            {
                                turn_matrix[1] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line2 + constant2 <= wj)
                            {
                                turn_matrix[0] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            break;

                        case 6:
                            if (wi * split_line2 + constant2 >= wj)
                            {
                                turn_matrix[2] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line1 + constant1 > wj && wi * split_line2 + constant2 < wj)
                            {
                                turn_matrix[1] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line1 + constant1 <= wj)
                            {
                                turn_matrix[0] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            break;

                        case 7:
                            if (wi * split_line1 + constant1 >= wj)
                            {
                                turn_matrix[2] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line1 + constant1 < wj && wi * split_line2 + constant2 < wj)
                            {
                                turn_matrix[1] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line2 + constant2 >= wj)
                            {
                                turn_matrix[0] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            break;

                        case 8:
                            if (wi * split_line2 + constant2 <= wj)
                            {
                                turn_matrix[2] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line1 + constant1 < wj && wi * split_line2 + constant2 > wj)
                            {
                                turn_matrix[1] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            if (wi * split_line1 + constant1 >= wj)
                            {
                                turn_matrix[0] += map_weight[wi][wj] / map_count[wi][wj];
                            }
                            break;

                        default:
                            printf("Error");
                        }
                    }
                }
            }
        }
    }
 */
    srand(time(NULL));
    if (cur_radius <= pre_radius)
    {
        if (*bool_predecision == 1)
        {
            *bool_predecision = 0;
            return *preturn;
        }
        else
        {
            if (turn_matrix[0] >= turn_matrix[1])
            {
                if (turn_matrix[0] > turn_matrix[2])
                {
                    return 0;
                }
                else if (turn_matrix[0] == turn_matrix[2])
                {
                    int random = rand() % 2;
                    if (random == 0)
                    {
                        return 0;
                    }
                    else
                    {
                        return 2;
                    }
                }
                else
                {
                    return 2;
                }
            }
            else if (turn_matrix[2] >= turn_matrix[1])
            {
                if (turn_matrix[2] > turn_matrix[0])
                {
                    return 2;
                }
                else if (turn_matrix[0] == turn_matrix[2])
                {
                    int random = rand() % 2;
                    if (random == 0)
                    {
                        return 2;
                    }
                    else
                    {
                        return 0;
                    }
                }
                else
                {
                    return 0;
                }
            }
            else
            {
                return 1;
            }

        }
    }
    else
    {
        *bool_predecision = 1;
        if (*preturn == 0)
        {
            *preturn = 0;
            return 0;
        }
        else if (*preturn == 2)
        {
            *preturn = 2;
            return 2;
        }
        else
        {
            int random = rand() % 2;

            if (rand == 0)
            {
                *preturn = 0;
                return 0;
            }
            else
            {
                *preturn = 2;
                return 2;
            }
        }
    }

}



void flightMove(double* currentX, double* currentY, double* preX, double* preY, int descision, double move_distance, double curYaw)
{

    *preX = *currentX;
    *preY = *currentY;

    *currentX -= move_distance*sin(curYaw);
    *currentY += move_distance*cos(curYaw);

}

double median_filter(double* rssi)
{
    double result;
    int mid = sizeof(rssi)/2;
    if(sizeof(*rssi)%2==0)
    {
        result = (rssi[mid]+rssi[mid+1])/2;
    }
    else
    {
        result = rssi[mid+1];
    }
    return result;
}

GuessPosition predictPos(double** map_weight, int** map_count, double currR, double currX, double currY, int height)  //predict positon in XY coordination
{
    double large[4] = {0.0, 0.0, 0.0, 0.0};
    double predictDistance = 0.0, distError = 0.0, threshold = 100.0;
    int errorCount=0;
    int i=0, j=0;

    GuessPosition g;


    //找到目前最大的權重
    for(i=0; i<height*2; i++)
    {
        for(j=0; j<height; j++)
        {
            if (map_weight[i][j] > 0 && map_count[i][j] > 0)
            {
                if (map_weight[i][j] / map_count[i][j] > large[0])
                {
                    large[0] = map_weight[i][j] / map_count[i][j];
                }
            }
        }
    }

    //找出dist_error 的 threshold
    for(i=0; i<height*2; i++)
    {
        for(j=0; j<height; j++)
        {
            if (map_weight[i][j] > 0 && map_count[i][j] > 0)
            {
                if ( map_weight[i][j] / map_count[i][j] > large[0] * 0.9)    //*0.9是避免RSSI誤差
                {
                    predictDistance = distance(i, j, currX, currY);
                    distError = abs(predictDistance-currR);
                    if(distError<=threshold)
                    {
                        threshold = distError;
                    }
                }
            }
        }
    }


    for(i=0; i<height*2; i++)
    {
        for(j=0; j<height; j++)
        {
            if (map_weight[i][j] > 0 && map_count[i][j] > 0)
            {
                if (map_weight[i][j] / map_count[i][j] >= large[0] * 0.9)    //*0.9是避免RSSI誤差
                {
                    large[3] += 1;
                    predictDistance = distance(i, j, currX, currY);
                    distError = abs(predictDistance-currR);
                    if(distError*0.9<=threshold)   //Hard Cord
                    {
                        errorCount++;
                        g.xt += i;
                        g.yt += j;
                    }
                }
            }
        }
    }
    g.xt = g.xt/errorCount;
    g.yt = g.yt/errorCount;
    g.threshold = distError;

    return g;
 
}


double leastSquare(double xyt, vector<double> &xy_matrix, vector<double> &latlon_matrix)
{
    int i, n=xy_matrix.size();
    double a,b;
    double y=0.0;


    double xsum=0,x2sum=0,ysum=0,xysum=0;                //variables for sums/sigma of xi,yi,xi^2,xiyi etc
    for (i=0; i<n; i++)
    {
        xsum=xsum+xy_matrix[i];                        //calculate sigma(xi)
        ysum=ysum+latlon_matrix[i];                        //calculate sigma(yi)
        x2sum=x2sum+pow(xy_matrix[i],2);                //calculate sigma(x^2i)
        xysum=xysum+xy_matrix[i]*latlon_matrix[i];                    //calculate sigma(xi*yi)
    }

    // y = ax+b
    a=(n*xysum-xsum*ysum)/(n*x2sum-xsum*xsum);            //calculate slope
    b=(x2sum*ysum-xsum*xysum)/(x2sum*n-xsum*xsum);            //calculate intercept

    y = a*xyt + b;

    return y;
}

double gaussFilter(vector<double> gauss)
{
    double u = 0.0, std = 0.0, len = gauss.size(), count = 0, gau_result = 0.0;
    
    for (int i = 0; i < len; i++) {
        u += gauss[i];
    }
    u = u / len;
    
    for (int i = 0; i < len; i++) {
        std += pow(gauss[i] - u, 2);
    }
    
    std = pow( (std/len) , 0.5);
    
    for (int i = 0; i < len; i++) {
        if ((u + 1.5 * std) >= gauss[i] && gauss[i] >= (u - 1.5 * std)) {
            gau_result += gauss[i];
            count++;
        }
    }
    
    gau_result = gau_result / count;
    
    return gau_result;
    
}

double kalman_filter(double* rssi)
{

    double X=0.0, P=3.0, Q=0.0001, K=0.0, R=1.0, I=1.0;

    for(int i=0; i<sizeof(*rssi); i++)
    {
        X=X;
        P = P+Q;
        K = P / (P + R);
        X = X + K * (rssi[i] - X);
        P = (I - K) * P;
    }

    return X;

}

int out4in5(double x)
{
    x = x+0.5;
    x = floor(x);
    return (int)x;
}
