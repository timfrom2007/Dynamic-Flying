#include "search.h"
#define _PI_ 3.14159265358979323846
double latitude(const Flight *flight)
{
    return flight->getPosition().latitude;
}
double longitude(const Flight *flight)
{
    return flight->getPosition().longitude;
}
double altitude(const Flight *flight)
{
    return flight->getPosition().altitude;
}
double getYaw(const Flight *flight){
    QuaternionData q = flight->getQuaternion();
    float yaw=atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1));
    return yaw;
}
void control(VirtualRC *vrc ,int pitch,int yaw)
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
    vector<PointData> *record =  params->record;;
    time_t startTime = time(0);
    clock_t start=clock(), finish;

    while(params->isFlying){
        usleep(100*1000);
        //finish = clock();
        PointData p;
        p.latitude = latitude(params->flight);
        p.longitude = longitude(params->flight);
        p.altitude = altitude(params->flight);
        p.RSSI = getFakeRSSI(params->flight,0.393448,1.988934,0, p.startSearch);

        struct filtering_result{
            float median;
        } filter;

		finish = clock();

		p.ctimeStamp = (finish - start)/ CLOCKS_PER_SEC;
		record->push_back(p);

    }
    return 0;
    /*
        iwrange range;
        int sock;
        wireless_info info;
        sock = iw_sockets_open();
        int collect[50];


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
                collect[count] = (int8_t)info.stats.qual.level;
                //printf("%d\n",(int8_t)info.stats.qual.level);
                count++;
            }
            //usleep(100);
            i++;
        }

		sort(collect, collect+50);
	  	filter.median = median_filter(collect);

		p.RSSI = filter.median;
    */
}

vector<PointData> planPath(CoreAPI *api){


    /////////////////////
    // extern variable //
    /////////////////////

    int turnCase = 5;
    int preturn = 1;
    int descision = 1;
    int bool_predecision = 0;  //Previous decision is correct or not

    /////////////////////
    // 2D Array malloc //
    /////////////////////

    // 生成一維指標陣列
    int count=0;
    int i=0, j=0;
    double **map_weight = new double*[600];
    int **map_count = new int*[600];

    // 每個指標陣列再生成整數陣列
    for(i=0; i<600; i++){
        map_count[i] = new int[1200];
        map_weight[i] = new double[1200];
    }
    // write
    for(i=0; i<600; i++) {
        for(j=0; j<1200; j++){
            map_count[i][j] = 0;
            map_weight[i][j] = 0.0;
        }
    }
    /////////////////////
    // extern variable //
    /////////////////////


    const Flight *flight = new Flight(api);
    double curYaw = 0.0;
    double currX = 600.0, currY=0.0, preX=0.0, preY=0.0;  //X and Y, in XY coordinate system
    double currLat=0.0, currLon=0.0, preLat=0.0, preLon=0.0; //Latitude and Longitude, in Geographic coordinate system
    double guessX=0.0, guessY=0.0, guessLon=0.0, guessLat=0.0;  //guess Target position, need to transform
    double Xa=0.0, Ya=0.0, Xb=0.0, Yb=0.0, Xt=0.0, Yt=0.0; //For CoordinateChanger
    vector<double> r_matrix;

    // Start Searching
    vector<PointData> record = goFind(api,"./preFlight.txt");  //main record
    vector<PointData> searchRecord = goFind(api,"./moveStraight.txt"); // each turn record

    currLat = searchRecord[searchRecord.size()-1].latitude;
    currLon = searchRecord[searchRecord.size()-1].longitude;
    preLat = record[record.size()-1].latitude;
    preLon = record[record.size()-1].longitude;

    double moveDistance = earth_distance(currLat, currLon, preLat, preLon, 'K') * 1000; //km to m
    double cur_radius = rssiToDist(searchRecord[searchRecord.size()-1].RSSI, searchRecord[searchRecord.size()-1].altitude);
    double pre_radius = rssiToDist(record[record.size()-1].RSSI, record[record.size()-1].altitude);

    record.insert( record.end(), searchRecord.begin(), searchRecord.end() );

    do{
        count+=1;
        if(count==20){
            for(i=0; i<600; i++) {
                for(j=0; j<1200; j++){
                    map_count[i][j] = 0;
                    map_weight[i][j] = 0.0;
                }
            }
        }


        curYaw = getYaw(flight);
        curYaw = curYaw * -1;
        if(curYaw<0){
            curYaw = 2*_PI_ + curYaw;
        }

        flightMove(&currX, &currY, &preX, &preY, descision, moveDistance, curYaw);

        Xa=currX;  Ya=currY;  Xb=preX; Yb=preY;
        coordinateChanger(Xt, Yt, Xa, Ya, Xb, Yb, currLat, currLon, preLat, preLon);

        r_matrix.clear();

        rotation_matrix(currX, currY, preX, preY, r_matrix, curYaw);
        cout << "Flag:7" <<endl;

        addWeight(currX, currY, preX, preY, cur_radius, pre_radius, map_weight, map_count, r_matrix);
        cout << "Flag:8" <<endl;
        descision = turnDecision(currX, currY, preX, preY, &preturn, &bool_predecision, &turnCase, cur_radius, pre_radius, map_weight, map_count, r_matrix);

        cout << "Decision: " << descision << " TurnCase: " << turnCase <<endl;

        searchRecord.clear();
        if(descision==0){
            searchRecord = goFind(api,"./moveLeft.txt");
        }
        else if(descision==1){
            searchRecord = goFind(api,"./moveStraight.txt");
        }
        else if(descision==2){
            searchRecord = goFind(api,"./moveRight.txt");
        }


        currLat = searchRecord[searchRecord.size()-1].latitude;
        currLon = searchRecord[searchRecord.size()-1].longitude;
        preLat = record[record.size()-1].latitude;
        preLon = record[record.size()-1].longitude;

        moveDistance = earth_distance(currLat, currLon, preLat, preLon, 'K') * 1000;
        cout << "moveDistance: " << moveDistance << endl;

        cur_radius = rssiToDist(searchRecord[searchRecord.size()-1].RSSI, searchRecord[searchRecord.size()-1].altitude);
        pre_radius = rssiToDist(record[record.size()-1].RSSI, record[record.size()-1].altitude);
        record.insert( record.end(), searchRecord.begin(), searchRecord.end() );

        cout << "Record Size:" << record.size() << " sRecord Size:" << searchRecord.size() <<endl;
        predictPos(map_weight, map_count, &Xt, &Yt);

        cout<< "------------------------------------\n" <<endl;

    }while(cur_radius>20);



/*
	    if(vote_4>=0){
                record2 = goFind(api,"./pathZone1.txt");
                record.insert( record.end(), record2.begin(), record2.end() );
            cout << 6 << endl;
	    }
            else{
                record2 = goFind(api,"./pathZone1.txt");
                record3 = goFind(api,"./pathZone3.txt");
                record.insert( record.end(), record2.begin(), record2.end() );
            cout << 7 << endl;

*/
    return record;
}

vector<PointData> goFind(CoreAPI *api,const char *pathFile)
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

    do{
    	control(&vrc,pitch,yaw);
    	usleep(100 * 1000);
    	fscanf(fp,"%d %d",&pitch,&yaw);

    }while(!feof(fp));

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
    double n=2, A= -10; //n:path-loss exponent, A:RSSI per unit
    double rssi = A - 10*n*log10(dist); // + 1.4 * normalDistribution();
    /*
    if(dist>300 && startSearch==0){
		rssi = 1;
	}
	*/
    return rssi;
}




double deg2rad(double deg) {
  return (deg * _PI_ / 180);
}
double rad2deg(double rad) {
  return (rad * 180 / _PI_);
}

double earth_distance(double lat1, double lon1, double lat2, double lon2, char unit) {
  double theta, dist;
  theta = lon1 - lon2;
  dist = sin(lat1) * sin(lat2) + cos(lat1) * cos(lat2) * cos(theta);
  dist = acos(dist);
  dist = rad2deg(dist);
  dist = dist * 60 * 1.1515;
  dist = dist * 1.609344;
  return (dist);
}

void coordinateChanger(double xt, double yt, double xa, double ya, double xb, double yb, double currLat, double currLon, double preLat, double preLon){  //XY coordinate to Lon,Lat coordinate
	/*  we solve the linear system
     *  ax+by=e  lonA*lon + latA*lat = xt*xa + yt*ya
     *  cx+dy=f  lonB*lon + latB*lat = xt*xb + yt*yb
     */
    /* we solve the linear system
     * ax+by=e
     * cx+dy=f
     */
    xt -=600;
    xa -=600;
    xb -=600;
    double a = currLon-1.988958;
	double b = currLat-0.393446;
	double c = preLon-1.988958;
	double d = preLat-0.393446;
	double e = xt*xa + yt*ya;
	double f = xt*xb + yt*yb;

	double tarLong=0.0, tarLat=0.0;

	double determinant = a*d - b*c;

	cout << "a:" << a << " b:" << b << " c:" << c << " d:" << d << " e:" << e << " f:" << f << " deter:" << determinant <<endl;

    if(determinant != 0) {
        tarLong = (e*d - b*f)/determinant;
        tarLat = (a*f - e*c)/determinant;
    } else {
        printf("Cramer equations system: determinant is zero\n");
    }

	cout << "predictLong:" <<tarLong << " predictLat:" << tarLat <<endl;
}
/*
PointData calculatePos(vector<PointData> *record) //Pure RSSI
{
    PointData pData;
    double pLat=0,pLon=0;
    double totalWeight=0;
    for(int i=0;i<record->size();i++){
	pLat += (*record)[i].latitude * (1/(*record)[i].RSSI);
	pLon += (*record)[i].longitude * (1/(*record)[i].RSSI);
	totalWeight += (1/(*record)[i].RSSI);
    }
    pData.latitude = pLat/totalWeight;
    pData.longitude = pLon/totalWeight;
    return pData;
}
PointData calculatePos2(vector<PointData> *record) //Differential RSSI
{
    PointData pData;
    double pLat=0,pLon=0;
    double minRSSI=0,maxRSSI=-9999999;
    double totalWeight=0;
    for(int i=0;i<record->size();i++){
	if((*record)[i].RSSI>maxRSSI) {maxRSSI=(*record)[i].RSSI;}
	if((*record)[i].RSSI<minRSSI) {minRSSI=(*record)[i].RSSI;}
    }
    for(int i=0;i<record->size();i++){
	pLat += (*record)[i].latitude * (2-((maxRSSI-(*record)[i].RSSI)/(maxRSSI-minRSSI)));
	pLon += (*record)[i].longitude * (2-((maxRSSI-(*record)[i].RSSI)/(maxRSSI-minRSSI)));
	totalWeight += (2-((maxRSSI-(*record)[i].RSSI)/(maxRSSI-minRSSI)));
    }
    pData.latitude = pLat/totalWeight;
    pData.longitude = pLon/totalWeight;
    return pData;
}
PointData calculatePos3(vector<PointData> *record) //Trilateration
{
    PointData pData;
    double pLat=0, pLon=0;
    double eXx, eXy, eYx, eYy; //Unit Vector
    double dis12, dis13; //distance P1-P2
    double i,ix,iy,j,jx,jy; //coefficient
    double x,y; //location position
    double r1, r2, r3; // p123 rssi to distance
    int p1=record->size()/4-3, p2=record->size()/2-3, p3=record->size()/2+3;
    r1 = rssiToDist((*record)[p1].RSSI, (*record)[p1].altitude);
    r2 = rssiToDist((*record)[p2].RSSI, (*record)[p2].altitude);
    r3 = rssiToDist((*record)[p3].RSSI, (*record)[p3].altitude);

    dis12 = 1000* distance((*record)[p1].latitude,(*record)[p1].longitude, (*record)[p2].latitude, (*record)[p2].longitude, 'K'); //unit meter
    dis13 = 1000* distance((*record)[p1].latitude,(*record)[p1].longitude, (*record)[p3].latitude, (*record)[p3].longitude, 'K');// unit meter
    eXx = ((*record)[p2].latitude - (*record)[p1].latitude) /dis12;
    eXy = ((*record)[p2].longitude - (*record)[p1].longitude) /dis12;
    ix = eXx *((*record)[p3].latitude - (*record)[p1].latitude);
    iy = eXy *((*record)[p3].longitude - (*record)[p1].longitude);
    i = ix+iy;
    eYx = ((*record)[p3].latitude - (*record)[p1].latitude - (ix*eXx) ) / sqrt(pow((*record)[p3].latitude-(*record)[p1].latitude-ix*eXx, 2)+ pow((*record)[p3].longitude-(*record)[p1].longitude-iy*eXy, 2));
    eYy = ((*record)[p3].longitude - (*record)[p1].longitude -(iy*eXy) ) / sqrt(pow((*record)[p3].latitude-(*record)[p1].latitude-ix*eXx, 2)+ pow((*record)[p3].longitude-(*record)[p1].longitude-iy*eXy, 2));
    jx = eYx *((*record)[p3].latitude - (*record)[p1].latitude);
    jy = eYy *((*record)[p3].longitude - (*record)[p1].longitude);
    j = jx+jy;
    x = (pow(r1,2)-pow(r2,2)+pow(dis12,2)) / (2*dis12);
    y = ((pow(r1,2)-pow(r3,2)+pow(i,2)+pow(j,2)) / (2*j)) - (i*x/j);
    pData.latitude = x;
    pData.longitude = y;
    return pData;

}
PointData calculatePos4(vector<PointData> *record) //Modified Differential RSSI
{
    PointData pData;
    double pLat=0,pLon=0;
    double minRSSI=0,maxRSSI=-9999999;
    double totalWeight=0;
    for(int i=0;i<record->size();i++){
	if((*record)[i].RSSI>maxRSSI) {maxRSSI=(*record)[i].RSSI;}
	if((*record)[i].RSSI<minRSSI) {minRSSI=(*record)[i].RSSI;}
    }
    cout<<"max"<<maxRSSI<<" min"<<minRSSI<<endl;
    for(int i=0;i<record->size();i++){
	pLat += (*record)[i].latitude * pow(20,(2-((maxRSSI-(*record)[i].RSSI)/(maxRSSI-minRSSI))));
	pLon += (*record)[i].longitude * pow(20,(2-((maxRSSI-(*record)[i].RSSI)/(maxRSSI-minRSSI))));
	totalWeight += pow(20,(2-((maxRSSI-(*record)[i].RSSI)/(maxRSSI-minRSSI))));
	//cout<<pow(20,(2-((maxRSSI-(*record)[i].RSSI)/(maxRSSI-minRSSI))))<<' ';
    }
    pData.latitude = pLat/totalWeight;
    pData.longitude = pLon/totalWeight;
    return pData;
}

*/
double rssiToDist(double rssi, double altitude)
{
    double n=2, A=-10; //n:path-loss exponent, A:RSSI per unit
    double dist=10, exp= (A-rssi) / (10*n);
    dist = pow(dist, exp);
    dist = sqrt(pow(dist,n) + pow(altitude, n));
    return dist;
}

double normalDistribution()
{
    srand(time(NULL));
    double u = rand() / (double)RAND_MAX;
    double v = rand() / (double)RAND_MAX;
    double x = sqrt(-2 * log(u)) * cos (2 * _PI_ * v);
    return x;
}


/*

Dynamic Path Planning Function

*/


void rotation_matrix(double currX, double currY, double preX, double preY, vector<double> &r_matrix, double curYaw){

    /*
    double degree = _PI_ / 4;
    int deltaX = currX - preX;
    int deltaY = currY - preY;

    if (deltaX != 0 && deltaY != 0) {
        if (deltaX > 5 && deltaY > 5) {
            degree = degree * 7;
        } else if (deltaX > 5 && deltaY < -5) {
            degree = degree * 5;
        } else if (deltaX < -5 && deltaY > 5) {
            degree = degree * 1;
        } else if (deltaX < -5 && deltaY < -5) {
            degree = degree * 3;

        } else if (-5 <= deltaX <= 5 && deltaY > 5) {
            degree = degree * 0;
        } else if (deltaX < -5 && -5 <= deltaY <= 5) {
            degree = degree * 2;
        } else if (-5 <= deltaX <= 5 && deltaY < -5) {
            degree = degree * 4;
        } else if (deltaX > 5 && -5 <= deltaY <= 5) {
            degree = degree * 6;
        }
    } else if (deltaX == 0 && deltaY != 0) {
        if (deltaY > 0) {
            degree = degree * 0;
        } else if (deltaY < 0) {
            degree = degree * 4;
        }

    } else if (deltaX != 0 && deltaY == 0) {
        if (deltaX > 0) {
            degree = degree * 6;
        } else if (deltaX < 0) {
            degree = degree * 2;
        }
    }
    */

    r_matrix.push_back(cos(curYaw));
    r_matrix.push_back(sin(curYaw));
    r_matrix.push_back(-sin(curYaw));
    r_matrix.push_back(cos(curYaw));
}

double moveDistance_to_speed(double move_distance){

    double speed;
    if(move_distance>=5){
	speed = 5;
    }
    else{
	speed = move_distance;
    }

    return speed;
}

double distance(int x1, int y1, int x2, int y2) {
    double d = pow(pow((x1 - x2), 2) + pow((y1 - y2), 2), 0.5);
    return d;
}

void addWeight(double currX, double currY, double preX, double preY, double cur_radius, double pre_radius, double** map_weight, int** map_count, vector<double> r_matrix) {

    int weight_i=0;
    int weight_j=0;

    cout << "PreX:" << preX << " PreY:" << preY  <<endl;
    cout << "CurrX: " << currX<< " CurrY:"<<currY <<endl;
    cout << "Cur R:" << cur_radius << " Pre R:" << pre_radius << endl;
    if (cur_radius - pre_radius > 0) {  //飛機遠離目標
        for (int i = currX - cur_radius - 10; i <= currX + cur_radius + 10; i++) {
            for (int j = currY; j >= currY - cur_radius + 10; j--) {
                weight_i = floor((i - currX) * r_matrix[0] + (j - currY) * r_matrix[1]) + floor(currX);
                weight_j = floor((i - currX) * r_matrix[2] + (j - currY) * r_matrix[3]) + floor(currY);
                if ( 600>weight_i && weight_i>=0 && 1200>weight_j && weight_j>=0 ) {
                    if (map_weight[weight_i][weight_j] >= 0) {
                        double dist = distance(weight_i, weight_j, currX, currY);
                        if (cur_radius >= dist) {
                            if (dist <= cur_radius) {
                                map_weight[weight_i][weight_j] += dist / pow((cur_radius), 2);
                                map_count[weight_i][weight_j] += 1;
                            } else {
                                map_weight[weight_i][weight_j] += (cur_radius * 2 - dist) / pow((cur_radius), 2);
                                map_count[weight_i][weight_j] += 1;
                            }
                        }
                    }
                }
            }
            for (int j = currY; j <= currY + cur_radius; j++) {
                weight_i = floor((i - currX) * r_matrix[0] + (j - currY) * r_matrix[1]) + floor(currX);
                weight_j = floor((i - currX) * r_matrix[2] + (j - currY) * r_matrix[3]) + floor(currY);
                if (600>weight_i && weight_i>=0 && 1200>weight_j && weight_j>=0) {
                    if (map_weight[weight_i][weight_j] >= 0) {
                        double dist = distance(weight_i, weight_j, currX, currY);
                        if (cur_radius >= dist) {
                            map_weight[weight_i][weight_j] += (dist / pow((cur_radius), 2))*0.8;
                            map_count[weight_i][weight_j] += 1;
                        }
                    }
                }
            }
        }
    } else if (cur_radius - pre_radius < 0) { //飛機靠近目標
        for (int i = currX - cur_radius - 10; i <= currX + cur_radius + 10; i++) {
            for (int j = currY; j <= currY + cur_radius + 10; j++) {
                weight_i = floor((i - currX) * r_matrix[0] + (j - currY) * r_matrix[1]) + floor(currX);
                weight_j = floor((i - currX) * r_matrix[2] + (j - currY) * r_matrix[3]) + floor(currY);
                if (600>weight_i && weight_i>=0 && 1200>weight_j && weight_j>=0) {
                    if (map_weight[weight_i][weight_j] >= 0) {
                        double dist = distance(weight_i, weight_j, currX, currY);
                        if (cur_radius >= dist) {

                            if (dist <= cur_radius) {
                                map_weight[weight_i][weight_j] += dist / pow((cur_radius), 2);
                                map_count[weight_i][weight_j] += 1;
                            } else {
                                map_weight[weight_i][weight_j] += (cur_radius * 2 - dist) / pow((cur_radius), 2);
                                map_count[weight_i][weight_j] += 1;
                            }
                        }
                    }
                }
            }
            for (int j = currY; j >= currY - cur_radius; j--) {
                weight_i = floor((i - currX) * r_matrix[0] + (j - currY) * r_matrix[1]) + floor(currX);
                weight_j = floor((i - currX) * r_matrix[2] + (j - currY) * r_matrix[3]) + floor(currY);
                if (600>weight_i && weight_i>=0 && 1200>weight_j && weight_j>=0) {
                    if (map_weight[weight_i][weight_j] >= 0) {
                        double dist = distance(weight_i, weight_j, currX, currY);
                        if (cur_radius >= dist) {
                            map_weight[weight_i][weight_j] += (dist / pow((cur_radius), 2))*0.8;
                            map_count[weight_i][weight_j] += 1;
                        }
                    }
                }
            }
        }
    }
}

double calConstant(double x, double y, double m) {

    double b = y - (m * x);
    return b;
}

int turnDecision(double currX, double currY, double preX, double preY, int* preturn, int* bool_predecision, int* turnCase, double cur_radius, double pre_radius, double** map_weight, int** map_count, vector<double> r_matrix) {

    double turn_matrix[3] = {0.0, 0.0, 0.0};

    double slope=0.0;
    double split_line1=0.0;
    double split_line2=0.0;



    if (currX - preX == 0) {
        slope = 0;
        split_line1 = (slope - tan( _PI_ / 3)) / (tan( _PI_ / 3) * slope + 1);
        split_line2 = (slope + tan( _PI_ / 3)) / (1 - tan( _PI_ / 3) * slope);
    } else {
        slope = (currY - preY) / (currX - preX);
        split_line1 = (slope - tan( _PI_ / 6)) / (tan( _PI_ / 6) * slope + 1);
        split_line2 = (slope + tan( _PI_ / 6)) / (1 - tan( _PI_ / 6) * slope);
    }

    if (split_line2 > split_line1) {
        split_line1 = split_line1 + split_line2;
        split_line2 = split_line1 - split_line2;
        split_line1 = split_line1 - split_line2;
    }
    double constant1 = calConstant(currX, currY, split_line1);
    double constant2 = calConstant(currX, currY, split_line2);

        cout << "turn Flag:1" << endl;

    int deltaX = currX - preX;
    int deltaY = currY - preY;

    cout << "deltaX:" << deltaX << " ,deltaY:" << deltaY <<endl;

    if (deltaX > 5 && deltaY > 5) {
        *turnCase = 1;
    } else if (deltaX < -5 && deltaY > 5) {
        *turnCase = 2;
    } else if (deltaX < -5 && deltaY < -5) {
        *turnCase = 3;
    } else if (deltaX > 5 && deltaY < -5) {
        *turnCase = 4;
    } else if (5 >= deltaX >= -5 && deltaY >= 5) {
        *turnCase = 5;
    } else if (deltaX < -5 && -5 <= deltaY <= 5) {
        *turnCase = 6;
    } else if (-5 <= deltaX <= 5 && deltaY < -5) {
        *turnCase = 7;
    } else if (deltaX > 5 && -5 <= deltaY <= 5) {
        *turnCase = 8;
    }

    int weight_i=0;
    int weight_j=0;

        cout << "turn Flag:2" << endl;

    if (cur_radius - pre_radius <= 0) {
        for (int i = currX - cur_radius - 10; i <= currX + cur_radius + 10; i++) {
            for (int j = currY; j <= currY + cur_radius + 10; j++) {
                weight_i = floor((i - currX) * r_matrix[0] + (j - currY) * r_matrix[1]) + floor(currX);
                weight_j = floor((i - currX) * r_matrix[2] + (j - currY) * r_matrix[3]) + floor(currY);
                if (600>weight_i && weight_i>=0 && 1200>weight_j && weight_j>=0) {
                    if (map_weight[weight_i][weight_j] > 0) {
                        if (pow(weight_i - currX, 2) + pow(weight_j - currY, 2) <= cur_radius) {
                            switch (*turnCase) {
                                case 1:
                                    if (weight_i * split_line1 + constant1 <= weight_j) {
                                        turn_matrix[0] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 > weight_j && weight_i * split_line2 + constant2 < weight_j) {
                                        turn_matrix[1] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line2 + constant2 >= weight_j) {
                                        turn_matrix[2] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    break;

                                case 2:
                                    if (weight_i * split_line1 + constant1 <= weight_j) {
                                        turn_matrix[0] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 > weight_j && weight_i * split_line2 + constant2 < weight_j) {
                                        turn_matrix[1] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line2 + constant2 >= weight_j) {
                                        turn_matrix[2] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    break;

                                case 3:
                                    if (weight_i * split_line1 + constant1 >= weight_j) {
                                        turn_matrix[0] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 < weight_j && weight_i * split_line2 + constant2 > weight_j) {
                                        turn_matrix[1] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line2 + constant2 <= weight_j) {
                                        turn_matrix[2] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    break;

                                case 4:
                                    if (weight_i * split_line1 + constant1 <= weight_j) {
                                        turn_matrix[0] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 > weight_j && weight_i * split_line2 + constant2 < weight_j) {
                                        turn_matrix[1] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 >= weight_j) {
                                        turn_matrix[2] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    break;

                                case 5:
                                    if (weight_i * split_line2 + constant2 >= weight_j) {
                                        turn_matrix[0] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 < weight_j && weight_i * split_line2 + constant2 < weight_j) {
                                        turn_matrix[1] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 >= weight_j) {
                                        turn_matrix[2] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    break;

                                case 6:
                                    if (weight_i * split_line1 + constant1 >= weight_j) {
                                        turn_matrix[0] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 < weight_j && weight_i * split_line2 + constant2 > weight_j) {
                                        turn_matrix[1] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line2 + constant2 <= weight_j) {
                                        turn_matrix[2] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    break;

                                case 7:
                                    if (weight_i * split_line2 + constant2 <= weight_j) {
                                        turn_matrix[0] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 > weight_j && weight_i * split_line2 + constant2 > weight_j) {
                                        turn_matrix[1] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 <= weight_j) {
                                        turn_matrix[2] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    break;

                                case 8:
                                    if (weight_i * split_line1 + constant1 <= weight_j) {
                                        turn_matrix[0] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 > weight_j && weight_i * split_line2 + constant2 < weight_j) {
                                        turn_matrix[1] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line2 + constant2 >= weight_j) {
                                        turn_matrix[2] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
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
    } else if (cur_radius - pre_radius > 0) {
        for (int i = currX - cur_radius - 10; i <= currX + cur_radius + 10; i++) {
            for (int j = currY; j >= currY - cur_radius - 10; j--) {
                weight_i = floor((i - currX) * r_matrix[0] + (j - currY) * r_matrix[1]) + floor(currX);
                weight_j = floor((i - currX) * r_matrix[2] + (j - currY) * r_matrix[3]) + floor(currY);
                if (600>weight_i && weight_i>=0 && 1200>weight_j && weight_j>=0) {
                    if (map_weight[weight_i][weight_j] > 0) {
                        if (pow(weight_i - currX, 2) + pow(weight_j - currY, 2) <= cur_radius) {
                            switch (*turnCase) {
                                case 1:
                                    if (weight_i * split_line2 + constant2 <= weight_j) {
                                        turn_matrix[2] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 < weight_j && weight_i * split_line2 + constant2 > weight_j) {
                                        turn_matrix[1] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 >= weight_j) {
                                        turn_matrix[0] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    break;

                                case 2:
                                    if (weight_i * split_line2 + constant2 >= weight_j) {
                                        turn_matrix[2] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 > weight_j && weight_i * split_line2 + constant2 < weight_j) {
                                        turn_matrix[1] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 <= weight_j) {
                                        turn_matrix[0] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    break;

                                case 3:
                                    if (weight_i * split_line2 + constant2 >= weight_j) {
                                        turn_matrix[2] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 > weight_j && weight_i * split_line2 + constant2 < weight_j) {
                                        turn_matrix[1] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 <= weight_j) {
                                        turn_matrix[0] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    break;

                                case 4:
                                    if (weight_i * split_line2 + constant2 <= weight_j) {
                                        turn_matrix[2] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 < weight_j && weight_i * split_line2 + constant2 > weight_j) {
                                        turn_matrix[1] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 >= weight_j) {
                                        turn_matrix[0] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    break;

                                case 5:
                                    if (weight_i * split_line1 + constant1 <= weight_j) {
                                        turn_matrix[2] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 > weight_j && weight_i * split_line2 + constant2 > weight_j) {
                                        turn_matrix[1] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line2 + constant2 <= weight_j) {
                                        turn_matrix[0] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    break;

                                case 6:
                                    if (weight_i * split_line2 + constant2 >= weight_j) {
                                        turn_matrix[2] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 > weight_j && weight_i * split_line2 + constant2 < weight_j) {
                                        turn_matrix[1] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 <= weight_j) {
                                        turn_matrix[0] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    break;

                                case 7:
                                    if (weight_i * split_line1 + constant1 >= weight_j) {
                                        turn_matrix[2] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 < weight_j && weight_i * split_line2 + constant2 < weight_j) {
                                        turn_matrix[1] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line2 + constant2 >= weight_j) {
                                        turn_matrix[0] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    break;

                                case 8:
                                    if (weight_i * split_line2 + constant2 <= weight_j) {
                                        turn_matrix[2] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 < weight_j && weight_i * split_line2 + constant2 > weight_j) {
                                        turn_matrix[1] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 >= weight_j) {
                                        turn_matrix[0] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
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
    }

        cout << "turn Flag:3" << endl;


    srand(time(NULL));
    if (cur_radius < pre_radius) {
        if (*bool_predecision == 1) {
            *bool_predecision = 0;
            return *preturn;
        } else {
            if (turn_matrix[0] >= turn_matrix[1]) {
                if (turn_matrix[0] > turn_matrix[2]) {
                    return 0;
                } else if (turn_matrix[0] == turn_matrix[2]) {
                    int random = rand() % 2;
                    if (random == 0) {
                        return 0;
                    } else {
                        return 2;
                    }
                } else {
                    return 2;
                }
            } else if (turn_matrix[2] >= turn_matrix[1]) {
                if (turn_matrix[2] > turn_matrix[0]) {
                    return 2;
                } else if (turn_matrix[0] == turn_matrix[2]) {
                    int random = rand() % 2;
                    if (random == 0) {
                        return 2;
                    } else {
                        return 0;
                    }
                } else {
                    return 0;
                }
            } else {
                return 1;
            }

        }
    } else {
        *bool_predecision = 1;
        if (*preturn == 0) {
            *preturn = 0;
            return 0;
        } else if (*preturn == 2) {
            *preturn = 2;
            return 2;
        } else {
            int random = rand() % 2;

            if (rand == 0) {
                *preturn = 0;
                return 0;
            } else {
                *preturn = 2;
                return 2;
            }
        }
    }
}



void flightMove(double* currentX, double* currentY, double* preX, double* preY, int descision, double move_distance, double curYaw) {

    *preX = *currentX;
    *preY = *currentY;


    *currentX -= move_distance*sin(curYaw);
    *currentY += move_distance*cos(curYaw);
    /*
    switch (abs(turnCase)) {
        case 1:
            if (descision == 0) {
                *currentX += 0;
                *currentY += move_distance;
            } else if (descision == 1) {
                *currentX += pow((pow(move_distance, 2)) / 2, 0.5);
                *currentY += pow((pow(move_distance, 2)) / 2, 0.5);
            } else if (descision == 2) {
                *currentX += move_distance;
                *currentY += 0;
            }
            break;
        case 2:
            if (descision == 0) {
                *currentX -= move_distance;
                *currentY += 0;
            } else if (descision == 1) {
                *currentX -= pow((pow(move_distance, 2)) / 2, 0.5);
                *currentY += pow((pow(move_distance, 2)) / 2, 0.5);
            } else if (descision == 2) {
                *currentX += 0;
                *currentY += move_distance;
            }
            break;
        case 3:
            if (descision == 0) {
                *currentX += 0;
                *currentY -= move_distance;
            } else if (descision == 1) {
                *currentX -= pow((pow(move_distance, 2)) / 2, 0.5);
                *currentY -= pow((pow(move_distance, 2)) / 2, 0.5);
            } else if (descision == 2) {
                *currentX -= move_distance;
                *currentY += 0;
            }
            break;
        case 4:
            if (descision == 0) {
                *currentX += move_distance;
                *currentY += 0;
            } else if (descision == 1) {
                *currentX += pow((pow(move_distance, 2)) / 2, 0.5);
                *currentY -= pow((pow(move_distance, 2)) / 2, 0.5);
            } else if (descision == 2) {
                *currentX += 0;
                *currentY += -move_distance;
            }
            break;
        case 5:
            if (descision == 0) {
                *currentX -= pow((pow(move_distance, 2)) / 2, 0.5);
                *currentY += pow((pow(move_distance, 2)) / 2, 0.5);
            } else if (descision == 1) {
                *currentX += 0;
                *currentY += move_distance;
            } else if (descision == 2) {
                *currentX += pow((pow(move_distance, 2)) / 2, 0.5);
                *currentY += pow((pow(move_distance, 2)) / 2, 0.5);
            }
            break;
        case 6:
            if (descision == 0) {
                *currentX -= pow((pow(move_distance, 2)) / 2, 0.5);
                *currentY -= pow((pow(move_distance, 2)) / 2, 0.5);
            } else if (descision == 1) {
                *currentX -= move_distance;
                *currentY += 0;
            } else if (descision == 2) {
                *currentX -= pow((pow(move_distance, 2)) / 2, 0.5);
                *currentY += pow((pow(move_distance, 2)) / 2, 0.5);
            }
            break;
        case 7:
            if (descision == 0) {
                *currentX += pow((pow(move_distance, 2)) / 2, 0.5);
                *currentY -= pow((pow(move_distance, 2)) / 2, 0.5);
            } else if (descision == 1) {
                *currentX += 0;
                *currentY -= move_distance;
            } else if (descision == 2) {
                *currentX -= pow((pow(move_distance, 2)) / 2, 0.5);
                *currentY -= pow((pow(move_distance, 2)) / 2, 0.5);
            }
            break;
        case 8:
            if (descision == 0) {
                *currentX += pow((pow(move_distance, 2)) / 2, 0.5);
                *currentY += pow((pow(move_distance, 2)) / 2, 0.5);
            } else if (descision == 1) {
                *currentX += move_distance;
                *currentY += 0;
            } else if (descision == 2) {
                *currentX += pow((pow(move_distance, 2)) / 2, 0.5);
                *currentY -= pow((pow(move_distance, 2)) / 2, 0.5);
            }
            break;
        default:
            cout << "flightMove Error" << endl;
    }
    */

}

double median_filter(int* rssi)
{
    double result;
    int mid = sizeof(rssi)/2;
    if(sizeof(*rssi)%2==0){
        result = (rssi[mid]+rssi[mid+1])/2;
    }
    else{
        result = rssi[mid+1];
    }
    return result;
}

void predictPos(double** map_weight, int** map_count, double* Xt, double* Yt)
{
     double large[4] = {0.0, 0.0, 0.0, 0.0};

     int i=0, j=0;
     for(i=0; i<600; i++) {
        for(j=0; j<1200; j++){
            if (map_weight[i][j] > 0 && map_count[i][j] > 0) {
                if (map_weight[i][j] / map_count[i][j] > large[0]) {
                    large[0] = map_weight[i][j] / map_count[i][j];
                }
            }
        }
    }

    for (i = 0; i < 600; i++) {
        for (j = 0; j < 1200; j++) {
            if (map_weight[i][j] > 0 && map_count[i][j] > 0) {
                if (map_weight[i][j] / map_count[i][j] >= large[0] * 0.9) {
                    large[1] += i;
                    large[2] += j;
                    large[3] += 1;
                }
            }
        }
    }

    *Xt = large[1] / large[3];
    *Yt = large[2] / large[3];

    cout << "X:" << *Xt << " Y:" << *Yt << " Count:" << large[3] << endl;

}

