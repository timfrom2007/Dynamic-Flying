#include "search.h"
#define _PI_ 3.14159265358979323846
double latitude(const Flight *flight)
{
    return flight->getPosition().latitude;
}
double longitude(const Flight *flight)
{
    cout << "YOYO" <<  setprecision(20) << flight->getPosition().longitude << endl;
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
    vector<PointData> *record =  params->record;
    time_t startTime = time(0);

    while(params->isFlying){
        usleep(100*1000);
        PointData p;
        p.latitude = latitude(params->flight);
        p.longitude = longitude(params->flight);
        p.altitude = altitude(params->flight);
        p.ctimeStamp = clock();

        double collect[100];
        int i=0;
        while(i<100){
            collect[i] = getFakeRSSI(params->flight,0.393463,1.988969, 0, p.startSearch); //Hard Cord
            i++;
        }
        cout << median_filter(collect) <<endl;
        p.RSSI = median_filter(collect);


        struct filtering_result{
            double median;
        } filter;



		record->push_back(p);
        return 0;
    }

/*
        iwrange range;
        int sock;
        wireless_info info;
        sock = iw_sockets_open();
        int collect[50];
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
                collect[count] = r;
                //printf("%d\n",(int8_t)info.stats.qual.level);
                count++;
            }
            i++;
        }

		sort(collect, collect+50);
		for(i = 0; i < 50; i++) {
            fCollect[i] = (double)collect[i];
        }
	  	filter.median = median_filter(fCollect);

		p.RSSI = filter.median;

		record->push_back(p);
    }

    return 0;
    */
}

vector<PointData> planPath(CoreAPI *api){

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
    double currX = 0.0, currY=0.0, preX=0.0, preY=0.0;  //X and Y, in XY coordinate system
    double currLat=0.0, currLon=0.0, preLat=0.0, preLon=0.0; //Latitude and Longitude, in Geographic coordinate system
    double guessLon=0.0, guessLat=0.0;  //guess Target position, need to transform
    double Xa=0.0, Ya=0.0, Xb=0.0, Yb=0.0, Xt=0.0, Yt=0.0, LatA=0.0, LonA=0.0, LatB=0.0, LonB=0.0; //For CoordinateChanger
    double tol_moveDist = 0.0, err_dist=0.0;
    double moveDistance =0.0, cur_radius=0.0, pre_radius=0.0;
    vector<double> r_matrix;

    vector<double> x_matrix;
    vector<double> y_matrix;
    vector<double> lat_matrix, lon_matrix;

    // Start Searching
    vector<PointData> record = goFind(api,"./preFlight.txt", distPercent);  //main record
    vector<PointData> searchRecord = goFind(api,"./moveStraight.txt", distPercent); // each turn record
    vector<GuessPosition> guess;

    currLat = searchRecord[searchRecord.size()-1].latitude;
    currLon = searchRecord[searchRecord.size()-1].longitude;
    preLat = record[record.size()-1].latitude;
    preLon = record[record.size()-1].longitude;

    moveDistance = earth_distance(currLat, currLon, preLat, preLon, 'K') * 1000; //km to m
    tol_moveDist += moveDistance;

    printf("moveDistance: %lf\n", moveDistance);
    printf("RSSI: %lf\n", searchRecord[searchRecord.size()-1].RSSI);

    cur_radius = rssiToDist(searchRecord[searchRecord.size()-1].RSSI, searchRecord[searchRecord.size()-1].altitude);
    pre_radius = rssiToDist(record[record.size()-1].RSSI, record[record.size()-1].altitude);

    currX = pre_radius+20; //Hard Cord


    // 2D Array malloc //
    int height = out4in5(pre_radius+20);  //Hard Cord
    int width = height*2;

    // 生成一維指標陣列
    int count=0;
    int i=0, j=0;
    double **map_weight = new double*[height];
    int **map_count = new int*[height];

    // 每個指標陣列再生成整數陣列
    for(i=0; i<height; i++){
        map_count[i] = new int[width];
        map_weight[i] = new double[width];
    }
    // write
    for(i=0; i<height; i++) {
        for(j=0; j<width; j++){
            map_count[i][j] = 0;
            map_weight[i][j] = 0.0;
        }
    }


    /////////////////////
    // extern variable //
    /////////////////////

    //Add First Weight//
    double d=0.0;
    for(i=0; i<height; i++) {
        for(j=0; j<width; j++){
            d = distance(i, j, currX, currY);
            if (cur_radius >= d) {
                if (d <= cur_radius) {
                    map_weight[i][j] += d / pow((cur_radius), 2);
                    map_count[i][j] += 1;
                } else {
                    map_weight[i][j] += (cur_radius * 2 - d) / pow((cur_radius), 2);
                    map_count[i][j] += 1;
                }
            }
        }
    }

    record.insert( record.end(), searchRecord.begin(), searchRecord.end() );
    record[record.size()-1].total_moveDist = tol_moveDist;

    //Start Dynamic Path Planning
    int recount=0;
    do{
        count+=1;
        if(cur_radius<height/2 && recount==0){  //Hard Cord
            for(i=0; i<height; i++) {
                for(j=0; j<width; j++){
                    map_count[i][j] = 0;
                    map_weight[i][j] = 0.0;
                }
            }
            recount =1;
        }

        //Get current Yaw
        curYaw = getYaw(flight);
        curYaw = curYaw * -1;
        if(curYaw<0){
            curYaw = 2*_PI_ + curYaw;
        }

        flightMove(&currX, &currY, &preX, &preY, descision, moveDistance, curYaw);
        x_matrix.push_back(preX); y_matrix.push_back(preY); lat_matrix.push_back(preLat); lon_matrix.push_back(preLon);

        printf("CurrLat:%lf CurrLon:%lf, PreLat:%lf PreLon:%lf\n", currLat, currLon, preLat, preLon);
        printf("CurrX:%lf CurrY:%lf, PreX:%lf PreY:%lf\n", currX, currY, preX, preY);

        r_matrix.clear();
        rotation_matrix(currX, currY, preX, preY, r_matrix, curYaw);

        if(cur_radius>0){
            addWeight(currX, currY, preX, preY, cur_radius, pre_radius, map_weight, map_count, r_matrix, height);
            guess = predictPos(map_weight, map_count, cur_radius, currX, currY, height);  //得出最高權重的(X,Y)點
            //coordinateChanger(Xt, Yt, x_matrix, y_matrix, lat_matrix, lon_matrix);  //hen不準，不知道自己在寫什麼
        }

        descision = turnDecision(currX, currY, preX, preY, &preturn, &bool_predecision, &turnCase, cur_radius, pre_radius, map_weight, map_count, r_matrix, height);



        if(count>10){ //Hard Cord
            for(int heyo=0; heyo<guess.size(); heyo++){
                guessLon = leastSquare(guess[heyo].xt, x_matrix, lon_matrix);
                guessLat = leastSquare(guess[heyo].yt, y_matrix, lat_matrix);
                err_dist = earth_distance(0.393463, 1.988969, guessLat, guessLon, 'K') * 1000;
                printf("least square:\n Xt:%d , Yt:%d , threshold:%lf \n Lat:%lf , Lon:%lf , err_dist:%lf\n  ", guess[heyo].xt, guess[heyo].yt, guess[heyo].threshold, guessLat, guessLon, err_dist);
            }

            //record[record.size()-1].error_dist = err_dist;
            //record[record.size()-1].guessLatitude = guessLat;
            //record[record.size()-1].guessLongitude = guessLon;

            //printf("least square:\n Lat:%lf , Lon:%lf\n", guessLat, guessLon);
            x_matrix.erase(x_matrix.begin()); y_matrix.erase(y_matrix.begin()); lon_matrix.erase(lon_matrix.begin()); lat_matrix.erase(lat_matrix.begin());


            //gaussFilter(record, &predictCount);
        }

        cout<< "------------------------------------\n" <<endl;

        cout << "Decision: " << descision << " TurnCase: " << turnCase <<endl;


        ////////////////////////////////
        ///// Next out4in5 of Flight /////
        ////////////////////////////////
        distPercent = cur_radius/height;
        if(distPercent>1.0 || distPercent<0){
            distPercent = 1.0;
        }

        searchRecord.clear();
        if(descision==0){
            searchRecord = goFind(api,"./moveLeft.txt", distPercent);
        }
        else if(descision==1){
            searchRecord = goFind(api,"./moveStraight.txt", distPercent);
        }
        else if(descision==2){
            searchRecord = goFind(api,"./moveRight.txt", distPercent);
        }


        currLat = searchRecord[searchRecord.size()-1].latitude;
        currLon = searchRecord[searchRecord.size()-1].longitude;
        preLat = record[record.size()-1].latitude;
        preLon = record[record.size()-1].longitude;

        moveDistance = earth_distance(currLat, currLon, preLat, preLon, 'K') * 1000;
        tol_moveDist += moveDistance;
        searchRecord[searchRecord.size()-1].total_moveDist = tol_moveDist;

        printf("moveDistance: %lf\n", moveDistance);

        cur_radius = rssiToDist(searchRecord[searchRecord.size()-1].RSSI, searchRecord[searchRecord.size()-1].altitude);
        pre_radius = rssiToDist(record[record.size()-1].RSSI, record[record.size()-1].altitude);

        printf("CurR:%lf, PreR:\n", cur_radius, pre_radius);

        record.insert( record.end(), searchRecord.begin(), searchRecord.end() );

        cout << "Record Size:" << record.size() << " sRecord Size:" << searchRecord.size() <<endl;



    }while(cur_radius>10); //Hard Cord


    for(int i=0; i<record.size(); i++){
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

    do{
    	control(&vrc, pitch*distPercent, yaw);  //Pitch Hard Code
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
    double n=2, A= -50; //n:path-loss exponent, A:RSSI per unit
    double noise = 0.0;
    if(dist>=100){
        noise = 6 * normalDistribution();
    }
    else if(100>dist>=50){
        noise = 4 * normalDistribution();
    }
    else if(50>dist>=20){
        noise = 2 * normalDistribution();
    }
    else if(20>dist){
        noise = normalDistribution();
    }
    double rssi = A - 10*n*log10(dist) + noise;
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

void coordinateChanger(double xt, double yt, vector<double> x_matrix, vector<double> y_matrix, vector<double> lat_matrix, vector<double> lon_matrix){  //XY coordinate to Lon,Lat coordinate

    double xA=0.0, xB =0.0, yA=0.0, yB=0.0, lonA=0.0, lonB=0.0, latA=0.0, latB=0.0, lonT=0.0, latT=0.0, delta=0.0, x=0.0, y=0.0;
    xA=x_matrix[x_matrix.size()-10] - x_matrix[x_matrix.size()];
    xB=x_matrix[x_matrix.size()-5] - x_matrix[x_matrix.size()];
    yA=y_matrix[y_matrix.size()-10] - y_matrix[y_matrix.size()];
    yB=y_matrix[y_matrix.size()-5] - y_matrix[y_matrix.size()];
    latA=lat_matrix[lat_matrix.size()-10] - lat_matrix[lat_matrix.size()];
    latB=lat_matrix[lat_matrix.size()-5] - lat_matrix[lat_matrix.size()];
    lonA=lon_matrix[lon_matrix.size()-10] - lon_matrix[lon_matrix.size()];
    lonB=lon_matrix[lon_matrix.size()-5] - lon_matrix[lon_matrix.size()];
    xt = xt - x_matrix[x_matrix.size()];
    yt = yt - y_matrix[y_matrix.size()];

    delta = 1/(latA*lonB-latB*lonA);
    x = xA*xt+xB*yt;
    y = yA*xt+yB*yt;
    latT = delta*(lonB*x-lonA*y) + lat_matrix[lat_matrix.size()];
    lonT = delta*(-latB*x+latA*y) + lon_matrix[lon_matrix.size()];

    cout << "Coordinate Change:" <<endl;
    cout << "Lat:" << latT << ", Lon:" << lonT << endl;



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
    double dist=0;
    if(rssi<0){
        double n=2, A=-50; //n:path-loss exponent, A:RSSI per unit
        double log=10, exp= (A-rssi) / (10*n);
        dist = pow(log, exp);
        dist = sqrt(pow(dist,n) + pow(altitude, n));
        return dist;
    }
    else {
        double dist=-1;
        return dist;
    }
}

double normalDistribution()
{
    srand(time(NULL));
    double u = rand() / (double)RAND_MAX;
    double v = rand() / (double)RAND_MAX;
    double x = sqrt(-2 * log(u)) * cos (2 * _PI_ * v);
    return x;

}


/**

Dynamic Path Planning Function

**/


void rotation_matrix(double currX, double currY, double preX, double preY, vector<double> &r_matrix, double curYaw){

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

void addWeight(double currX, double currY, double preX, double preY, double cur_radius, double pre_radius, double** map_weight, int** map_count, vector<double> r_matrix, int height) {

    int weight_i=0;
    int weight_j=0;


    if (cur_radius - pre_radius > 0) {  //飛機遠離目標
        for (int i = currX - cur_radius - 5; i <= currX + cur_radius + 5; i++) {  //Hard Cord
            for (int j = currY; j >= currY - cur_radius + 5; j--) {
                weight_i = out4in5((i - currX) * r_matrix[0] + (j - currY) * r_matrix[1]) + out4in5(currX);
                weight_j = out4in5((i - currX) * r_matrix[2] + (j - currY) * r_matrix[3]) + out4in5(currY);
                if ( height>weight_i && weight_i>=0 && (height*2)>weight_j && weight_j>=0 ) {
                    if (map_weight[weight_i][weight_j] >= 0) {
                        double dist = distance(weight_i, weight_j, currX, currY);
                        if(cur_radius >= dist) {
                            map_weight[weight_i][weight_j] += dist / pow((cur_radius), 2);
                            map_count[weight_i][weight_j] += 1;
                        }
                        else {
                            map_weight[weight_i][weight_j] += (cur_radius * 2 - dist) / pow((cur_radius), 2);
                            map_count[weight_i][weight_j] += 1;
                        }
                    }
                }
            }
            for (int j = currY; j <= currY + cur_radius+5; j++) {  //Hard Cord
                weight_i = out4in5((i - currX) * r_matrix[0] + (j - currY) * r_matrix[1]) + out4in5(currX);
                weight_j = out4in5((i - currX) * r_matrix[2] + (j - currY) * r_matrix[3]) + out4in5(currY);
                if (height>weight_i && weight_i>=0 && (height*2)>weight_j && weight_j>=0) {
                    if (map_weight[weight_i][weight_j] >= 0) {
                        double dist = distance(weight_i, weight_j, currX, currY);
                        if(cur_radius >= dist) {
                            map_weight[weight_i][weight_j] += dist / pow((cur_radius), 2)*0.9;
                            map_count[weight_i][weight_j] += 1;
                        }
                        else {
                            map_weight[weight_i][weight_j] += (cur_radius * 2 - dist) / pow((cur_radius), 2)*0.9;
                            map_count[weight_i][weight_j] += 1;
                        }
                    }
                }
            }
        }
    } else if (cur_radius - pre_radius < 0) { //飛機靠近目標
        for (int i = currX - cur_radius - 5; i <= currX + cur_radius + 5; i++) { //Hard Cord
            for (int j = currY; j <= currY + cur_radius + 5; j++) {
                weight_i = out4in5((i - currX) * r_matrix[0] + (j - currY) * r_matrix[1]) + out4in5(currX);
                weight_j = out4in5((i - currX) * r_matrix[2] + (j - currY) * r_matrix[3]) + out4in5(currY);
                if (height>weight_i && weight_i>=0 && (height*2)>weight_j && weight_j>=0) {
                    if (map_weight[weight_i][weight_j] >= 0) {
                        double dist = distance(weight_i, weight_j, currX, currY);
                        if(cur_radius >= dist) {
                            map_weight[weight_i][weight_j] += dist / pow((cur_radius), 2);
                            map_count[weight_i][weight_j] += 1;
                        }
                        else {
                            map_weight[weight_i][weight_j] += (cur_radius * 2 - dist) / pow((cur_radius), 2);
                            map_count[weight_i][weight_j] += 1;
                        }
                    }
                }
            }
            for (int j = currY; j >= currY - cur_radius-5; j--) { //Hard Cord
                weight_i = out4in5((i - currX) * r_matrix[0] + (j - currY) * r_matrix[1]) + out4in5(currX);
                weight_j = out4in5((i - currX) * r_matrix[2] + (j - currY) * r_matrix[3]) + out4in5(currY);
                if (height>weight_i && weight_i>=0 && (height*2)>weight_j && weight_j>=0) {
                    if (map_weight[weight_i][weight_j] >= 0) {
                        double dist = distance(weight_i, weight_j, currX, currY);
                        if(cur_radius >= dist) {
                            map_weight[weight_i][weight_j] += dist / pow((cur_radius), 2)*0.9;
                            map_count[weight_i][weight_j] += 1;
                        }
                        else {
                            map_weight[weight_i][weight_j] += (cur_radius * 2 - dist) / pow((cur_radius), 2)*0.9;
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

int turnDecision(double currX, double currY, double preX, double preY, int* preturn, int* bool_predecision, int* turnCase, double cur_radius, double pre_radius, double** map_weight, int** map_count, vector<double> r_matrix, int height) {

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




        for (int i = currX - cur_radius - 5; i <= currX + cur_radius + 5; i++) {
            for (int j = currY; j <= currY + cur_radius + 5; j++) {
                weight_i = out4in5((i - currX) * r_matrix[0] + (j - currY) * r_matrix[1]) + out4in5(currX);
                weight_j = out4in5((i - currX) * r_matrix[2] + (j - currY) * r_matrix[3]) + out4in5(currY);
                if (height>weight_i && weight_i>=0 && (height*2)>weight_j && weight_j>=0) {
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

            for (int j = currY; j >= currY - cur_radius - 5; j--) {
                weight_i = out4in5((i - currX) * r_matrix[0] + (j - currY) * r_matrix[1]) + out4in5(currX);
                weight_j = out4in5((i - currX) * r_matrix[2] + (j - currY) * r_matrix[3]) + out4in5(currY);
                if (height>weight_i && weight_i>=0 && (height*2)>weight_j && weight_j>=0) {
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


        cout << "turn Flag:3" << endl;



    srand(time(NULL));
    if (cur_radius <= pre_radius) {
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

}

double median_filter(double* rssi)
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

vector<GuessPosition> predictPos(double** map_weight, int** map_count, double currR, double currX, double currY, int height)  //predict positon in XY coordination
{
    double large[4] = {0.0, 0.0, 0.0, 0.0};
    double predictDistance = 0.0, distError = 0.0, threshold = 100.0;
    int errorCount=0;
    int i=0, j=0;

    vector<GuessPosition> guess;
    GuessPosition g;


     //找到目前最大的權重
     for(i=0; i<height; i++) {
        for(j=0; j<(height*2); j++){
            if (map_weight[i][j] > 0 && map_count[i][j] > 0) {
                if (map_weight[i][j] / map_count[i][j] > large[0]) {
                    large[0] = map_weight[i][j] / map_count[i][j];
                }
            }
        }
    }

    //找出dist_error 的 threshold
    for (i=0; i<height; i++) {
        for (j=0; j<(height*2); j++) {
            if (map_weight[i][j] > 0 && map_count[i][j] > 0) {
                if ( map_weight[i][j] / map_count[i][j] > large[0] * 0.95) {  //*0.9是避免RSSI誤差
                    predictDistance = distance(i, j, currX, currY);
                    distError = abs(predictDistance-currR);
                    if(distError<=threshold){
                        //*Xt = i;
                        //*Yt = j;
                        threshold = distError;
                    }

                }
                else if(map_weight[i][j] / map_count[i][j] == large[0]){
                    g.xt = i;
                    g.yt = j;
                    guess.push_back(g);
                }
            }
        }
    }


    for (i=0; i<height; i++) {
        for (j=0; j<(height*2); j++) {
            if (map_weight[i][j] > 0 && map_count[i][j] > 0) {
                if (map_weight[i][j] / map_count[i][j] >= large[0] * 0.95) {  //*0.9是避免RSSI誤差

                    large[3] += 1;
                    predictDistance = distance(i, j, currX, currY);
                    distError = abs(predictDistance-currR);
                    if(distError*0.95<=threshold){  //Hard Cord
                        errorCount++;
                        g.xt = i;
                        g.yt = j;
                        g.threshold = distError;
                        guess.push_back(g);
                    }

                }
            }
        }
    }

    return guess;
}


double leastSquare(double xyt, vector<double> &xy_matrix, vector<double> &latlon_matrix)
{
    int i, n=xy_matrix.size();
    double a,b;
    double y=0.0;


    double xsum=0,x2sum=0,ysum=0,xysum=0;                //variables for sums/sigma of xi,yi,xi^2,xiyi etc
    for (i=0;i<n;i++)
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

void gaussFilter(vector<PointData> record, int *predictCount){

    double uLat=0.0, uLon=0.0, sdLat=0.0, sdLon=0.0; //Standard Deviation
    double guessLon=0.0, guessLat=0.0;
    int guessCount=0, i=0;
    for(i=record.size()-8; i<record.size(); i++){
        uLat += record[i].guessLatitude;
        uLon += record[i].guessLongitude;
    }

    uLat = uLat/8; //Hard Cord
    uLon = uLon/8; //Hard Cord
    cout << "uLat:" << uLat << ", uLon:"<< uLon << endl;

    for(i=record.size()-8; i<record.size(); i++){
        sdLat += pow((record[i].guessLatitude - uLat), 2);
        sdLon += pow((record[i].guessLongitude - uLon), 2);
    }

    cout << "sdLat:" << sdLat << ", sdLon:"<<sdLon << endl;
    sdLat = sqrt(sdLat/8);  //Hard Cord
    sdLon = sqrt(sdLon/8);  //Hard Cord
    cout << "sdLat:" << sdLat << ", sdLon:"<<sdLon << endl;

    for(i=record.size()-8; i<record.size(); i++){
        if((uLat-sdLat)<=record[i].guessLatitude<=(uLat+sdLat)){
            if((uLon-sdLon)<=record[i].guessLongitude<=(uLon+sdLon)){
                guessLat += record[i].guessLatitude;
                guessLon += record[i].guessLongitude;
                guessCount++;
            }
        }
    }
    guessLat = guessLat/guessCount;
    guessLon = guessLon/guessCount;
    record[i].guessLatitude = guessLat;
    record[i].guessLongitude = guessLon;
    printf("Gauss Guess:\n Lat:%lf , Lon:%lf\n", guessLat, guessLon);

    if(record[i-1].guessLatitude = record[i].guessLatitude && record[i-1].guessLongitude==record[i].guessLongitude){
        *predictCount += 1;
    }
    else{
        *predictCount==0;
    }
}

double kalman_filter(double* rssi){

    double X=0.0, P=3.0, Q=0.0001, K=0.0, R=1.0, I=1.0;

    for(int i=0; i<sizeof(*rssi); i++){
        X=X;
        P = P+Q;
        K = P / (P + R);
        X = X + K * (rssi[i] - X);
        P = (I - K) * P;
    }

    return X;

}

int out4in5(double x){
    x = x+0.5;
    x = floor(x);
    return (int)x;
}
