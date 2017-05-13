var currentX=500;
var currentY=500;
var targetX = 683;
var targetY = 483;
var prepointX=500; 
var prepointY=550;
var turnCase=0;
var count = 0;
var radius = Math.floor(Math.pow(Math.pow((currentX-targetX),2) + Math.pow((currentY-targetY),2),0.5));

var step=0;
var range = [];

var map_weight = [];  //Create a Map with Weight
for (var i=0; i<2000; i++){    
    map_weight[i] = []
    for(var j=0; j<1200; j++){
        map_weight[i][j] = 0;
    }
}

var map_count = [];  //Create a Map with Weight
for (var i=0; i<2000; i++){    
    map_count[i] = []
    for(var j=0; j<1200; j++){
        map_count[i][j] = 0;
    }
}



/*
for (var i=currentX-radius; i<currentX+radius; i++){ 
    for(var j=currentY; j>currentY-radius; j++){
        var radius_i = i-500;
        var radius_j = j-500;  //此兩個變數用來判斷該點是否在圓內
        
        if(Math.pow(radius,2)>=(Math.pow(radius_i,2) + Math.pow(radius_j,2))){
            map_weight[i][j] = 1;
        }
    }
}
*/

function setup() {
    createCanvas(2000, 1200);
    
    
    var radius = distance(currentX, currentY, targetX, targetY);
    
    
    background(255);
    stroke(0);
    drawCircle(currentX,currentY,radius);
    line(0, 500, width, 500);
    line(500, 0, 500, height);
    
    
    stroke(22, 151, 131); //RGB&Opacity
    for(var x=targetX-3;x<targetX+4;x++){
        for(var y=targetY-3; y<targetY+4; y++){
            point(x,y);
        }
    }
    
    stroke(102, 41, 31); //RGB&Opacity
    for(var x=prepointX-3;x<prepointX+4;x++){
        for(var y=prepointY-3; y<prepointY+4; y++){
            point(x,y);
        }
    }
    
    var cur_radius = distance(currentX, currentY, targetX, targetY);
    var pre_radius = distance(prepointX, prepointY, targetX, targetY);
    addWeight(currentX, currentY, prepointX, prepointY, cur_radius, pre_radius);
    
}
    

function draw() {
    
    
    drawpoint(currentX, currentY, 255);
    drawCircle(currentX,currentY,radius);
    stroke(22, 151, 131); //RGB&Opacity
    for(var x=targetX-3;x<targetX+4;x++){
        for(var y=targetY-3; y<targetY+4; y++){
            point(x,y);
        }
    }
}


function mousePressed() {
    count++;
    var cur_radius = distance(currentX, currentY, targetX, targetY);
    var pre_radius = distance(prepointX, prepointY, targetX, targetY);
    var descision = turnDecision(currentX, currentY, prepointX, prepointY);
    flightMove(currentX, currentY, turnCase, descision);
    cur_radius = distance(currentX, currentY, targetX, targetY);
    pre_radius = distance(prepointX, prepointY, targetX, targetY);
    addWeight(currentX, currentY, prepointX, prepointY, cur_radius, pre_radius);
    
    redraw(); 
    console.log(count);
    
    if(count==30){
        var large=[0,0,0];
        background(255);
        for (var i=0; i<2000; i++){    
            for(var j=0; j<1200; j++){
                if(map_weight[i][j]>0 && map_count[i][j]>0){
                    map_weight[i][j] = map_weight[i][j]/map_count[i][j];
                    if(map_weight[i][j]>large[0]){
                        large[0] = map_weight[i][j];
                        large[1] = i;
                        large[2] = j;
                    }
                    stroke(102, 211, 131, Math.floor(map_weight[i][j]*100) ); //RGB&Opacity
                    point(i,j);
                }
            }
        }
        
        console.log("Largest: %s, X: %s, Y: %s", large[0], large[1], large[2]);
    }
    
}


function drawpoint(pointX, pointY, opacity){
    stroke(202, 111, 31, opacity); //RGB&Opacity
    for(var x=pointX-3;x<pointX+4;x++){
        for(var y=pointY-3; y<pointY+4; y++){
            point(x,y);
        }
    }
}

function drawCircle(x,y,radius){
    stroke(0,0,0);
    fill(0,0);  //fill(0,50);
    ellipse(x,y,radius*2);
}

function distance(x1, y1, x2, y2 ){
    var d = Math.floor(Math.pow(Math.pow((x1-x2),2) + Math.pow((y1-y2),2),0.5));
    return d;
}


function addWeight(currX, currY, preX, preY, cur_radius, pre_radius){
    
    var r_matrix = retation_matrix(currX, currY, preX, preY) ;  //旋轉矩陣

    
    if(cur_radius-pre_radius>0){  //半徑變大，表示遠離
        for(i=currX-cur_radius; i<=currX+cur_radius; i++){ 
            for(j=currY; j<=currY+cur_radius; j++){
                var weight_i = Math.floor((i-currX)*r_matrix[0] + (j-currY)*r_matrix[1]) + currX;
                var weight_j = Math.floor((i-currX)*r_matrix[2] + (j-currY)*r_matrix[3]) + currY;
                if(weight_i>=0 && weight_j>=0){  //避免rotate後，weight_i&j為負數型態
                    if(map_weight[weight_i][weight_j]>=0){
                        var dist = distance(weight_i, weight_j, currX, currY);
                        if(cur_radius>=dist){
                            map_weight[weight_i][weight_j] += Math.pow(dist/cur_radius, 2);
                            map_count[weight_i][weight_j] +=1;
                            stroke(102, 211, 131, Math.floor(map_weight[weight_i][weight_j]*100/map_count[weight_i][weight_j])); //RGB&Opacity
                            point(weight_i,weight_j);  
                        }
                    }
                }
            }
        }
    }
    else if(cur_radius-pre_radius<0){  //半徑變小，表示靠近
        for(i=currX-radius; i<=currX+radius; i++){ 
            for(j=currY; j>=currY-radius; j--){
                var weight_i = Math.floor((i-currX)*r_matrix[0] + (j-currY)*r_matrix[1]) + currX;
                var weight_j = Math.floor((i-currX)*r_matrix[2] + (j-currY)*r_matrix[3]) + currY;
                if(weight_i>=0 && weight_j>=0){  //避免rotate後，weight_i&j為負數型態
                    if(map_weight[weight_i][weight_j]>=0){
                        var dist = distance(weight_i, weight_j, currX, currY);
                        if(cur_radius>=dist){
                            map_weight[weight_i][weight_j] += Math.pow(dist/cur_radius, 2)
                            map_count[weight_i][weight_j] +=1;
                            stroke(102, 211, 131, Math.floor(map_weight[weight_i][weight_j]*100/map_count[weight_i][weight_j])); //RGB&Opacity
                            point(weight_i,weight_j);
                        }
                    }
                }
            }
        }
    } 
    
    
    
}




function drawMap(){
    
    for (i = 0; i < 1000; i++) {  //Draw Map Coverage
        for(j = 0; j< 600; j++){
            if(map_weight[i][j]>0){
                stroke(102, 211, 131, map_weight[i][j]*5); //RGB&Opacity
                point(i,j);
            }        
        }
    }
}

function turnDecision(currX, currY, preX, preY){
    
    var r_matrix = retation_matrix(currX, currY, preX, preY) ;  //旋轉矩陣
    var turn_matrix = [0,0,0];
    
    var cur_radius = distance(currX, currY, targetX, targetY);
    var pre_radius = distance(preX, preY, targetX, targetY);
    
    if(currX-preX==0){
        var slope = 0; //行進路線斜率
        var split_line1 = (slope - Math.tan(Math.PI/3)) / (Math.tan(Math.PI/3)*slope +1);
        var split_line2 = (slope + Math.tan(Math.PI/3)) / (1 - Math.tan(Math.PI/3)*slope);    
    }
    else{
        var slope = (currY-preY) / (currX - preX); //行進路線斜率
        var split_line1 = (slope - Math.tan(Math.PI/6)) / (Math.tan(Math.PI/6)*slope +1);
        var split_line2 = (slope + Math.tan(Math.PI/6)) / (1 - Math.tan(Math.PI/6)*slope);
    }
    
    if(split_line2>split_line1){  //把斜率比較大的當成line1
            split_line1 = split_line1 + split_line2;
            split_line2 = split_line1 - split_line2;
            split_line1 = split_line1 - split_line2;
    }
    var constant1 = calConstant(currX, currY, split_line1);
    var constant2 = calConstant(currX, currY, split_line2);  
    
    
    console.log("currX: %s, currY: %s, preX: %s, preY: %s", currX, currY, preX, preY);
    
    
    if(currX-preX>0 && currY-preY<0){  //行進方向為↗
        turnCase = 1;
    }        
    else if(currX-preX<0 && currY-preY<0){  //行進方向為↖
        turnCase = 2;  
    }           
    else if(currX-preX<0 && currY-preY>0){  //行進方向為↙
        turnCase = 3;   
    }
    else if(currX-preX>0 && currY-preY>0){  //行進方向為↘       
        turnCase = 4; 
    }
    else if(currX-preX==0 && currY-preY<0){  //行進方向為↑
        turnCase = 5;
    }
    else if(currX-preX<0 && currY-preY==0){  //行進方向為←
        turnCase = 6;
    }
    else if(currX-preX==0 && currY-preY>0){  //行進方向為↓
        turnCase = 7;
    }
    else if(currX-preX>0 && currY-preY==0){  //行進方向為→
        turnCase = 8;
    }
    
    console.log("TurnCase: ", turnCase);
    if(cur_radius-pre_radius<=0){  //半徑變小，表示靠近
        for(i=currX-cur_radius; i<=currX+cur_radius; i++){ 
            for(j=currY; j>=currY-cur_radius; j--){
            var weight_i = Math.floor((i-currX)*r_matrix[0] + (j-currY)*r_matrix[1]) + currX;
            var weight_j = Math.floor((i-currX)*r_matrix[2] + (j-currY)*r_matrix[3]) + currY;
            if(weight_i>=0 && weight_j>=0){
                if(map_weight[weight_i][weight_j]>0){
                    
                    if(Math.pow(weight_i-currX,2)+Math.pow(weight_j-currY,2) <= cur_radius){
                        switch(turnCase) {
                            case 1:
                                if(weight_i*split_line2+constant2>=weight_j){
                                    turn_matrix[0] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line1+constant1>weight_j && weight_i*split_line2+constant2<weight_j){
                                    turn_matrix[1] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line1+constant1<=weight_j){
                                    turn_matrix[2] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                break;

                            case 2:
                                if(weight_i*split_line2+constant2<=weight_j){
                                    turn_matrix[0] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line1+constant1<weight_j && weight_i*split_line2+constant2>weight_j){
                                    turn_matrix[1] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line1+constant1>=weight_j){
                                    turn_matrix[2] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                break;

                            case 3:
                                if(weight_i*split_line2+constant2<=weight_j){
                                    turn_matrix[0] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line1+constant1<weight_j && weight_i*split_line2+constant2>weight_j){
                                    turn_matrix[1] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line1+constant1>=weight_j){
                                    turn_matrix[2] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                break;

                            case 4:
                                if(weight_i*split_line2+constant2>=weight_j){
                                    turn_matrix[0] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line1+constant1>weight_j && weight_i*split_line2+constant2<weight_j){
                                    turn_matrix[1] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line1+constant1<=weight_j){
                                    turn_matrix[2] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                break;

                            case 5:
                                if(weight_i*split_line1+constant1<=weight_j){
                                    turn_matrix[0] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line1+constant1>weight_j && weight_i*split_line2+constant2>weight_j){
                                    turn_matrix[1] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line2+constant2<=weight_j){
                                    turn_matrix[2] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                break;

                            case 6:
                                if(weight_i*split_line2+constant2<=weight_j){
                                    turn_matrix[0] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line1+constant1<weight_j && weight_i*split_line2+constant2>weight_j){
                                    turn_matrix[1] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line1+constant1>=weight_j){
                                    turn_matrix[2] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                break;

                            case 7:
                                if(weight_i*split_line1+constant1>=weight_j){
                                    turn_matrix[0] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line1+constant1<weight_j && weight_i*split_line2+constant2<weight_j){
                                    turn_matrix[1] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line2+constant2>=weight_j){
                                    turn_matrix[2] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                break;

                            case 8:
                                if(weight_i*split_line2+constant2>=weight_j){
                                    turn_matrix[0] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line1+constant1>weight_j && weight_i*split_line2+constant2<weight_j){
                                    turn_matrix[1] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line1+constant1<=weight_j){
                                    turn_matrix[2] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                break;

                            default:
                                console.log("Error");
                            }   
                        }
                    }
                }
            }
        }
    }
    else if(cur_radius-pre_radius>0){  //半徑變大，表示遠離
        for(i=currX-cur_radius; i<=currX+cur_radius; i++){ 
            for(j=currY; j<=currY+cur_radius; j++){
            var weight_i = Math.floor((i-currX)*r_matrix[0] + (j-currY)*r_matrix[1]) + currX;
            var weight_j = Math.floor((i-currX)*r_matrix[2] + (j-currY)*r_matrix[3]) + currY;
            if(weight_i>=0 && weight_j>=0){
                if(map_weight[weight_i][weight_j]>0){
                    if(Math.pow(weight_i-currX,2)+Math.pow(weight_j-currY,2) <= cur_radius){
                        switch(turnCase) {
                            case 1:
                                if(weight_i*split_line2+constant2<=weight_j){
                                    turn_matrix[2] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line1+constant1<weight_j && weight_i*split_line2+constant2>weight_j){
                                    turn_matrix[1] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line1+constant1>=weight_j){
                                    turn_matrix[0] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                break;

                            case 2:
                                if(weight_i*split_line2+constant2>=weight_j){
                                    turn_matrix[2] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line1+constant1>weight_j && weight_i*split_line2+constant2<weight_j){
                                    turn_matrix[1] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line1+constant1<=weight_j){
                                    turn_matrix[0] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                break;

                            case 3:
                                if(weight_i*split_line2+constant2>=weight_j){
                                    turn_matrix[2] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line1+constant1>weight_j && weight_i*split_line2+constant2<weight_j){
                                    turn_matrix[1] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line1+constant1<=weight_j){
                                    turn_matrix[0] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                break;

                            case 4:
                                if(weight_i*split_line2+constant2<=weight_j){
                                    turn_matrix[2] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line1+constant1<weight_j && weight_i*split_line2+constant2>weight_j){
                                    turn_matrix[1] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line1+constant1>=weight_j){
                                    turn_matrix[0] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                break;

                            case 5:
                                if(weight_i*split_line1+constant1>=weight_j){
                                    turn_matrix[2] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line1+constant1<weight_j && weight_i*split_line2+constant2<weight_j){
                                    turn_matrix[1] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line2+constant2>=weight_j){
                                    turn_matrix[0] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                break;

                            case 6:
                                if(weight_i*split_line2+constant2>=weight_j){
                                    turn_matrix[2] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line1+constant1>weight_j && weight_i*split_line2+constant2<weight_j){
                                    turn_matrix[1] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line1+constant1<=weight_j){
                                    turn_matrix[0] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                break;

                            case 7:
                                if(weight_i*split_line1+constant1<=weight_j){
                                    turn_matrix[2] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line1+constant1>weight_j && weight_i*split_line2+constant2>weight_j){
                                    turn_matrix[1] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line2+constant2<=weight_j){
                                    turn_matrix[0] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                break;

                            case 8:
                                if(weight_i*split_line2+constant2<=weight_j){
                                    turn_matrix[2] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line1+constant1<weight_j && weight_i*split_line2+constant2>weight_j){
                                    turn_matrix[1] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                if(weight_i*split_line1+constant1>=weight_j){
                                    turn_matrix[0] += map_weight[weight_i][weight_j]/map_count[weight_i][weight_j];
                                }
                                break;

                            default:
                                console.log("Error");
                            }     
                        }
                    }
                }
            }
        }
    }
    console.log("TrunMatrix: %s, %s, %s", turn_matrix[0],turn_matrix[1],turn_matrix[2]);
    
    
    if(turn_matrix[0]>=turn_matrix[1]){
        if(turn_matrix[0]>turn_matrix[2]){
            return 0;
        }
        else if(turn_matrix[0]==turn_matrix[2]){
            var rand = Math.floor(Math.random()*2)
            if(rand==0){
                return 0;
            }
            else{
                return 2;
            }
        }
        else{
            return 2;
        }
    }
    else if(turn_matrix[2]>=turn_matrix[1]){
        if(turn_matrix[2]>turn_matrix[0]){
            return 2;
        }
        else if(turn_matrix[0]==turn_matrix[2]){
            var rand = Math.floor(Math.random()*2)
            if(rand==0){
                return 2;
            }
            else{
                return 0;
            }
        }
        else{
            return 0;
        }
    }
    else{
        return 1;
    }
        
    
    
    
}

function retation_matrix(currX, currY, preX, preY){
    var degree = Math.PI/4; //  π/4 = 45°
    var deltaX = currX - preX;
    var deltaY = currY - preY;
    
    if(deltaX!=0 && deltaY!=0){
        if(deltaX>0 && deltaY<0){ // ↗ °
            degree = degree*7;
        }
        else if(deltaX>0 && deltaY>0){ // ↘ °
            degree = degree*5;
        }
        else if(deltaX<0 && deltaY<0){  // ↖ °
            degree = degree*1;
        }
        else if(deltaX<0 && deltaY>0){  // ↙ °
            degree = degree*3;
        }
    }
    
    else if(deltaX==0 && deltaY!=0){
        if(deltaY<0){  // ↑ °
            degree = degree*0;
        }
        else if(deltaY>0){  // ↓ °
            degree = degree*4;
        }
        
    }
    else if(deltaX!=0 && deltaY==0){
        if(deltaX>0){  // → °
            degree = degree*6;
        }
        else if(deltaX<0){  // ← °
            degree = degree*2;
        }
    }
    console.log("Degree: %s", 180*degree/Math.PI);
    var r_matrix = [Math.cos(degree), Math.sin(degree), -Math.sin(degree), Math.cos(degree)] ;  //旋轉矩陣
    
    return r_matrix;
}

function calConstant(x, y, m){  //計算直線常數
    //假設直線方程式為 y = mx+b 
    var b = y - (m*x);
    return b;
}

function flightMove(currX, currY, turnCases, descision){
    
    
    prepointX = currX;
    prepointY = currY;
    console.log("Descision: %s", descision)
    switch(Math.abs(turnCases)){
        case 1:
            if(descision==0){
                currentX += 0; 
                currentY += -20;
            }
            else if(descision==1){
                currentX += Math.floor(Math.pow(200,0.5)); 
                currentY += -Math.floor(Math.pow(200,0.5));
            }
            else if(descision==2){
                currentX += 20; 
                currentY += 0;
            }
            break;
        case 2:
            if(descision==0){
                currentX += -20; 
                currentY += 0;
            }
            else if(descision==1){
                currentX += -Math.floor(Math.pow(200,0.5)); 
                currentY += -Math.floor(Math.pow(200,0.5));
            }
            else if(descision==2){
                currentX += 0; 
                currentY += -20;
            }
            break;
        case 3:
            if(descision==0){
                currentX += 0; 
                currentY += 20;
            }
            else if(descision==1){
                currentX += -Math.floor(Math.pow(200,0.5)); 
                currentY += Math.floor(Math.pow(200,0.5));
            }
            else if(descision==2){
                currentX += -20; 
                currentY += 0;
            }
            break;
        case 4:
            if(descision==0){
                currentX += 20; 
                currentY += 0;
            }
            else if(descision==1){
                currentX += Math.floor(Math.pow(200,0.5)); 
                currentY += Math.floor(Math.pow(200,0.5));
            }
            else if(descision==2){
                currentX += 0; 
                currentY += 20;
            }
            break;
        case 5:
            if(descision==0){
                currentX += -Math.floor(Math.pow(200,0.5)); 
                currentY += -Math.floor(Math.pow(200,0.5));
            }
            else if(descision==1){
                currentX += 0; 
                currentY += -20;
            }
            else if(descision==2){
                currentX += Math.floor(Math.pow(200,0.5)); 
                currentY += -Math.floor(Math.pow(200,0.5));
            }
            break;
        case 6:
            if(descision==0){
                currentX += -Math.floor(Math.pow(200,0.5)); 
                currentY += Math.floor(Math.pow(200,0.5));
            }
            else if(descision==1){
                currentX += -20; 
                currentY += 0;
            }
            else if(descision==2){
                currentX += -Math.floor(Math.pow(200,0.5)); 
                currentY += -Math.floor(Math.pow(200,0.5));
            }
            break;
        case 7:
            if(descision==0){
                currentX += Math.floor(Math.pow(200,0.5)); 
                currentY += Math.floor(Math.pow(200,0.5));
            }
            else if(descision==1){
                currentX += 0; 
                currentY += 20;
            }
            else if(descision==2){
                currentX += -Math.floor(Math.pow(200,0.5)); 
                currentY += Math.floor(Math.pow(200,0.5));
            }
            break;
        case 8:
            if(descision==0){
                currentX += Math.floor(Math.pow(200,0.5)); 
                currentY += -Math.floor(Math.pow(200,0.5));
            }
            else if(descision==1){
                currentX += 20; 
                currentY += 0;
            }
            else if(descision==2){
                currentX += Math.floor(Math.pow(200,0.5)); 
                currentY += Math.floor(Math.pow(200,0.5));
            }
            break;
        default:
            console.log("Error");
    }
    
}


function createArray(length) {
    var arr = new Array(length || 0),
        i = length;

    if (arguments.length > 1) {
        var args = Array.prototype.slice.call(arguments, 1);
        while(i--) arr[length-1 - i] = createArray.apply(this, args);
    }

    return arr;
}




