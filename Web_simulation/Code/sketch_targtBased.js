// 圓周率
var _PI_ = 3.14159265358979323846;

//飛機位置
var currentX = 500,
    currentY = 500,
    prepointX = 500,
    prepointY = 550;
var startX = currentX,
    startY = currentY;

//目標位置
var targetX = 350,
    targetY = 350;

var radius = 0;
var pre_radius = 0;

var degree = _PI_ / 4; //  π/4 = 45°
var turnCase = 0;
var step_count = 0; //該次飛行的第幾步
var re_calculate_weight = 0; //重新計算權重
var re_calculate_weight_dist_more_100 = 0;
var preturn = 1;
var bool_predecision = 0; //上次走的方向是正確或錯誤，0表正確
var restartbool = 0;
var restart_count = 0; //計算單點總共重新幾次，0表第一次
var accumulation = 0;

var arrPredictTarX = [],
    arrPredictTarY = [],
    predictTarX = 0,
    predictTarY = 0,
    acc_move_true = -3;



var move_distance = 20;


var rssi_filter = [];
var g_rssiFiltered = [];


var speed; //單位m/s
var tmp_timer = 0;
var timer = []; //單位second

var map_weight = []; //Create a Map with Weight
var map_count = [];


var map_width = 0;

var map_count_all = 0;

for (var i = 0; i < 2000; i++) {
    map_weight[i] = [];
    map_count[i] = [];
    for (var j = 0; j < 1200; j++) {
        map_weight[i][j] = 0;
        map_count[i][j] = 0;
    }
}


var guess_error = [];
var ge = "Guess error: ";

var total_distance = [];
var td = "Total Distance: ";

//Each Filter Rssi
var g_all = [];
var m_all = [];
var k_all = [];
var d_all = [];
var loczlization_error = [];
var g = "Gauss Filter: ";
var m = "Median Filter: ";
var k = "Kalman Filter: ";
var d = "RSSI: ";
var collect_all_count = [];

var x="", y="";


for (var i = 0; i < 100; i++) {
    guess_error[i] = 0;
    total_distance[i] = 0;
    g_all[i] = 0;
    m_all[i] = 0;
    k_all[i] = 0;
    d_all[i] = 0;
    collect_all_count[i] = 0
    loczlization_error[i] = 0;

}

function setup() {
    createCanvas(2000, 1200);


    var gg = "",
        gg_d = "";
    var kk = "",
        kk_d = "";
    var mm = "",
        mm_d = "";
    var m_rssi_filter = [];


    for (var i = 0; i < 100; i++) {
        var fakeRssi = getFakeRSSI(currentX, currentY, true);
        rssi_filter.push(fakeRssi);
        m_rssi_filter.push(fakeRssi);
        g_rssiFiltered.push(gaussFilter(rssi_filter));
    }

    var k_rssiFiltered = kalmanFilter(g_rssiFiltered);
    g_rssiFiltered = [];
    radius = rssiToDist(k_rssiFiltered);




    rssi_filter = [];


    map_width = radius * 1.2; //Local matrix width
    //console.log(radius);
    var true_radius = distance(currentX, currentY, targetX, targetY);


    background(255);
    stroke(0);
    drawCircle(currentX, currentY, radius);
    line(0, 500, width, 500);
    line(500, 0, 500, height);


    /*
    //Draw Loacl Matrix
    stroke(123, 51, 31); //RGB
    for (var x = -1; x <= 1; x++) {
        line(currentX - map_width + x, currentY + x, currentX - map_width + x, currentY - map_width + x);
        line(currentX + map_width + x, currentY + x, currentX + map_width + x, currentY - map_width + x);
        line(currentX - map_width + x, currentY - map_width + x, currentX + map_width + x, currentY - map_width + x);
    }

    */






    //Previous Point
    stroke(102, 41, 31); //RGB&Opacity
    for (var x = currentX - 2; x < currentX + 3; x++) {
        for (var y = currentY - 2; y < currentY + 3; y++) {
            point(x, y);
        }
    }

    //Previous Point
    stroke(102, 41, 31); //RGB&Opacity
    for (var x = targetX - 2; x < targetX + 3; x++) {
        for (var y = targetY - 2; y < targetY + 3; y++) {
            point(x, y);
        }
    }

    addWeight(currentX, currentY, prepointX, prepointY, radius, pre_radius);
    //linear_addWeight(currentX, currentY, prepointX, prepointY, radius, pre_radius, k_rssiFiltered);
    pre_radius = radius;

}


function draw() {
    //Draw
}


function mousePressed() {
    
    while (targetX <= 650) {

        if (step_count < 100) {
        

            var large = large = [0, 0, 0, 0];
            predictPosition(large);
            
            if (!(isNaN(large[1]) && isNaN(large[2]))) {
                arrPredictTarX.push(large[1]);
                arrPredictTarY.push(large[2]);
            }
            if (arrPredictTarX.length > 10) {
                arrPredictTarX.shift();
                arrPredictTarY.shift();
            }

            if((predictTarX==0&&predictTarY==0)||step_count%5==0){
                predictTarX = gaussFilter(arrPredictTarX);
                predictTarY = gaussFilter(arrPredictTarY);
            }
    
            x = x.concat(currentX);
            x = x.concat(", ");
            y = y.concat(currentY);
            y = y.concat(", ");


            //console.log(x);
            //console.log(y);

            

    
            /*
            //Previous Point
            stroke(52, 241, 131); //RGB&Opacity
            for (var i = predictTarX - 2; i < predictTarX + 3; i++) {
                for (var j = predictTarY - 2; j < predictTarY + 3; j++) {
                    point(i, j);
                }
            }
            */
           
            


            try {
                var descision = tar_turnDecision(currentX, currentY, prepointX, prepointY, predictTarX, predictTarY);

            } catch (e) {
                console.log(e) // 把例外物件傳給錯誤處理器
            }



            if (preturn == 1) {
                preturn = descision;
            }

            //console.log(radius);


            var move_distance = 20;
            if (radius / (map_width / 1.2) <= 0.5) {
                move_distance = 20 * (radius / (map_width / 1.2)) * 2;
            }



            for (var i = 0; i < 100; i++) {
                var fakeRssi = getFakeRSSI(currentX, currentY, true);
                rssi_filter.push(fakeRssi);
                g_rssiFiltered.push(gaussFilter(rssi_filter));
            }
            var rssiFiltered = kalmanFilter(g_rssiFiltered);

            radius = rssiToDist(rssiFiltered);




            flightMove(currentX, currentY, turnCase, descision, move_distance);
            //twoleaf_FlightMove(currentX, currentY, turnCase, descision, move_distance);
            drawpoint(currentX, currentY, 100);

            try {
                addWeight(currentX, currentY, prepointX, prepointY, radius, pre_radius);
                //linear_addWeight(currentX, currentY, prepointX, prepointY, radius, pre_radius, rssiFiltered);
            } catch (e) {

                console.log(e) // 把例外物件傳給錯誤處理器
            }





            cleanMap();

            pre_radius = radius;

            var dist_error = Math.pow(Math.pow((large[1] - targetX), 2) + Math.pow((large[2] - targetY), 2), 0.5);

            try {
                if (dist_error < 5) {
                    loczlization_error[step_count] += 1;
                }
            } catch (e) {
                console.log(e) // 把例外物件傳給錯誤處理器
            }



            record_each100_point(dist_error, move_distance);
            g_rssiFiltered = [];
            rssi_filter = [];

            if (restart_count == 100 && step_count == 99) {
                console.log(accumulation / (step_count + 1));
            }



            //Show Result
            if (step_count == 99 && restart_count == 100 && targetX == 650) {

                for (var i = 0; i < g_all.length; i++) {
                    ge = ge.concat((guess_error[i] / collect_all_count[i]).toFixed(7));
                    ge = ge.concat(" ");
                    td = td.concat((total_distance[i] / collect_all_count[i]).toFixed(7));
                    td = td.concat(" ");
                    k = k.concat((k_all[i] / collect_all_count[i]).toFixed(7));
                    k = k.concat(" ");
                    d = d.concat((d_all[i] / collect_all_count[i]).toFixed(7));
                    d = d.concat(" ");
                }
                console.log(ge);
                console.log(td);
                console.log(k);
                console.log(d);
                console.log(loczlization_error);
                //break;
            }

            step_count++;
            //console.log(step_count);
    
        } else {
            step_count = 0;
            restart();
            setup();
            console.log("restart_count:%s", restart_count);
        }
    }
    


}



function drawpoint(pointX, pointY, opacity) {
    stroke(202, 111, 31, opacity); //RGB&Opacity
    for (var x = pointX - 2; x < pointX + 3; x++) {
        for (var y = pointY - 2; y < pointY + 3; y++) {
            point(x, y);
        }
    }
}

function drawCircle(x, y, radius) {
    stroke(0, 0, 0);
    fill(0, 0); //fill(0,50);
    ellipse(x, y, radius * 2);
}

function distance(x1, y1, x2, y2) {
    var d = Math.pow(Math.pow((x1 - x2), 2) + Math.pow((y1 - y2), 2), 0.5);
    return d;
}


function addWeight(currX, currY, preX, preY, cur_radius, pre_radius) {

    var r_matrix = rotation_matrix(currX, currY, preX, preY, false); //旋轉矩陣
    var r_matrix = rotation_matrix(currX, currY, preX, preY, false); //旋轉矩陣

    map_count_all++;


    if (cur_radius - pre_radius > 0) { //半徑變大，表示遠離
        for (i = currX - cur_radius - 10; i <= currX + cur_radius + 10; i++) { //-10+10用來額外預估
            for (j = currY; j <= currY + cur_radius + 10; j++) {
                var weight_i = Math.round(((i - currX) * r_matrix[0] + (j - currY) * r_matrix[1]) + currX);
                var weight_j = Math.round(((i - currX) * r_matrix[2] + (j - currY) * r_matrix[3]) + currY);
                //console.log("Wi: %s, Wj: %s", weight_i, weight_j);
                if (startX + map_width > weight_i && weight_i > startX - map_width && startY > weight_j && weight_j > startY - map_width) { //be sure weight in local matrix
                    if (map_weight[weight_i][weight_j] >= 0) {
                        var dist = distance(weight_i, weight_j, currX, currY);

                        if (dist <= cur_radius) {
                            map_weight[weight_i][weight_j] += dist / Math.pow((cur_radius), 2);
                            map_count[weight_i][weight_j] += 1;
                        } else {
                            map_weight[weight_i][weight_j] += (cur_radius * 2 - dist) / Math.pow((cur_radius), 2);
                            map_count[weight_i][weight_j] += 1;
                        }


                    }
                }
            }
            for (j = currY; j >= currY - cur_radius; j--) {
                if (startX + map_width > weight_i && weight_i > startX - map_width && startY > weight_j && weight_j > startY - map_width) { //be sure weight in local matrix
                    if (map_weight[weight_i][weight_j] >= 0) {
                        var dist = distance(weight_i, weight_j, currX, currY);
                        if (dist <= cur_radius) {
                            map_weight[weight_i][weight_j] += dist / Math.pow((cur_radius), 2) * 0.9;
                            map_count[weight_i][weight_j] += 1;
                        } else {
                            map_weight[weight_i][weight_j] += (cur_radius * 2 - dist) / Math.pow((cur_radius), 2) * 0.9;
                            map_count[weight_i][weight_j] += 1;
                        }


                    }
                }
            }
        }
    } else if (cur_radius - pre_radius < 0) { //半徑變小，表示靠近
        for (i = currX - cur_radius - 10; i <= currX + cur_radius + 10; i++) { //-5+5用來額外預估
            for (j = currY; j >= currY - cur_radius - 10; j--) {
                var weight_i = Math.round((i - currX) * r_matrix[0] + (j - currY) * r_matrix[1]) + currX;
                var weight_j = Math.round((i - currX) * r_matrix[2] + (j - currY) * r_matrix[3]) + currY;
                if (startX + map_width > weight_i && weight_i > startX - map_width && startY > weight_j && weight_j > startY - map_width) { //be sure weight in local matrix
                    if (map_weight[weight_i][weight_j] >= 0) {
                        var dist = distance(weight_i, weight_j, currX, currY);
                        if (dist <= cur_radius) {
                            map_weight[weight_i][weight_j] += dist / Math.pow((cur_radius), 2);
                            map_count[weight_i][weight_j] += 1;
                        } else {
                            map_weight[weight_i][weight_j] += (cur_radius * 2 - dist) / Math.pow((cur_radius), 2);
                            map_count[weight_i][weight_j] += 1;
                        }

                    }
                }
            }
            for (j = currY; j <= currY + cur_radius; j++) {
                if (startX + map_width > weight_i && weight_i > startX - map_width && startY > weight_j && weight_j > startY - map_width) { //be sure weight in local matrix
                    if (map_weight[weight_i][weight_j] >= 0) {
                        var dist = distance(weight_i, weight_j, currX, currY);
                        if (dist <= cur_radius) {
                            map_weight[weight_i][weight_j] += dist / Math.pow((cur_radius), 2);
                            map_count[weight_i][weight_j] += 1;
                        } else {
                            map_weight[weight_i][weight_j] += (cur_radius * 2 - dist) / Math.pow((cur_radius), 2);
                            map_count[weight_i][weight_j] += 1;
                        }


                    }
                }
            }
        }
    }



}

function turnDecision(currX, currY, preX, preY) {

    var r_matrix = rotation_matrix(currX, currY, preX, preY, true); //旋轉矩陣
    degree = parseFloat(_PI_ / 4);
    var turn_matrix = [0, 0, 0];

    var cur_radius = Math.round(distance(currX, currY, targetX, targetY));
    var pre_radius = Math.round(distance(preX, preY, targetX, targetY));

    var slide1 = (1 / Math.sqrt(3));
    var slide2 = (-1 / Math.sqrt(3));
    var constant1 = calConstant(currX, currY, slide1);
    var constant2 = calConstant(currX, currY, slide2);



    //console.log("currX: %s, currY: %s, preX: %s, preY: %s", currX, currY, preX, preY);
    if (currX - preX > 0 && currY - preY < 0) { //行進方向為↗↖↙↘↑←↓→
        turnCase = 1;
    } else if (currX - preX < 0 && currY - preY < 0) { //行進方向為↖
        turnCase = 2;
    } else if (currX - preX < 0 && currY - preY > 0) { //行進方向為↙
        turnCase = 3;
    } else if (currX - preX > 0 && currY - preY > 0) { //行進方向為↘       
        turnCase = 4;
    } else if (currX - preX == 0 && currY - preY < 0) { //行進方向為↑
        turnCase = 5;
    } else if (currX - preX < 0 && currY - preY == 0) { //行進方向為←
        turnCase = 6;
    } else if (currX - preX == 0 && currY - preY > 0) { //行進方向為↓
        turnCase = 7;
    } else if (currX - preX > 0 && currY - preY == 0) { //行進方向為→
        turnCase = 8;
    }


    //console.log("TurnCase: ", turnCase);
    for (i = currX - cur_radius - 10; i <= currX + cur_radius + 10; i++) {
        for (j = currY + cur_radius + 10; j >= currY - cur_radius - 10; j--) {
            var weight_i = Math.round((i - currX) * r_matrix[0] + (j - currY) * r_matrix[1]) + currX;
            var weight_j = Math.round((i - currX) * r_matrix[2] + (j - currY) * r_matrix[3]) + currY;
            if (startX + map_width > weight_i && weight_i > startX - map_width && startY > weight_j && weight_j > startY - map_width) { //be sure weight in local matrix
                if (map_weight[weight_i][weight_j] > 0) {
                    if (Math.pow(weight_i - currX, 2) + Math.pow(weight_j - currY, 2) <= cur_radius) {
                        if (weight_j > currY) {
                            if (weight_i >= currX) {
                                turn_matrix = 2;
                            } else {
                                turn_matrix = 0;
                            }
                        } else {
                            if (weight_i >= currX) {
                                if (weight_j < (slide1 * weight_i + constant1)) {
                                    turn_matrix = 2;
                                } else {
                                    turn_matrix = 1;
                                }
                            } else {
                                if (weight_j < (slide2 * weight_i + constant2)) {
                                    turn_matrix = 0;
                                } else {
                                    turn_matrix = 1;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    if (cur_radius <= pre_radius) { //半徑變小，表示靠近
        if (bool_predecision == 1) {
            bool_predecision = 0;
            return preturn;
        } else {
            if (turn_matrix[0] >= turn_matrix[1]) {
                if (turn_matrix[0] > turn_matrix[2]) {
                    return 0;
                } else if (turn_matrix[0] == turn_matrix[2]) {
                    var rand = Math.round(Math.random() * 2)
                    if (rand == 0) {
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
                    var rand = Math.round(Math.random() * 2);
                    if (rand == 0) {
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
        bool_predecision = 1;
        if (preturn == 0) {
            preturn = 0;
            return 0;
        } else if (preturn == 2) {
            preturn = 2;
            return 2;
        } else {
            var rand = Math.round(Math.random() * 2);
            if (rand == 0) {
                preturn = 0;
                return 0;
            } else {
                preturn = 2;
                return 2;
            }
        }
    }


}

function rotation_matrix(currX, currY, preX, preY, oppo) {

    var deltaX = currX - preX;
    var deltaY = currY - preY;

    if (deltaX != 0 && deltaY != 0) {
        if (deltaX > 0 && deltaY < 0) { // ↗ °
            degree = degree * 7;
        } else if (deltaX > 0 && deltaY > 0) { // ↘ °
            degree = degree * 5;
        } else if (deltaX < 0 && deltaY < 0) { // ↖ °
            degree = degree * 1;
        } else if (deltaX < 0 && deltaY > 0) { // ↙ °
            degree = degree * 3;
        }
    } else if (deltaX == 0 && deltaY != 0) {
        if (deltaY < 0) { // ↑ °
            degree = degree * 0;
        } else if (deltaY > 0) { // ↓ °
            degree = degree * 4;
        }

    } else if (deltaX != 0 && deltaY == 0) {
        if (deltaX > 0) { // → °
            degree = degree * 6;
        } else if (deltaX < 0) { // ← °
            degree = degree * 2;
        }
    }
    if (oppo) {
        degree = -degree;
    }
    //console.log("Degree: %s", 180*degree/_PI_);
    var r_matrix = [Math.cos(degree), Math.sin(degree), -Math.sin(degree), Math.cos(degree)]; //旋轉矩陣

    return r_matrix;
}

function calConstant(x, y, m) { //計算直線常數
    //假設直線方程式為 y = mx+b 
    var b = y - (m * x);
    return b;
}

function flightMove(currX, currY, turnCases, descision, move_distance) {


    prepointX = currX;
    prepointY = currY;


    switch (Math.abs(turnCases)) {
        case 1: //行進方向為↗
            if (descision == 0) {
                currentX += 0;
                currentY += -Math.round(move_distance);
            } else if (descision == 1) {
                currentX += Math.round(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
                currentY += -Math.round(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
            } else if (descision == 2) {
                currentX += Math.round(move_distance);
                currentY += 0;
            }
            break;
        case 2: //行進方向為↖
            if (descision == 0) {
                currentX += -Math.round(move_distance);
                currentY += 0;
            } else if (descision == 1) {
                currentX += -Math.round(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
                currentY += -Math.round(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
            } else if (descision == 2) {
                currentX += 0;
                currentY += -Math.round(move_distance);
            }
            break;
        case 3: //行進方向為↙
            if (descision == 0) {
                currentX += 0;
                currentY += Math.round(move_distance);
            } else if (descision == 1) {
                currentX += -Math.round(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
                currentY += Math.round(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
            } else if (descision == 2) {
                currentX += -Math.round(move_distance);
                currentY += 0;
            }
            break;
        case 4: //行進方向為↘
            if (descision == 0) {
                currentX += Math.round(move_distance);
                currentY += 0;
            } else if (descision == 1) {
                currentX += Math.round(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
                currentY += Math.round(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
            } else if (descision == 2) {
                currentX += 0;
                currentY += Math.round(move_distance);
            }
            break;
        case 5: //行進方向為↑
            if (descision == 0) {
                currentX += -Math.round(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
                currentY += -Math.round(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
            } else if (descision == 1) {
                currentX += 0;
                currentY += -Math.round(move_distance);
            } else if (descision == 2) {
                currentX += Math.round(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
                currentY += -Math.round(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
            }
            break;
        case 6: //行進方向為←
            if (descision == 0) {
                currentX += -Math.round(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
                currentY += Math.round(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
            } else if (descision == 1) {
                currentX += -Math.round(move_distance);
                currentY += 0;
            } else if (descision == 2) {
                currentX += -Math.round(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
                currentY += -Math.round(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
            }
            break;
        case 7: //行進方向為↓
            if (descision == 0) {
                currentX += Math.round(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
                currentY += Math.round(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
            } else if (descision == 1) {
                currentX += 0;
                currentY += Math.round(move_distance);
            } else if (descision == 2) {
                currentX += -Math.round(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
                currentY += Math.round(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
            }
            break;
        case 8: //行進方向為→
            if (descision == 0) {
                currentX += Math.round(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
                currentY += -Math.round(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
            } else if (descision == 1) {
                currentX += Math.round(move_distance);
                currentY += 0;
            } else if (descision == 2) {
                currentX += Math.round(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
                currentY += Math.round(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
            }
            break;
        default:
            //console.log("Error");
    }


}

function twoleaf_FlightMove(currX, currY, turnCases, descision, move_distance) {


    prepointX = currX;
    prepointY = currY;

    if (step_count < 20) {
        currentX += (move_distance / 20) * (20 - step_count);
        currentY += -(move_distance / 20) * (step_count);
    } else if (20 <= step_count && step_count < 40) {
        currentX += -(move_distance / 20) * (40 - step_count);
        currentY += (move_distance / 20) * (step_count - 20);
    } else if (40 <= step_count && step_count < 60) {
        currentX += -(move_distance / 20) * (60 - step_count);
        currentY += -(move_distance / 20) * (step_count - 40);
    } else if (60 <= step_count && step_count < 80) {
        currentX += (move_distance / 20) * (80 - step_count);
        currentY += (move_distance / 20) * (step_count - 60);
    }


}


function restart() {

    if (restart_count == 100) {
        targetX += 3;
        targetY = 500 - Math.round(Math.sqrt(Math.pow(150, 2) - Math.pow(targetX - 500, 2)));
        restart_count = 0;
        console.log("Tar: %s, %s", targetX, targetY);
    }

    restart_count++;
    currentX = 500;
    currentY = 500;
    prepointX = 500;
    prepointY = 550;
    turnCase = 0;
    step_count = 0;

    preturn = 1;
    bool_predecision = 0; //上次走的方向是正確或錯誤，0表正確



    for (var i = 0; i < 2000; i++) {
        map_weight[i] = [];
        map_count[i] = [];
        for (var j = 0; j < 1200; j++) {
            map_weight[i][j] = 0;
            map_count[i][j] = 0;
        }
    }

    map_count_all = 0;


}

function createArray(length) {
    var arr = new Array(length || 0),
        i = length;

    if (arguments.length > 1) {
        var args = Array.prototype.slice.call(arguments, 1);
        while (i--) arr[length - 1 - i] = createArray.apply(this, args);
    }
    return arr;
}

function earth_distance(lat1, lon1, lat2, lon2, unit) {
    var theta, dist;
    theta = lon1 - lon2;
    dist = Math.sin(lat1) * Math.sin(lat2) + Math.cos(lat1) * Math.cos(lat2) * Math.cos(theta);
    dist = Math.acos(dist);
    dist = rad2deg(dist);
    dist = dist * 60 * 1.1515;
    dist = dist * 1.609344;
    return dist;
}

function deg2rad(deg) {
    return (deg * _PI_ / 180);
}

function rad2deg(rad) {
    return (rad * 180 / _PI_);
}

function gaussFilter(arrayRSSI) {


    for (var i = 0; i < arrayRSSI.length; i++) {
        arrayRSSI[i] = parseFloat(arrayRSSI[i]);
    }


    var u = 0.0,
        std = 0.0,
        len = arrayRSSI.length,
        count = 0,
        gau_result = 0.0;
    for (var i = 0; i < len; i++) {
        u += arrayRSSI[i];
    }
    u = u / len;

    for (var i = 0; i < len; i++) {
        std += pow(arrayRSSI[i] - u, 2);
    }

    std = pow(std / len, 0.5);

    for (var i = 0; i < len; i++) {
        if (u + 1.5 * std >= arrayRSSI[i] && arrayRSSI[i] >= u - 1.5 * std) {
            gau_result += arrayRSSI[i];
            count++;
        }
    }

    gau_result = gau_result / count;

    return gau_result;
}

function meidanFilter(arrayRSSI) {

    var m_result;
    arrayRSSI.sort(sortNumber);


    if (arrayRSSI.length % 2 == 0) {
        m_result = (arrayRSSI[(arrayRSSI.length / 2)] + arrayRSSI[(arrayRSSI.length / 2) - 1]) / 2;
    } else {
        m_result = arrayRSSI[floor((arrayRSSI.length / 2))];
    }

    return m_result;

}

function sortNumber(a, b) {
    return a - b;
}


function kalmanFilter(arrayRSSI) {
    var X = 0.0,
        P = 3.0,
        Q = 0.001,
        K = 0.0,
        R = 1.0,
        I = 1.0;
    for (var i = 0; i < arrayRSSI.length; i++) {
        X = X;
        P = P + Q;
        K = P / (P + R);
        X = X + K * (arrayRSSI[i] - X);
        P = (I - K) * P;
    }

    return X;
}

function rssiToDist(cur_rssi) {
    var dist = 0;

    var n = 2,
        A = -50; //n:path-loss exponent, A:RSSI per unit
    var log = 10,
        exp = (A - cur_rssi) / (10 * n);
    dist = pow(log, exp);
    return dist;

}

function getFakeRSSI(currX, currY, isFake) {

    var dist = distance(currX, currY, targetX, targetY);
    var n = 2,
        A = -50; //n:path-loss exponent, A:RSSI per unit
    var noise = 0.0;
    if (isFake) {
        if (dist >= 100) {
            noise = normalRandomScaled(noise, 120);
        } else if (100 > dist >= 50) {
            noise = normalRandomScaled(noise, 60);
        } else if (50 > dist >= 20) {
            noise = normalRandomScaled(noise, 30);
        } else if (20 > dist) {
            noise = normalRandomScaled(noise, 15);
        }
    }


    var rssi = A - 10 * n * Math.log10(dist) + noise;

    return rssi;
}

function predictPosition(large) {
    var threshold = 100.0;
    var predictDistance = 0.0;
    var distError = 0.0;

    //Find Current Maximum Weight
    for (var i = 0; i < map_weight.length; i++) {
        for (var j = 0; j < map_weight[i].length; j++) {
            if (map_weight[i][j] > 0 && map_count[i][j] > 0) {
                if (map_weight[i][j] / map_count[i][j] > large[0]) {
                    large[0] = map_weight[i][j] / map_count[i][j];
                }
            }
        }
    }

    for (var i = 0; i < map_weight.length; i++) {
        for (var j = 0; j < map_weight[i].length; j++) {
            if (map_weight[i][j] > 0 && map_count[i][j] > 0) {
                if (map_weight[i][j] / map_count[i][j] >= large[0] * 0.8) {
                    predictDistance = distance(i, j, currentX, currentY);
                    distError = Math.abs(predictDistance - radius);
                    if (distError <= threshold) {
                        threshold = distError;
                    }
                }
            }
        }
    }


    for (var i = 0; i < map_weight.length; i++) {
        for (var j = 0; j < map_weight[i].length; j++) {
            if (map_weight[i][j] > 0 && map_count[i][j] > 0) {
                if (map_weight[i][j] / map_count[i][j] >= large[0] * 0.8) {
                    predictDistance = distance(i, j, currentX, currentY);
                    distError = Math.abs(predictDistance - radius);
                    if (distError * 0.9 <= threshold) { //Hard Cord
                        large[1] += i;
                        large[2] += j;
                        large[3] += 1;
                    }
                }
            }
        }
    }


    large[1] = large[1] / large[3];
    large[2] = large[2] / large[3];
    //console.log("Large: %s, i: %s, j: %s, count: %s", large[0], large[1], large[2], large[3]);

}

function cleanMap() {

    if (radius > (map_width / 1.2) / 2 && re_calculate_weight_dist_more_100 != 10) {
        re_calculate_weight_dist_more_100++;
    } else if (re_calculate_weight_dist_more_100 == 10) {
        for (var i = 0; i < 2000; i++) {
            map_weight[i] = [];
            map_count[i] = [];
            for (var j = 0; j < 1200; j++) {
                map_weight[i][j] = 0;
                map_count[i][j] = 0;
            }
        }
        map_count_all = 0;
        re_calculate_weight_dist_more_100 = 0;
    }

    if (radius <= (map_width / 1.2) / 2 && re_calculate_weight == 0) { //距離目標20m時，地圖權重歸零重新分配
        for (var i = 0; i < 2000; i++) {
            map_weight[i] = [];
            map_count[i] = [];
            for (var j = 0; j < 1200; j++) {
                map_weight[i][j] = 0;
                map_count[i][j] = 0;
            }
        }
        map_count_all = 0;
        re_calculate_weight = 1;
    } else if (radius <= (map_width / 1.2) / 4 && re_calculate_weight == 1) {
        large = [0, 0, 0, 0];
        for (var i = 0; i < 2000; i++) {
            map_weight[i] = [];
            map_count[i] = [];
            for (var j = 0; j < 1200; j++) {
                map_weight[i][j] = 0;
                map_count[i][j] = 0;
            }
        }
        map_count_all = 0;
        re_calculate_weight = 2;
    }

}

function record_each100_point(d_error, move_d) {

    accumulation++;


    var k_rssiFiltered = kalmanFilter(g_rssiFiltered);


    var dist = distance(currentX, currentY, targetX, targetY);
    var n = 2,
        A = -50; //n:path-loss exponent, A:RSSI per unit
    var rssi = A - 10 * n * Math.log10(dist);

    if (!isNaN(k_rssiFiltered) && !isNaN(rssi) && !isNaN(d_error) && !isNaN(move_d)) {

        k_all[step_count] += parseFloat(k_rssiFiltered.toFixed(7));
        d_all[step_count] += parseFloat(rssi.toFixed(7));
        guess_error[step_count] += parseFloat(d_error.toFixed(7));
        total_distance[step_count] += parseFloat(move_d.toFixed(7));

        collect_all_count[step_count] += 1;


        /*
        if (targetX == 350  && restart_count==0) {

            //Collect Rssi
            g_all.push(parseFloat(g_rssiFiltered.toFixed(6)));
            m_all.push(parseFloat(m_rssiFiltered.toFixed(6)));
            k_all.push(parseFloat(k_rssiFiltered.toFixed(6)));
            d_all.push(parseFloat(rssi.toFixed(6)));

            guess_error.push(parseFloat(d_error.toFixed(6))); //Predict Position to Target Error
            total_distance.push(parseFloat(move_d.toFixed(6))); //Total move distance
            
            collect_all_count.push(1);


        } else {

            g_all[step_count] += parseFloat(g_rssiFiltered.toFixed(6));
            m_all[step_count] += parseFloat(m_rssiFiltered.toFixed(6));
            k_all[step_count] += parseFloat(k_rssiFiltered.toFixed(6));
            d_all[step_count] += parseFloat(rssi.toFixed(6));
            guess_error[step_count] += parseFloat(d_error.toFixed(6));
            total_distance[step_count] += parseFloat(move_d.toFixed(6));
            
            collect_all_count[step_count] += 1;

        }
        */
        //console.log(total_distance[0]);
    } else {
        //console.log("NaN");
    }


}



function linear_addWeight(currX, currY, preX, preY, cur_radius, pre_radius, rssi) {

    var r_matrix = rotation_matrix(currX, currY, preX, preY, false); //旋轉矩陣

    map_count_all++;


    if (cur_radius - pre_radius > 0) { //半徑變大，表示遠離
        for (i = currX - cur_radius - 10; i <= currX + cur_radius + 10; i++) { //-10+10用來額外預估
            for (j = currY; j <= currY + cur_radius + 10; j++) {
                var weight_i = Math.round(((i - currX) * r_matrix[0] + (j - currY) * r_matrix[1]) + currX);
                var weight_j = Math.round(((i - currX) * r_matrix[2] + (j - currY) * r_matrix[3]) + currY);
                //console.log("Wi: %s, Wj: %s", weight_i, weight_j);
                if (startX + map_width > weight_i && weight_i > startX - map_width && startY > weight_j && weight_j > startY - map_width) { //be sure weight in local matrix
                    if (map_weight[weight_i][weight_j] >= 0) {
                        var dist = distance(weight_i, weight_j, currX, currY);
                        var point_rssi = getFakeRSSI(i, j, false);

                        if (dist <= cur_radius) {
                            map_weight[weight_i][weight_j] += point_rssi / rssi;
                            //map_weight[weight_i][weight_j] += dist / Math.pow((cur_radius), 2);
                            map_count[weight_i][weight_j] += 1;
                        } else {
                            map_weight[weight_i][weight_j] += (2 * rssi - point_rssi) / rssi;
                            //map_weight[weight_i][weight_j] += (cur_radius * 2 - dist) / Math.pow((cur_radius), 2);
                            map_count[weight_i][weight_j] += 1;
                        }


                    }
                }
            }
            for (j = currY; j >= currY - cur_radius; j--) {
                if (startX + map_width > weight_i && weight_i > startX - map_width && startY > weight_j && weight_j > startY - map_width) { //be sure weight in local matrix
                    if (map_weight[weight_i][weight_j] >= 0) {
                        var dist = distance(weight_i, weight_j, currX, currY);
                        var point_rssi = getFakeRSSI(i, j, false);
                        if (dist <= cur_radius) {
                            map_weight[weight_i][weight_j] += point_rssi / rssi;
                            //map_weight[weight_i][weight_j] += dist / Math.pow((cur_radius), 2);
                            map_count[weight_i][weight_j] += 1;
                        } else {
                            map_weight[weight_i][weight_j] += (2 * rssi - point_rssi) / rssi;
                            //map_weight[weight_i][weight_j] += (cur_radius * 2 - dist) / Math.pow((cur_radius), 2);
                            map_count[weight_i][weight_j] += 1;
                        }


                    }
                }
            }
        }
    } else if (cur_radius - pre_radius < 0) { //半徑變小，表示靠近
        for (i = currX - cur_radius - 10; i <= currX + cur_radius + 10; i++) { //-5+5用來額外預估
            for (j = currY; j >= currY - cur_radius - 10; j--) {
                var weight_i = Math.round((i - currX) * r_matrix[0] + (j - currY) * r_matrix[1]) + currX;
                var weight_j = Math.round((i - currX) * r_matrix[2] + (j - currY) * r_matrix[3]) + currY;
                if (startX + map_width > weight_i && weight_i > startX - map_width && startY > weight_j && weight_j > startY - map_width) { //be sure weight in local matrix
                    if (map_weight[weight_i][weight_j] >= 0) {
                        var dist = distance(weight_i, weight_j, currX, currY);
                        var point_rssi = getFakeRSSI(i, j, false);
                        if (dist <= cur_radius) {
                            map_weight[weight_i][weight_j] += point_rssi / rssi;
                            //map_weight[weight_i][weight_j] += dist / Math.pow((cur_radius), 2);
                            map_count[weight_i][weight_j] += 1;
                        } else {
                            map_weight[weight_i][weight_j] += (2 * rssi - point_rssi) / rssi;
                            //map_weight[weight_i][weight_j] += (cur_radius * 2 - dist) / Math.pow((cur_radius), 2);
                            map_count[weight_i][weight_j] += 1;
                        }

                    }
                }
            }
            for (j = currY; j <= currY + cur_radius; j++) {
                if (startX + map_width > weight_i && weight_i > startX - map_width && startY > weight_j && weight_j > startY - map_width) { //be sure weight in local matrix
                    if (map_weight[weight_i][weight_j] >= 0) {
                        var dist = distance(weight_i, weight_j, currX, currY);
                        var point_rssi = getFakeRSSI(i, j, false);
                        if (dist <= cur_radius) {
                            map_weight[weight_i][weight_j] += point_rssi / rssi;
                            //map_weight[weight_i][weight_j] += dist / Math.pow((cur_radius), 2);
                            map_count[weight_i][weight_j] += 1;
                        } else {
                            map_weight[weight_i][weight_j] += (2 * rssi - point_rssi) / rssi;
                            //map_weight[weight_i][weight_j] += (cur_radius * 2 - dist) / Math.pow((cur_radius), 2);
                            map_count[weight_i][weight_j] += 1;
                        }


                    }
                }
            }
        }
    }
}

function tar_turnDecision(currX, currY, preX, preY, tarX, tarY) {

    var r_matrix = rotation_matrix(currX, currY, preX, preY, true); //旋轉矩陣
    degree = parseFloat(_PI_ / 4);
    var turn_matrix = 0;

    var slide1 = (1 / Math.sqrt(3));
    var slide2 = (-1 / Math.sqrt(3));
    var constant1 = calConstant(currX, currY, slide1);
    var constant2 = calConstant(currX, currY, slide2);
    

    var new_tarX = Math.round((tarX - currX) * r_matrix[0] + (tarY - currY) * r_matrix[1]) + currX;
    var new_tarY = Math.round((tarX - currX) * r_matrix[2] + (tarY - currY) * r_matrix[3]) + currY;
    



    //console.log("currX: %s, currY: %s, preX: %s, preY: %s", currX, currY, preX, preY);
    if (currX - preX > 0 && currY - preY < 0) { //行進方向為↗↖↙↘↑←↓→
        turnCase = 1;
    } else if (currX - preX < 0 && currY - preY < 0) { //行進方向為↖
        turnCase = 2;
    } else if (currX - preX < 0 && currY - preY > 0) { //行進方向為↙
        turnCase = 3;
    } else if (currX - preX > 0 && currY - preY > 0) { //行進方向為↘       
        turnCase = 4;
    } else if (currX - preX == 0 && currY - preY < 0) { //行進方向為↑
        turnCase = 5;
    } else if (currX - preX < 0 && currY - preY == 0) { //行進方向為←
        turnCase = 6;
    } else if (currX - preX == 0 && currY - preY > 0) { //行進方向為↓
        turnCase = 7;
    } else if (currX - preX > 0 && currY - preY == 0) { //行進方向為→
        turnCase = 8;
    }

    if(turnCase!=7){
        if (new_tarY > currY) {
            if (new_tarX >= currX) {
                turn_matrix = 2;
            } else {
                turn_matrix = 0;
            }
        } else {
            if (new_tarX <= currX) {
                if (new_tarY < (slide1 * new_tarX + constant1)) {
                    turn_matrix = 1;
                } else {
                    turn_matrix = 0;
                }
            } else {
                if (new_tarY < (slide2 * new_tarX + constant2)) {
                    turn_matrix = 1;
                } else {
                    turn_matrix = 2;
                }
            }
        }
    }
    else{
        if (new_tarY < currY) {
            if (new_tarX >= currX) {
                turn_matrix = 2;
            } else {
                turn_matrix = 0;
            }
        } else {
            if (new_tarX <= currX) {
                if (new_tarY < (slide1 * new_tarX + constant1)) {
                    turn_matrix = 1;
                } else {
                    turn_matrix = 2;
                }
            } else {
                if (new_tarY < (slide2 * new_tarX + constant2)) {
                    turn_matrix = 1;
                } else {
                    turn_matrix = 0;
                }
            }
        }
    }
    


    return turn_matrix;


}

//以下為標準差部分，用以添加高斯常態分佈變數
var spareRandom = null;

function normalRandom() {
    var val, u, v, s, mul;

    if (spareRandom !== null) {
        val = spareRandom;
        spareRandom = null;
    } else {
        do {
            u = Math.random() * 2 - 1;
            v = Math.random() * 2 - 1;

            s = u * u + v * v;
        } while (s === 0 || s >= 1);

        mul = Math.sqrt(-2 * Math.log(s) / s);

        val = u * mul;
        spareRandom = v * mul;
    }

    return val / 14; // 7 standard deviations on either side
}

function normalRandomInRange(min, max) {
    var val;
    do {
        val = normalRandom();
    } while (val < min || val > max);

    return val;
}

function normalRandomScaled(mean, stddev) {
    var r = normalRandomInRange(-1, 1);

    r = r * stddev + mean;

    return Math.round(r);
}

function lnRandomScaled(gmean, gstddev) {
    var r = normalRandomInRange(-1, 1);

    r = r * Math.log(gstddev) + Math.log(gmean);

    return Math.round(Math.exp(r));
}