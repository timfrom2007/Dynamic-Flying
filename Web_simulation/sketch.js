// 參數初始化

//飛機位置
var currentX = 500;
var currentY = 500;

//目標位置
var targetX = 350;
var targetY = 500;

//飛機上個位置
var prepointX = 500;
var prepointY = 550;

var turnCase = 0;
var step_count = 0; //該次飛行的第幾步
var re_calculate = 0; //重新計算權重
var preturn = 1;
var bool_predecision = 0; //上次走的方向是正確或錯誤，0表正確
var range = [];
var restartbool = 0;
var restart_count = 1; //計算總共重新幾次


var speed; // 單位m/s
var tmp_timer=0;
var timer=[];  //單位second

var map_weight = []; //Create a Map with Weight
var map_count = [];
for (var i = 0; i < 2000; i++) {
    map_weight[i] = [];
    map_count[i] = [];
    for (var j = 0; j < 1200; j++) {
        map_weight[i][j] = 0;
        map_count[i][j] = 0;
    }
}


var guess_error = [];
var dist_error_cdf = [];
var cosume_time = [];
var total_distance = [];
var total_guess_error = [];


function setup() {
    createCanvas(2000, 1200);


    var radius = distance_error(currentX, currentY, targetX, targetY);
    var true_radius = distance(currentX, currentY, targetX, targetY);

    background(255);
    stroke(0);
    drawCircle(currentX, currentY, true_radius);
    line(0, 500, width, 500);
    line(500, 0, 500, height);


    stroke(22, 151, 131); //RGB&Opacity
    for (var x = targetX - 3; x < targetX + 4; x++) {
        for (var y = targetY - 3; y < targetY + 4; y++) {
            point(x, y);
        }
    }

    stroke(102, 41, 31); //RGB&Opacity
    for (var x = prepointX - 3; x < prepointX + 4; x++) {
        for (var y = prepointY - 3; y < prepointY + 4; y++) {
            point(x, y);
        }
    }

    var cur_radius = distance(currentX, currentY, targetX, targetY);
    var pre_radius = distance(prepointX, prepointY, targetX, targetY);
    addWeight(currentX, currentY, prepointX, prepointY, cur_radius, pre_radius);



}


function draw() {

    var cur_radius = Math.floor(distance(currentX, currentY, targetX, targetY));

    drawpoint(currentX, currentY, 255);
    //drawCircle(currentX,currentY,cur_radius);
    stroke(22, 151, 131); //RGB&Opacity
    for (var x = targetX - 3; x < targetX + 4; x++) {
        for (var y = targetY - 3; y < targetY + 4; y++) {
            point(x, y);
        }
    }




}



function mousePressed() {

    while (true) {

        if (step_count != 100) {
            step_count++;
            //console.log(step_count);
            var cur_radius = Math.floor(distance(currentX, currentY, targetX, targetY));
            var pre_radius = Math.floor(distance(prepointX, prepointY, targetX, targetY));
            var descision = turnDecision(currentX, currentY, prepointX, prepointY);
            var move_distance = 20;
            if (preturn == 1) {
                preturn = descision;
            }

            if (cur_radius <= 20) {
                move_distance = cur_radius;
            }

            if (step_count == 0) {
                total_distance.push(move_distance);
            } else {
                total_distance[restart_count - 1] += move_distance;
            }

            flightMove(currentX, currentY, turnCase, descision, move_distance);
            
            cur_radius = distance(currentX, currentY, targetX, targetY);
            pre_radius = distance(prepointX, prepointY, targetX, targetY);
            addWeight(currentX, currentY, prepointX, prepointY, cur_radius, pre_radius);

            var large = [0, 0, 0, 0];
            for (var i = 0; i < 2000; i++) {
                for (var j = 0; j < 1200; j++) {
                    if (map_weight[i][j] > 0 && map_count[i][j] > 0) {
                        if (map_weight[i][j] / map_count[i][j] > large[0]) {
                            large[0] = map_weight[i][j] / map_count[i][j];
                        }
                    }
                }
            }

            for (var i = 0; i < 2000; i++) {
                for (var j = 0; j < 1200; j++) {
                    if (map_weight[i][j] > 0 && map_count[i][j] > 0) {
                        if (map_weight[i][j] / map_count[i][j] >= large[0] * 0.9) {
                            large[1] += i;
                            large[2] += j;
                            large[3] += 1;
                        }
                    }
                }
            }

            large[1] = large[1] / large[3];
            large[2] = large[2] / large[3];
            //console.log("Large: %s, i: %s, j: %s, count: %s", large[0], large[1], large[2], large[3]);

            if (cur_radius <= 20 && re_calculate == 0) {   //距離目標20m時，地圖權重歸零重新分配
                large = [0, 0, 0, 0];
                for (var i = 0; i < 2000; i++) {
                    map_weight[i] = [];
                    map_count[i] = [];
                    for (var j = 0; j < 1200; j++) {
                        map_weight[i][j] = 0;
                        map_count[i][j] = 0;
                    }
                }
                re_calculate = 1;
            }

            var dist_error = Math.floor(Math.pow(Math.pow((large[1] - targetX), 2) + Math.pow((large[2] - targetY), 2), 0.5));
            var speed = moveDistance_to_speed(move_distance);
            accumulate_timer(move_distance,step_count,dist_error);

            if (restart_count == 1) {
                guess_error.push(dist_error);
            } else {
                guess_error[step_count - 1] += dist_error;
            }

            //console.log(targetX);

            if (step_count == 98) {
                dist_error_cdf.push(dist_error);
            }


        } else {



            for (var x = 0; x < 100; x++) {
                if (restart_count == 1) {
                    total_guess_error.push(guess_error[x]);
                    guess_error[x] = 0;
                } else {
                    total_guess_error[x] += guess_error[x];
                    guess_error[x] = 0;
                }
            }

            if (restart_count == 5) {

                for (var x = 0; x < 100; x++) {
                    //console.log("%s %s", Math.floor(total_guess_error[x] / restart_count), x + 1);
                    total_guess_error[x] = total_guess_error[x] / restart_count;
                }
                console.log(total_guess_error);
                console.log(dist_error_cdf);
                console.log(total_distance);
                console.log(timer);
                break;
            }
            restart();

        }


    }


}



function drawpoint(pointX, pointY, opacity) {
    stroke(202, 111, 31, opacity); //RGB&Opacity
    for (var x = pointX - 3; x < pointX + 4; x++) {
        for (var y = pointY - 3; y < pointY + 4; y++) {
            point(x, y);
        }
    }
}

function drawCircle(x, y, radius) {
    stroke(0, 0, 0);
    fill(0, 0); //fill(0,50);
    ellipse(x, y, radius * 2);
}

function distance_error(x1, y1, x2, y2) {
    var d = Math.pow(Math.pow((x1 - x2), 2) + Math.pow((y1 - y2), 2), 0.5);

    if (d > 80) {
        d = normalRandomScaled(d, 16.7);
    } else if (80 >= d && d > 40) {
        d = normalRandomScaled(d, 5.3);
    } else {
    d = d;
    }


    return d;
}

function distance(x1, y1, x2, y2) {
    var d = Math.pow(Math.pow((x1 - x2), 2) + Math.pow((y1 - y2), 2), 0.5);
    return d;
}


function addWeight(currX, currY, preX, preY, cur_radius, pre_radius) {

    var r_matrix = rotation_matrix(currX, currY, preX, preY); //旋轉矩陣


    if (cur_radius - pre_radius > 0) { //半徑變大，表示遠離
        for (i = currX - cur_radius - 10; i <= currX + cur_radius + 10; i++) { //-5+5用來額外預估
            for (j = currY; j <= currY + cur_radius + 10; j++) {
                var weight_i = Math.floor((i - currX) * r_matrix[0] + (j - currY) * r_matrix[1]) + currX;
                var weight_j = Math.floor((i - currX) * r_matrix[2] + (j - currY) * r_matrix[3]) + currY;
                if (weight_i >= 0 && weight_j >= 0) { //避免rotate後，weight_i&j為負數型態
                    if (map_weight[weight_i][weight_j] >= 0) {
                        var dist = distance(weight_i, weight_j, currX, currY);
                        if (cur_radius >= dist) {
                            if (dist <= cur_radius) {
                                map_weight[weight_i][weight_j] += dist / Math.pow((cur_radius), 2);
                                map_count[weight_i][weight_j] += 1;
                            } else {
                                map_weight[weight_i][weight_j] += (cur_radius * 2 - dist) / Math.pow((cur_radius), 2);
                                map_count[weight_i][weight_j] += 1;
                            }
                            //stroke(102, 211, 131, Math.floor(map_weight[weight_i][weight_j] * 2000 / map_count[weight_i][weight_j])); //RGB&Opacity
                            //point(weight_i, weight_j);
                        }
                    }
                }
            }
            for (j = currY; j >= currY - cur_radius; j--) {
                if (weight_i >= 0 && weight_j >= 0) { //避免rotate後，weight_i&j為負數型態
                    if (map_weight[weight_i][weight_j] >= 0) {
                        var dist = distance(weight_i, weight_j, currX, currY);
                        if (cur_radius >= dist) {
                            map_weight[weight_i][weight_j] += (dist / Math.pow((cur_radius), 2)) * 0.8;
                            map_count[weight_i][weight_j] += 1;
                            //stroke(102, 211, 131, Math.floor(map_weight[weight_i][weight_j] * 2000 / map_count[weight_i][weight_j])); //RGB&Opacity
                            //point(weight_i, weight_j);
                        }
                    }
                }
            }
        }
    } else if (cur_radius - pre_radius < 0) { //半徑變小，表示靠近
        for (i = currX - cur_radius - 10; i <= currX + cur_radius + 10; i++) { //-5+5用來額外預估
            for (j = currY; j >= currY - cur_radius - 10; j--) {
                var weight_i = Math.floor((i - currX) * r_matrix[0] + (j - currY) * r_matrix[1]) + currX;
                var weight_j = Math.floor((i - currX) * r_matrix[2] + (j - currY) * r_matrix[3]) + currY;
                if (weight_i >= 0 && weight_j >= 0) { //避免rotate後，weight_i&j為負數型態
                    if (map_weight[weight_i][weight_j] >= 0) {
                        var dist = distance(weight_i, weight_j, currX, currY);
                        if (cur_radius >= dist) {

                            if (dist <= cur_radius) {
                                map_weight[weight_i][weight_j] += dist / Math.pow((cur_radius), 2);
                                map_count[weight_i][weight_j] += 1;
                            } else {
                                map_weight[weight_i][weight_j] += (cur_radius * 2 - dist) / Math.pow((cur_radius), 2);
                                map_count[weight_i][weight_j] += 1;
                            }
                            //stroke(102, 211, 131, Math.floor(map_weight[weight_i][weight_j] * 2000 / map_count[weight_i][weight_j])); //RGB&Opacity
                            //point(weight_i, weight_j);
                        }
                    }
                }
            }
            for (j = currY; j <= currY + cur_radius; j++) {
                if (weight_i >= 0 && weight_j >= 0) { //避免rotate後，weight_i&j為負數型態
                    if (map_weight[weight_i][weight_j] >= 0) {
                        var dist = distance(weight_i, weight_j, currX, currY);
                        if (cur_radius >= dist) {
                            map_weight[weight_i][weight_j] += (dist / Math.pow((cur_radius), 2)) * 0.8;
                            map_count[weight_i][weight_j] += 1;
                            //stroke(102, 211, 131, Math.floor(map_weight[weight_i][weight_j] * 2000 / map_count[weight_i][weight_j])); //RGB&Opacity
                            //point(weight_i, weight_j);
                        }
                    }
                }
            }
        }
    }



}




function drawMap() {

    for (i = 0; i < 1000; i++) { //Draw Map Coverage
        for (j = 0; j < 600; j++) {
            if (map_weight[i][j] > 0) {
                stroke(102, 211, 131, map_weight[i][j] * 5); //RGB&Opacity
                point(i, j);
            }
        }
    }
}

function turnDecision(currX, currY, preX, preY) {

    var r_matrix = rotation_matrix(currX, currY, preX, preY); //旋轉矩陣
    var turn_matrix = [0, 0, 0];

    var cur_radius = Math.floor(distance(currX, currY, targetX, targetY));
    var pre_radius = Math.floor(distance(preX, preY, targetX, targetY));

    if (currX - preX == 0) {
        var slope = 0; //行進路線斜率
        var split_line1 = (slope - Math.tan(Math.PI / 3)) / (Math.tan(Math.PI / 3) * slope + 1);
        var split_line2 = (slope + Math.tan(Math.PI / 3)) / (1 - Math.tan(Math.PI / 3) * slope);
    } else {
        var slope = (currY - preY) / (currX - preX); //行進路線斜率
        var split_line1 = (slope - Math.tan(Math.PI / 6)) / (Math.tan(Math.PI / 6) * slope + 1);
        var split_line2 = (slope + Math.tan(Math.PI / 6)) / (1 - Math.tan(Math.PI / 6) * slope);
    }

    if (split_line2 > split_line1) { //把斜率比較大的當成line1
        split_line1 = split_line1 + split_line2;
        split_line2 = split_line1 - split_line2;
        split_line1 = split_line1 - split_line2;
    }
    var constant1 = calConstant(currX, currY, split_line1);
    var constant2 = calConstant(currX, currY, split_line2);


    //console.log("currX: %s, currY: %s, preX: %s, preY: %s", currX, currY, preX, preY);


    if (currX - preX > 0 && currY - preY < 0) { //行進方向為↗
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
    if (cur_radius - pre_radius <= 0) { //半徑變小，表示靠近
        for (i = currX - cur_radius - 10; i <= currX + cur_radius + 10; i++) {
            for (j = currY; j >= currY - cur_radius - 10; j--) {
                var weight_i = Math.floor((i - currX) * r_matrix[0] + (j - currY) * r_matrix[1]) + currX;
                var weight_j = Math.floor((i - currX) * r_matrix[2] + (j - currY) * r_matrix[3]) + currY;
                if (weight_i >= 0 && weight_j >= 0) {
                    if (map_weight[weight_i][weight_j] > 0) {

                        if (Math.pow(weight_i - currX, 2) + Math.pow(weight_j - currY, 2) <= cur_radius) {
                            switch (turnCase) {
                                case 1:
                                    if (weight_i * split_line2 + constant2 >= weight_j) {
                                        turn_matrix[0] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 > weight_j && weight_i * split_line2 + constant2 < weight_j) {
                                        turn_matrix[1] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 <= weight_j) {
                                        turn_matrix[2] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    break;

                                case 2:
                                    if (weight_i * split_line2 + constant2 <= weight_j) {
                                        turn_matrix[0] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 < weight_j && weight_i * split_line2 + constant2 > weight_j) {
                                        turn_matrix[1] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 >= weight_j) {
                                        turn_matrix[2] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    break;

                                case 3:
                                    if (weight_i * split_line2 + constant2 <= weight_j) {
                                        turn_matrix[0] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 < weight_j && weight_i * split_line2 + constant2 > weight_j) {
                                        turn_matrix[1] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 >= weight_j) {
                                        turn_matrix[2] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    break;

                                case 4:
                                    if (weight_i * split_line2 + constant2 >= weight_j) {
                                        turn_matrix[0] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 > weight_j && weight_i * split_line2 + constant2 < weight_j) {
                                        turn_matrix[1] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 <= weight_j) {
                                        turn_matrix[2] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    break;

                                case 5:
                                    if (weight_i * split_line1 + constant1 <= weight_j) {
                                        turn_matrix[0] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 > weight_j && weight_i * split_line2 + constant2 > weight_j) {
                                        turn_matrix[1] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line2 + constant2 <= weight_j) {
                                        turn_matrix[2] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    break;

                                case 6:
                                    if (weight_i * split_line2 + constant2 <= weight_j) {
                                        turn_matrix[0] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 < weight_j && weight_i * split_line2 + constant2 > weight_j) {
                                        turn_matrix[1] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 >= weight_j) {
                                        turn_matrix[2] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    break;

                                case 7:
                                    if (weight_i * split_line1 + constant1 >= weight_j) {
                                        turn_matrix[0] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 < weight_j && weight_i * split_line2 + constant2 < weight_j) {
                                        turn_matrix[1] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line2 + constant2 >= weight_j) {
                                        turn_matrix[2] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    break;

                                case 8:
                                    if (weight_i * split_line2 + constant2 >= weight_j) {
                                        turn_matrix[0] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 > weight_j && weight_i * split_line2 + constant2 < weight_j) {
                                        turn_matrix[1] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    if (weight_i * split_line1 + constant1 <= weight_j) {
                                        turn_matrix[2] += map_weight[weight_i][weight_j] / map_count[weight_i][weight_j];
                                    }
                                    break;

                                default:
                                    //console.log("Error");
                            }
                        }
                    }
                }
            }
        }
    } else if (cur_radius - pre_radius > 0) { //半徑變大，表示遠離
        for (i = currX - cur_radius - 10; i <= currX + cur_radius + 10; i++) {
            for (j = currY; j <= currY + cur_radius + 10; j++) {
                var weight_i = Math.floor((i - currX) * r_matrix[0] + (j - currY) * r_matrix[1]) + currX;
                var weight_j = Math.floor((i - currX) * r_matrix[2] + (j - currY) * r_matrix[3]) + currY;
                if (weight_i >= 0 && weight_j >= 0) {
                    if (map_weight[weight_i][weight_j] > 0) {
                        if (Math.pow(weight_i - currX, 2) + Math.pow(weight_j - currY, 2) <= cur_radius) {
                            switch (turnCase) {
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
                                    //console.log("Error");
                            }
                        }
                    }
                }
            }
        }
    }
    //console.log("TrunMatrix: %s, %s, %s", turn_matrix[0].toFixed(5), turn_matrix[1].toFixed(5), turn_matrix[2].toFixed(5));


    if (cur_radius < pre_radius) { //半徑變小，表示靠近
        if (bool_predecision == 1) {
            bool_predecision = 0;
            return preturn;
        } else {
            if (turn_matrix[0] >= turn_matrix[1]) {
                if (turn_matrix[0] > turn_matrix[2]) {
                    return 0;
                } else if (turn_matrix[0] == turn_matrix[2]) {
                    var rand = Math.floor(Math.random() * 2)
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
                    var rand = Math.floor(Math.random() * 2);
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
            var rand = Math.floor(Math.random() * 2);
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

function rotation_matrix(currX, currY, preX, preY) {
    var degree = Math.PI / 4; //  π/4 = 45°
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
    //console.log("Degree: %s", 180*degree/Math.PI);
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
    
    
    
    //console.log("Descision: %s", descision)
    switch (Math.abs(turnCases)) {
        case 1:
            if (descision == 0) {
                currentX += 0;
                currentY += -move_distance;
            } else if (descision == 1) {
                currentX += Math.floor(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
                currentY += -Math.floor(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
            } else if (descision == 2) {
                currentX += move_distance;
                currentY += 0;
            }
            break;
        case 2:
            if (descision == 0) {
                currentX += -move_distance;
                currentY += 0;
            } else if (descision == 1) {
                currentX += -Math.floor(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
                currentY += -Math.floor(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
            } else if (descision == 2) {
                currentX += 0;
                currentY += -move_distance;
            }
            break;
        case 3:
            if (descision == 0) {
                currentX += 0;
                currentY += move_distance;
            } else if (descision == 1) {
                currentX += -Math.floor(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
                currentY += Math.floor(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
            } else if (descision == 2) {
                currentX += -move_distance;
                currentY += 0;
            }
            break;
        case 4:
            if (descision == 0) {
                currentX += move_distance;
                currentY += 0;
            } else if (descision == 1) {
                currentX += Math.floor(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
                currentY += Math.floor(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
            } else if (descision == 2) {
                currentX += 0;
                currentY += move_distance;
            }
            break;
        case 5:
            if (descision == 0) {
                currentX += -Math.floor(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
                currentY += -Math.floor(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
            } else if (descision == 1) {
                currentX += 0;
                currentY += -move_distance;
            } else if (descision == 2) {
                currentX += Math.floor(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
                currentY += -Math.floor(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
            }
            break;
        case 6:
            if (descision == 0) {
                currentX += -Math.floor(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
                currentY += Math.floor(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
            } else if (descision == 1) {
                currentX += -move_distance;
                currentY += 0;
            } else if (descision == 2) {
                currentX += -Math.floor(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
                currentY += -Math.floor(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
            }
            break;
        case 7:
            if (descision == 0) {
                currentX += Math.floor(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
                currentY += Math.floor(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
            } else if (descision == 1) {
                currentX += 0;
                currentY += move_distance;
            } else if (descision == 2) {
                currentX += -Math.floor(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
                currentY += Math.floor(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
            }
            break;
        case 8:
            if (descision == 0) {
                currentX += Math.floor(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
                currentY += -Math.floor(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
            } else if (descision == 1) {
                currentX += move_distance;
                currentY += 0;
            } else if (descision == 2) {
                currentX += Math.floor(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
                currentY += Math.floor(Math.pow((Math.pow(move_distance, 2)) / 2, 0.5));
            }
            break;
        default:
            //console.log("Error");
    }

}

function moveDistance_to_speed(move_distance){
    
    if(move_distance>=5){
        speed = 5;
    }
    else{
        speed = move_distance;
    }
    
    return speed;
}

function accumulate_timer(move_distance,step,dist_error,step){  //主要以誤差低於2時停止計時，或是step=99仍無法精準定位出時停止計時
    
    if(move_distance>5){
        tmp_timer += move_distance/speed;
    }
    else{
        tmp_timer += 1;
    }
    
    if (speed < 5) {
        if (step == 99 || dist_error <= 5) {
            timer.push(tmp_timer);
            tmp_timer = 0;
        }
    }
    
    
    
}

function restart() {
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

    /*
    var x_rand = Math.floor((Math.random() * 6) + 5) * 20;
    var y_rand = Math.floor((Math.random() * 6) + 5) * 20;
    var x_bool = Math.floor((Math.random() * 1) + 1);
    if (x_bool < 1) {
        x_rand = x_rand * -1;
    }
    targetX = 500 + x_rand;
    targetY = 500 - y_rand;
    */

    if (targetY == 400) {
        if (targetX == 354) {
            targetX = 645;
        }
        targetX += 1;
        targetY = 500;

    } else {
        targetY -= 1;
    }


    radius = Math.floor(Math.pow(Math.pow((currentX - targetX), 2) + Math.pow((currentY - targetY), 2), 0.5));

    //console.log("Tar: %s, %s", targetX, targetY);

}

function restart_next() {
    targetX = 400;
    targetY = 500;
    restart_count = 1;

    restart();
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