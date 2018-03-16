# Dynamic Path Planning
以下說明各個資料夾與檔案在負責什麼任務

---
資料夾 Web_simulation <br>
-Code：包含各個飛行方式的html與其對應的js檔，以網頁的方式呈現，容易觀察其飛行路徑的變化 <br><br>
-Code Doc：點開index.html便可查看在Code內的js檔，每個function功能與其帶入參數為何<br><br>
-Recording Data：<br>
draw.py：帶入下面四個csv檔數值繪製實驗結果圖，程式比較簡單，看內部註解即可<br>
(Distance.csv、Location-Error.csv、Rate_below_5m.csv、RSSI.csv)<br>
&nbsp;&nbsp;--Result Pic：實驗結果圖，其根據主要有用的資料來繪製而成<br>
&nbsp;&nbsp;--Other Data：包含各種飛行方式、方向抉擇策略、權重方式等等所模擬出來的原始資料<br>

---
資料夾 Onboard-SDK-3.1<br>
\sample\commandline\ACNproject：只需要看這個資料夾，其他的是DJI所提供的範例<br><br>

\sample\commandline\ACNproject\DJI Simulation log 資料夾中，其包含了實際飛行前，使用DJI Simulator所跑的的結果<br>
\sample\commandline\ACNproject\DJI Real Flying log 資料夾中，為實際飛行的結果<br>
其儲存格式為[ UAV_latitude, UAV_longitude, UAV_altitude, RSSI, total_moveDist, guessLatitude, guessLongitude, error_dist, timeStamp]<br><br>

\sample\commandline\ACNproject\conboardSDK 資料夾 放置控制無人機程式碼<br>
-search.cpp 包含每個控制無人機的Function，可參照search.h<br>
-main.cpp 用以初始化無人機系統，並呼叫search.cpp內方法<br>
-其餘檔案皆不需動到，為DJI OnboardSDK內部套件<br><br>

編譯與使用方法：更改在conboardSDK內的程式碼後，直接在commandline下輸入make，<br>
使無人機起飛後，接著執行在bin內部下的a.out程式<br>

---
資料夾 Trajectory<br>
\Trajectory\Code 資料夾內有兩個Python檔案，用來畫
-draw_location_error.py 用來繪製每秒的location error關係圖
-trajectory.py 用根據每秒的經緯度繪製無人機的飛行路徑，另外也繪製每秒定位出來的訊號源位置
