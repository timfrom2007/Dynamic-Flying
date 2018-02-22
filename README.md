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
\sample\commandline\ACNproject：只需要看這個資料夾<br>
