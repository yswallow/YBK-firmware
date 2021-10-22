# nRF52 Keyboard Firmware

**試用限定です！バグがあります！業務で使用しないでください！**

<del>`doc/`もご覧ください。</del>

## 実装済みの機能

* WebHIDでの初期設定
    - マトリックスキーボードのCOL_PINSとROW_PINSの設定
* USBキーボード
* BLEキーボード
* 無線分割キーボード
    * Slave側だけでもUSBキーボードとして使用できる
* Remapでのキー設定（一部）
    - 長押しと短押しで異なる機能を割り当てる
    - マウスキー

## 実装予定の機能

* 音量調整などのConsumerControl機能
* 短押しで`Ctrl+Space`，長押しでレイヤー切り替え


## 書き込み方法

1. [Adafruit_nRF52_Bootloader](https://github.com/adafruit/Adafruit_nRF52_Bootloader)が導入されたボードをパソコンに接続し，リセットボタンを2連打する
2. 表示されたドライブにUF2ファイルをコピーする
3. ドライブが自動で接続解除されたらリセットボタンを押し，再度ドライブとして認識されなければ書き込み成功
    
    - 書き込みに失敗する場合，ファイルが壊れている可能性があります。

## 初期設定方法

1. [設定ページ](https://yswallow.github.io/javascript/simple/webhid/nrf52_init.html)を開く
2. [Connect keyboard]を押す
3. 必要項目を埋める
    * Keyboard Rows: キーボードの行数
    * Keyboard Cols: キーボードの列数
    * keyboard Central Cols: キーボードの左手側の列数（分割キーボード用の設定）
    * Row Pins: マトリックススキャンの行ピンの番号を0~48の番号をカンマ区切りで記入
    * Col Pins: マトリックススキャンの列ピンの番号を0~48の番号をカンマ区切りで記入
4. [Send]を押す
5. 下の欄に正しく表示されればキーボードのリセットボタンを押す。
6. 再度[Connect keyboard]を押す。
7. 下の欄に正しく表示されれば完了。
    * 表示が正しくなければ3.から繰り返す。

## キーマップ設定方法

1. [Remap](https://remap-keys.app)を開く。
2. 設定ファイルを作る
    - [サリチル酸さんの記事](https://salicylic-acid3.hatenablog.com/entry/via-support#VIA%E3%81%AB%E8%AA%AD%E3%81%BF%E8%BE%BC%E3%81%BE%E3%81%9B%E3%82%8Bjson%E3%83%95%E3%82%A1%E3%82%A4%E3%83%AB%E3%82%92%E4%BD%9C%E6%88%90%E3%81%99%E3%82%8B)を参考にしてください
    - VID/PIDはRemapでキーボードを接続したときに左上に表示されるのをコピペしてください。
3. Remapに設定ファイルを適用する
4. Remapで設定する

## バグ

* <del>キー入力が止まらなくなる</del>
* 元のレイヤーに戻らなくなる
* <del>特定のキーを打てなくなる</del>
* 左右間の接続が切れて復帰しない
* <del>初期設定後に文字が入力され続ける</del>


## わかっている問題と解決策

### LinuxでWebHIDを利用できない

`sudo chmod 666 /dev/hidraw*` してください

