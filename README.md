# YBK ( Yswallow|Ysni_pub Bluetooth Keyboard ) Firmware

<del>`docs/`もご覧ください。</del>

## 実装済みの機能

* WebHIDでの初期設定
    - マトリックスキーボードのCOL\_PINSとROW\_PINSの設定
* USBキーボード
* BLEキーボード
* 無線分割キーボード
    - Peripheral側だけでもUSBキーボードとして使用できる
* Remapでのキー設定（一部）
    - 長押しと短押しで異なる機能を割り当てる
    - マウスキー
    - [実装済みのキー](./docs/Keycodes.md) 参照
* Bluetooth経由でのRemapの利用
    - USB経由とはPIDが異なるため，JSONの書き換えが必要です。
* 音量調整などのConsumerControl機能(一部)
* NeoPixel機能

## 書き込み方法

1. [Adafruit_nRF52_Bootloader](https://github.com/adafruit/Adafruit_nRF52_Bootloader)が導入されたボードをパソコンに接続し，リセットボタンを2連打する
2. 表示されたドライブに[Releases](https://github.com/yswallow/YBK-firmware/releases)のUF2ファイルをコピーする
    - 分割キーボードの左手側は`CentralRelease.uf2`を, 右手側は`PeripheralRelease.uf2`を, 一体型キーボードでは `IntegratedRelease.uf2` を書き込んでください。
    
3. ドライブが自動で接続解除されたらリセットボタンを押し，再度ドライブとして認識されなければ書き込み成功
    
    - 書き込みに失敗する場合，ファイルが壊れている可能性があります。

## 初期設定方法

1. [設定ページ](https://yswallow.github.io/javascript/simple/webhid/nrf52_init.html)を開く
2. [Connect keyboard]を押す
3. 必要項目を埋める
    * Keyboard Rows: キーボードの行数
    * Keyboard Cols: キーボードの列数
    * keyboard Central Cols: キーボードの左手側の列数（分割キーボード用の設定）
    * Row Pins: マトリックススキャンの行ピンの番号(0~47)をカンマ区切りで記入
    * Col Pins: マトリックススキャンの列ピンの番号(0~47)をカンマ区切りで記入
4. [Send]を押す
5. 電源LEDやNeoPixelを使用する場合:
     1. Enable Power LEDにチェックを入れる
     2. Pin Number of Power LEDを入力する
     3. NeoPixel信号を出力するピンの番号`Pin Number of NeoPixel`に入力する
     4. NeoPixelの数を`LEDs count of NeoPixel`に入力する。NeoPixelを使用しない場合は0
     5. [Send Additional Data]を押す
6. 下の欄に正しく表示されればキーボードのリセットボタンを押す。
7. 再度[Connect keyboard]を押す。
8. 下の欄に正しく表示されれば完了。
    * 表示が正しくなければ3.から繰り返す。

## キーマップ設定方法

1. [Remap](https://remap-keys.app)を開く。
2. 設定ファイルを作る
    - [サリチル酸さんの記事](https://salicylic-acid3.hatenablog.com/entry/via-support#VIA%E3%81%AB%E8%AA%AD%E3%81%BF%E8%BE%BC%E3%81%BE%E3%81%9B%E3%82%8Bjson%E3%83%95%E3%82%A1%E3%82%A4%E3%83%AB%E3%82%92%E4%BD%9C%E6%88%90%E3%81%99%E3%82%8B)を参考にしてください
    - 本家VIAと違って、分割型キーボードは右手側のキーが左手側のcolの右に並ぶようになっています。（QMKは左手のROWのあとに右手のROWが左右反転して続く）
    - VID/PIDはRemapでキーボードを接続したときに左上に表示されるのをコピペしてください。
3. Remapに設定ファイルを適用する
4. Remapで設定する

## NeoPixelパターンの設定

https://yswallow.github.io/YBK-firmware/NeoPixel_Setting.html

## バグ

* <del>キー入力が止まらなくなる</del>
* <del>元のレイヤーに戻らなくなる</del>
* <del>特定のキーを打てなくなる</del>
* <del>左右間の接続が切れて復帰しない</del>
* <del>初期設定後に文字が入力され続ける</del>
* <del>Bluetoothが切断・再接続を無限に繰り返す</del>

## わかっている問題と解決策

### LinuxでWebHIDを利用できない

`sudo chmod 666 /dev/hidraw*` してください

### Windows上でBLE経由でRemapを利用する際，Unknown Deviceと表示される

Windowsの仕様です。

### 左右接続がうまくいかない

右手側の電源を先に入れるとうまくいきやすい気がします。

### Bluetoothが切断・再接続を無限に繰り返す

キーボードの電源を切り, コンピュータやスマートフォンでペアリングを解除し, 再度ペアリングしてください。

### BLEで新たに実装された機能を利用できない

コンピュータやスマートフォンでペアリングを解除し, 再度ペアリングしてください。

## 開発者向け

[ビルド方法](https://github.com/yswallow/YBK-firmware/blob/main/docs/How_to_Build.md)

