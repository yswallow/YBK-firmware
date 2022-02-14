#  ビルド方法

## 必要な外部ファイル

* [Segger Embedded Studio](https://www.segger.com/downloads/embedded-studio)
* [nRF5 SDK](https://www.nordicsemi.com/Products/Development-software/nRF5-SDK/Download#infotabs)
    * nRF5 SDKの最新版とSoft Device S140が必要です。

## ビルド方法

1. Segger Embedded Studioをインストールします。
2. nRF5 SDKの圧縮ファイルを展開し，`example/peripheral/`又はそれと同等の深さを持ったフォルダ(`example/keyboard/`など)の下に当リポジトリをクローンします。
    * [Microsoft/UF2](https://github.com/Microsoft/uf2)を取り込むために`--recursive`が必要になっています。
    * 例: `~/nRF5_SDK/example/keyboard $ git clone --recursive https://github.com/yswallow/nRF52-keyboard-firmware.git`
3. Segger Embedded Studioで`(nRF52-keyboard-firmware/)keyboard/s140/ses/usbd_ble_split_keyboard_s140.emProject`を開きます。

### UF2をビルドする場合

4. 画面左上のドロップボックスから`PeripheralRelease`, `IntegratedRelease`又は`CentralRelease`を選択します。
5. `Build`から`Build usbd_ble_split_keyboard_s140`を選択するとビルドされます。
6. ファイルは `(nRF52-keyboard-firmware/)keyboard/s140/ses/Output/uf2/(ビルド設定名).uf2`に作成されます。


### SWDで書き込む場合
4. 画面左上のドロップボックスから`PeripheralDebug`, `IntegratedDebug`又は`CentralDebug`を選択します。
5. J-linkでキーボードを接続します。
6. `Debug`から`Go`を選択するとビルドと書き込みが行われ，main関数開始前で止まった状態になります。