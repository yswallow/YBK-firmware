# ビルド方法

UF2を作るだけならGNU Makeがおすすめです。

# GNU Makeでのビルド

## 必要なコマンド・外部ファイル

* [nRF5 SDK](https://www.nordicsemi.com/Products/Development-software/nRF5-SDK/Download#infotabs)
    * nRF5 SDK 17が必要です。
* arm-none-eabi{-gcc, -ld, -objcopy, -size}
* ccache
* make

## ビルド方法

1. nRF5 SDKの圧縮ファイルを任意の場所に展開します。
2. 当リポジトリをクローンします。`git clone --recursive https://github.com/yswallow/YBK-firmware.git`
3. `cd YBK-firmware/armgcc`
4. `cp Makefile.user.sample Makefile.user`
4. `Makefile.user`を編集します。
	* `SDK_ROOT`: nRF5 SDKの最上位ディレクトリへのパス (`path/to/nRF5_SDK`)。下記のSEGGER Embedded Studio用にの場所にクローンした場合は書かなくて済みます。（デフォルト: `../../../..`）
	* `GNU_INSTALL_ROOT`: arm-none-eabi-\* にPATHが通ってない場合は書いてください
	* `USE_RECENT_ARMGCC`: 基本的に1だと思います。SDKのファイルへのWarningでビルドが停止するのを防止します。
	* `UF2_OUTPUT_DIR`: UF2ファイルを保存するフォルダ。（デフォルト: `uf2`）
5. make {KeyboardType}:{ScanType}:{SoftDeviceVersion}; (例: `make peripheral:matrix:6`) 
	* `KeyboardType`:
		- `central`: 左手用
		- `peripheral`: 右手用
		- `integrated`: 一体型
	* `ScanType`:
		- `matrix`: COL2ROWのマトリックススキャン
		- `steno`: マトリックススキャンに上下隣接2キー同時押しのキーを加えたもの（設定方法のドキュメントはありません）
	* `SoftDeviceVersion`: 6 or 7 (UF2ブートローダーが使われているボードは基本的に6だと思います。例外的にSeeed XIAO BLEは7です。）
6. `UF2_OUTPUT_DIR`(YBK-firmware/armgcc/uf2/)に`{KeyboardType}_{ScanType}_{SoftDeviceVersion}.uf2`ができます。

# SEGGER Embedded Studio でのビルド

## 必要な外部ファイル

* [SEGGER Embedded Studio](https://www.segger.com/downloads/embedded-studio)
* [nRF5 SDK](https://www.nordicsemi.com/Products/Development-software/nRF5-SDK/Download#infotabs)
    * nRF5 SDKの最新版とSoft Device S140が必要です。

## ビルド方法

1. SEGGER Embedded Studioをインストールします。
2. nRF5 SDKの圧縮ファイルを展開し，`example/peripheral/`又はそれと同等の深さを持ったフォルダ(`example/keyboard/`など)の下に当リポジトリをクローンします。
    * [Microsoft/UF2](https://github.com/Microsoft/uf2)を取り込むために`--recursive`が必要になっています。
    * 例: `~/nRF5_SDK/example/keyboard $ git clone --recursive https://github.com/yswallow/YBK-firmware.git`
3. Segger Embedded Studioで`(YBK-firmware/)ses/usbd_ble_split_keyboard_s140.emProject`を開きます。

### UF2をビルドする場合

4. 画面左上のドロップボックスから`PeripheralRelease`, `IntegratedRelease`又は`CentralRelease`を選択します。
5. `Build`から`Build usbd_ble_split_keyboard_s140`を選択するとビルドされます。
6. ファイルは `(nRF52-keyboard-firmware/)keyboard/s140/ses/Output/uf2/(ビルド設定名).uf2`に作成されます。


### SWDで書き込む場合
4. 画面左上のドロップボックスから`PeripheralDebug`, `IntegratedDebug`又は`CentralDebug`を選択します。
5. J-linkでキーボードを接続します。
6. `Debug`から`Go`を選択するとビルドと書き込みが行われ，main関数開始前で止まった状態になります。


