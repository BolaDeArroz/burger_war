# 環境を準備する
## Ubuntu 16.04 の環境構築
LinuxのUbuntu 16.04が必要です。
これにはいくつかの方法があります。

## HDDへインストールする
PCのHDDへインストールする方法です。

使用するUbuntuは古いバージョンの16.04です。
http://releases.ubuntu.com/16.04/
からダウンロードできます。ほとんどの場合、「64-bit PC (AMD64) desktop image」をダウンロードすればよいです。

次にUSBメモリを準備します。２GB以上必要です。
専用の書き込みソフトを使って、ダウンロードしたimageをUSBへ書き込みます。
Rufusというソフトがシンプルでおすすめです。　https://rufus.ie/

インストール方法はインターネット上にたくさん解説があるのでご覧ください。

外付けUSBメモリ等にインストールする方法もあります。(参考 https://freepc.jp/post-34573)
ブートローダーのインストール先を外付けUSBドライブにしないと内蔵HDDの環境に影響与えてしまうので注意してください。

## Windowの上でUbuntuを動かす
VirtualPCやVMwareを使うことでWindows上にも環境を作れます。
参考
https://github.com/gogo5nta/burger_war/blob/master/info.md

Windows subaystem for Linux使った方法はGazeboが動作しない場合があり、難しいかもしれません。

## VPSを使う
参考
https://github.com/hotic06/burger_war/blob/master/doc/CloudInstall.md
