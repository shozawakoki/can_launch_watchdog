# can_launch_watchdog
# ROS 2 CAN通信監視・自動再起動プログラム

`can_launch_watchdog.py` は、ZLAC8015DモータドライバとのCAN通信を監視し、フィードバックデータの受信停止を検知した場合に、ROS 2のlaunchプロセスを自動的に再起動するPythonプログラムです。

CAN通信の物理的な切断を防止するものではなく、**異常が発生した後に既存のlaunchを停止・再起動すること**を目的としています。既存のROS 2パッケージやlaunchファイルを変更せずに使用できます。

## 動作環境

- Ubuntu 22.04
- ROS 2 Humble
- SocketCANインターフェース（初期設定：`can0`）
- CANopen対応のZLAC8015Dモータドライバ（初期設定：ノードID `1`）
- 起動対象のROS 2パッケージ：`zlac_can_bringup`
- Python 3（追加のPythonライブラリは不要）

**前提条件：** 通常のコマンド `ros2 launch zlac_can_bringup zlac_can_bringup.launch.py` でモータの起動・通信ができ、SocketCANインターフェースが使用可能な状態であること。

## ファイル構成

```
.
├── README.md
└── can_launch_watchdog.py
```

## 起動方法

1. `can_launch_watchdog.py` をロボットのUbuntu PCのホームディレクトリ（例：`/home/ubuntu/`）に配置する。
2. ターミナルでROS 2環境を読み込み、監視プログラムを実行する。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 配置したファイルの確認
ls -l ~/can_launch_watchdog.py

# 監視・自動再起動を開始
python3 ~/can_launch_watchdog.py
```

監視プログラムが内部で次のコマンドを実行するため、**別のターミナルでlaunchを重複起動しないでください。**

```bash
ros2 launch zlac_can_bringup zlac_can_bringup.launch.py
```

制御ログCSVを出力しない構成では、ログ更新監視を無効化して起動してください。

```bash
python3 ~/can_launch_watchdog.py --log-file ""
```

## 監視内容

監視プログラムは、独立したSocketCANソケットで以下のCANフレームを受信します（ノードID `1` の場合）。

| CAN ID | 監視対象 | 再起動条件 |
| --- | --- | --- |
| `0x181` | 左右車輪の速度フィードバック | 最終受信から2秒以上経過 |
| `0x381` | 左右車輪のエンコーダ位置フィードバック | 最終受信から2秒以上経過 |

**両方のCAN IDの受信を確認**し、いずれかの受信が途絶えた場合に異常と判定します。対象フレームは8バイト以上のデータを持つ標準CANデータフレームです。

さらに、既定では `/tmp/ladrc_track_200hz.csv` の更新時刻を監視し、**5秒以上更新されない場合**も異常と判定します。制御ログが存在しない、または更新されない環境では `--log-file ""` を指定してください。

起動直後はCANフィードバックと制御ログの初回受信・更新を待つ**15秒間の起動猶予**があります。

## 自動再起動の流れ

1. `zlac_can_bringup.launch.py` を起動する。
2. CANフレームと、設定されている場合は制御ログCSVの更新を監視する。
3. 受信停止、ログ更新停止、launch終了、CAN受信エラーなどを検知する。
4. 起動中のlaunchへ `SIGINT` を送り、終了処理を行う（終了しない場合は段階的に `SIGTERM` / `SIGKILL` を使用）。
5. **3秒間待機**し、launchを再起動する。
6. 監視を再開し、異常が続く場合は同じ処理を繰り返す。

正常動作中は約10秒ごとにCANフレームの最終受信からの経過時間を表示します。

## 設定項目

コマンドライン引数で設定を変更できます。

| 引数 | 初期値 | 説明 |
| --- | --- | --- |
| `--can-interface` | `can0` | CANインターフェース名 |
| `--node-id` | `1` | CANopenノードID（1〜127） |
| `--package` | `zlac_can_bringup` | 起動対象のROS 2パッケージ |
| `--launch-file` | `zlac_can_bringup.launch.py` | 起動対象のlaunchファイル |
| `--startup-timeout` | `15.0` | 起動直後の受信・ログ更新待機時間（秒） |
| `--feedback-timeout` | `2.0` | CANフレームの無受信判定時間（秒） |
| `--log-file` | `/tmp/ladrc_track_200hz.csv` | 更新監視するCSVファイル（空文字列で無効） |
| `--log-timeout` | `5.0` | CSV更新停止の判定時間（秒） |
| `--restart-delay` | `3.0` | launch終了から再起動までの待機時間（秒） |
| `--status-interval` | `10.0` | 正常時の監視ログ出力間隔（秒） |

設定例：CAN受信停止の判定時間を3秒、再起動までの待機時間を5秒に変更し、CSV監視を無効化する場合。

```bash
python3 ~/can_launch_watchdog.py \
  --feedback-timeout 3 \
  --restart-delay 5 \
  --log-file ""
```

すべての設定項目を確認する場合：

```bash
python3 ~/can_launch_watchdog.py --help
```

## 停止方法

監視プログラムを実行しているターミナルで `Ctrl+C` を押してください。監視を終了し、起動したlaunchにも終了シグナルを送信します。

## 注意事項

- **本プログラムは通信切断の原因を修復しません。** CANケーブル、コネクタ、電源などの物理的な異常が継続する場合、再起動が繰り返されます。
- 再起動中は制御処理が中断されます。**通信断時にモータが安全に停止することを本プログラム単体では保証しません。** ドライバ側の停止設定や安全回路を別途確認してください。
- launch再起動により、ハードウェア側のエンコーダ位置やオドメトリの連続性が失われる可能性があります。
- CSV監視を有効にする場合、対象ファイルが定期的に書き込まれている必要があります。ファイルが更新されない構成では `--log-file ""` を使用してください。
- CANフレーム受信の有無を検知する方式であり、モータの動作状態そのものを直接判定するものではありません。
- 初回の動作試験は、車輪を浮かせるなど安全を確保した状態で行ってください。

## ライセンス

このREADMEにはライセンスを指定していません。公開時に必要に応じてリポジトリへ `LICENSE` ファイルを追加してください。
