# 環境構築手順書

## 1 本書について

本書では、ネットワークデータ作成支援ツール（以下「本ツール」という。）の利用環境構築手順について記載しています。本ツールの構成や仕様の詳細については以下も参考にしてください。

[技術検証レポート](XXX)

## 2 動作環境

本ツールの動作環境は以下のとおりです。

【LOD1、LOD2交通（道路）モデルからネットワークを作成する場合】

| 項目 | 最小動作環境 | 推奨動作環境 |
| - | - | - |
| OS | Microsoft Windows 10 または 11 | 同左 |
| CPU | Intel Core i5以上 | Intel Core i7以上 |
| メモリ | 8GB以上 | 16GB以上 |
| ネットワーク | 不要 |  同左 |

【LOD3交通（道路）モデルからネットワークを作成する場合】

| 項目 | 最小動作環境 | 推奨動作環境 |
| - | - | - |
| OS | Microsoft Windows 10 または 11 | 同左 |
| CPU | Intel Core i7以上 | 同左 |
| メモリ | 16GB以上 | 同左 |
| ネットワーク | 不要 |  同左 |

## 3 インストール手順

[こちら](https://github.com/Project-PLATEAU/PLATEAU-RoadNetwork-Generator/releases/)
からアプリケーションをダウンロードします。

ダウンロード後、zipファイルを右クリックし、「すべて展開」を選択することで、zipファイルを展開します。

展開されたフォルダ内の「NetworkCreator.exe」をダブルクリックすることで、アプリケーションが起動します。

![zip解凍](../resources/devMan/devMan_001.png)

## 4 ビルド手順

自身でソースファイルをダウンロードしビルドを行うことで、実行ファイルを作成することができます。\
ソースファイルは
[こちら](https://github.com/Project-PLATEAU/PLATEAU-RoadNetwork-Generator/)
からダウンロード可能です。

GitHubからダウンロードしたソースファイルの構成は以下のようになっています。

![ソース構成](../resources/devMan/devMan_002.png)

（1）本ツールのソリューションファイル（NetworkCreator.sln）をVisualStudio2019で開きます。

ソリューションファイルは、PLATEAU-RoadNetwork-Generator\\srcに格納されています。

（2）NetworkCreator.slnをVisualStudio2019で開くと、ソリューション'NetworkCreator'に4つのプロジェクトが表示されます。

以下の赤枠部分のように、ソリューション構成を【Release】に、ソリューションプラットフォームを【x64】に設定します。

![ソリューション構成](../resources/devMan/devMan_003.png)

（3）以下の赤枠部分のように、\[ソリューションのビルド\]を選択し、ソリューション全体をビルドします。

![ビルド](../resources/devMan/devMan_004.png)

（4）ビルドが正常に終了すると、ソリューションファイルと同じフォルダにあるNetworkCreator\\bin\\Releaseフォルダに実行ファイルが作成されます。 \
Releaseフォルダ内のファイル一式がアプリケーション実行時に必要なファイルです。

![ビルド結果](../resources/devMan/devMan_005.png)
