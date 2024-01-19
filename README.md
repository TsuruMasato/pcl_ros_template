# 日本語解説 (English follows below)

PCLをROSで利用する際の、CMakeLists.txtとpackage.xmlの作例です。

初心者が実行まで確認できるよう、線分検出のデモ付きの形で実装しています。

https://github.com/TsuruMasato/pcl_ros_template/assets/14979823/cb0fd983-3f9c-45f7-b7f1-bed66a645a99



## コンパイル方法

> ROS初めての人はまずROSのインストール作業を完了して下さい。  
また、pcl_rosというROSの基本的なライブラリも使用しています。ROSのdesktop full版であればapt install時に自動入手済みのはずです。  
`$ rospack find pcl_ros`と打って見つからなければ、`$ apt install ros-noetic-pcl-ros`でpcl_rosをインストールして下さい。

まず、自分のcatkin_wsのsrcディレクトリにコードをダウンロードします。

1. `$ cd your_catkin_ws/src`

2. `$ git clone git@github.com:TsuruMasato/edge_detector_ros.git`

次にビルドです。Releaseオプションは飛ばしても大丈夫ですが、付けると実行速度が上がります。

3. `$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release`

4. `$ catkin build`

最後に、環境変数を読み込み直しましょう。

5. `$ source devel/setup.bash`

## デモの実行

#### パターンA : AzureKinectの入力点群をリアルタイムで処理するデモ

1. AzureKinectのROSドライバノードが必須です。

   以前記事を書きましたので、これに従いドライバをインストールしてから次にお進み下さい(https://qiita.com/sunrise_lover/items/1a70ddafee44419eda2a)

2. ドライバが入れば、あとは以下のlaunchでAzureKinectとこの点群処理ノードが同時に起動します。

`$ roslaunch edge_detector_ros edge_detector_with_AzureKinect.launch`


#### パターンB : ROSノード単体の起動

`$ rosrun edge_detector_ros edge_detector_node`

上記のコマンドでノード自体は起動できます。

ただし、入力となる点群ストリームが必須です。RealSenseやLiDARなど、sensor_msgs::PointCloud2型であれば接続できます。

launchファイルを参考に、トピック名のremapを適切に行って下さい。


## ここから開発を始める方へ

デモまで続けて実行するため、このソフトはedge_detectorというプロジェクト名を設定しています。

CMakeLists.txtやpackage.xmlファイル、cppファイルなどに散在する`edge_detector` や `edge_detector_ros` といった固有名詞をご自身の開発に沿った名称に変更すると良いでしょう。




---

---

******



# English explanation of PCL ROS templete

Example of CMakeLists.txt and package.xml for PointCloud process with ROS.

This contains a simple demo:

https://github.com/TsuruMasato/pcl_ros_template/assets/14979823/cb0fd983-3f9c-45f7-b7f1-bed66a645a99



## How to Build

First, please go to your catkin src directory and download this source code:

1. `$ cd your_catkin_ws/src`

2. `$ git clone git@github.com:TsuruMasato/edge_detector_ros.git`

Then, compile the catkin projects

3. `$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release`

4. `$ catkin build`

Finally, please re-load new environment variables:

5. `$ source devel/setup.bash`

## How to Run the demo

#### Type A : start with AzureKinect ros driver

1. AzureKinect ROS Driver is NECESSARY!!

   Please install it first.(https://qiita.com/sunrise_lover/items/1a70ddafee44419eda2a)

2. Then, start the ros node with:

`$ roslaunch edge_detector_ros edge_detector_with_AzureKinect.launch`


#### Type B : single node

`$ rosrun edge_detector_ros edge_detector_node`

This starts a ros node alone.

You need to remap the input point cloud topic.

## For your development

To simplify the code, I named the whole project as "edge_detector_ros".

Please replace all "edge_detector" and "edge_detector_ros" to your own project name.

