不定期更新工具脚本

#### changeBagFramerate.py
将Bag中的指定Topic转换到目标帧率并输出为新的Bag文件。
**用法**：Input Format: Input_bag_path topic_name1 target_rate1 [topic_name2 target_rate2 ... ] Output_bag_path

#### convertIMU2csv.py
将Bag中的`/camera/imu`的IMU Topic以EuRoC格式输出为csv文件。
**用法**：Input Format: Input_bag_path

#### extractSelectedTopic.py
提取源Bag文件中的指定的Topic到新的Bag文件中。
**用法**：Input Format: Input_bag_path

#### genTimeStamps.py
用于根据影像文件名生成时间戳索引文件。
**用法**：Input Format: Input_image_dir

#### GPULogger.py
用于获取某段时间内的GPU使用情况，并且将监测数据保存到文件。
**用法**：Input Format: process_name

#### parseBag.py
用于提取Bag文件中的Image类型Topic，并输出，包含多种模式：
* (1)Output preview video
* (2)Output general summary file
* (3)Output preview video and general summary file
* (4)Output frame images
* (5)Output frames of all image topics
* (6)Output preview video, summary file, frame images of all topics
* (7)Show frame images

各模式解释如下：
* Mode 1: 将选择的Image Topic生成一段预览视频
* Mode 2: 输出一个包含包起止时间、时长、所有Topic基本信息、Image Topic的帧率、大小、数据类型的文本文件
* Mode 3：同时生成预览视频和信息汇总文件
* Mode 4：将选择的Image Topic拆解成帧影像
* Mode 5：将所有的Image Topic都拆解成帧影像
* Mode 6: 同时生成预览视频、信息汇总文件、以及将所有Image Topic拆解成帧影像
* Mode 7: 只展示指定Topic

**用法**：Input Format: Input_bag_path

#### parseBagAuto.py
用于批量提取Bag文件中的Image类型Topic，并输出，包含多种模式：
* (1)Output preview video
* (2)Output general summary file
* (3)Output preview video and general summary file
* (4)Output frames of all image topics
* (5)Output preview video, summary file, frame images of all topics

各模式解释如下：
* Mode 1: 批量生成预览视频文件
* Mode 2: 批量生成汇总信息文件
* Mode 3: 同时批量生成预览视频文件和信息汇总文件
* Mode 4: 批量将Bag中所有Image Topic拆解成帧影像
* Mode 5: 同时批量生成预览视频文件、信息汇总文件，并将Bag中所有Image Topic拆解成帧影像

**用法**：Input Format: Input_bag_path

#### parseEuRoCgroundtruth2TUM.py
用于将EuRoC格式的IMU数据转换为TUM格式。
**用法**：Input Format: Input_file_path

#### saveOdometry.py
用于保存ROS节点发布的Odometry Topic。
**用法**：Input Format: subscriber_name topic_name out_path

#### savePathTopic.py
用于保存ROS节点发布的Path Topic。
**用法**：Input Format: topic_name out_path

#### savePointStamped.py
用于保存ROS节点发布的PointStamped Topic。
**用法**：Input Format: subscriber_name topic_name out_path

#### saveTFMessage.py
用于保存ROS节点发布的TFMessage Topic。
**用法**：Input Format: subscriber_name topic_name out_path

#### saveTransformStamped.py
用于保存ROS节点发布的TransformStamped Topic。
**用法**：Input Format: subscriber_name topic_name out_path

#### d435i_stage1_data_collection.py
用于利用D435i自动采集数据，包括：RGB影像、双目红外影像和IMU数据
**用法**：Input Format: python d435i_stage1_data_collection.py

#### d435i_stage2_postprocessing.py
用于对采集的ROS Bag格式的D435i数据进行解析，得到普通影像、IMU数据和时间戳文件。
**用法**：Input Format: input_bag_path

#### genBagFromImg.py
将某个影像序列转换成ROS Bag文件，时间单位默认为纳秒(10的9次方)。

* img_dir = sys.argv[1]   # 影像所在文件夹路径
* img_type = sys.argv[2]  # 影像类型
* topic_name = sys.argv[3]    # Topic名称
* bag_path = sys.argv[4]  # 输出Bag路径

#### genBagFromIMU.py
将IMU数据(EuRoC格式)转换成ROS Bag文件，时间单位默认为纳秒(10的9次方)。

* imu_path = sys.argv[1]  # IMU数据文件路径
* imu_topic_name = sys.argv[2]    # Topic名称
* bag_path = sys.argv[3]  # 输出Bag路径

#### genBagFromImgAndIMU.py
将影像序列与IMU数据转换并合并成一个ROS Bag文件，时间单位默认为纳秒(10的9次方)。

* img_dir = sys.argv[1]   # 影像所在文件夹路径
* img_type = sys.argv[2]  # 影像文件类型
* img_topic_name = sys.argv[3]    # 影像Topic名称
* imu_path = sys.argv[4]  # IMU文件路径
* imu_topic_name = sys.argv[5]    # IMU Topic名称
* bag_path = sys.argv[6]  # Bag文件输出路径

#### genBagFromLidar.py
将多个csv文件保存的点云数据转换成ROS Bag文件，时间单位默认为纳秒(10的9次方)。

* input_dir = sys.argv[1]   # csv所在文件夹路径
* out_dir = sys.argv[2]  # 输出结果文件夹路径

#### genBagFromCamImuLidar.py
将相机、IMU、LiDAR数据转换成一个ROS Bag文件，时间单位默认为纳秒(10的9次方)。

* input_dir = sys.argv[1]   # 输入数据文件夹路径
* out_dir = sys.argv[2]  # 输出结果文件夹路径

为了更加方便了解代码和测试，整理了一小部分数据放在百度云盘上。链接：https://pan.baidu.com/s/1Ip-oskL9LsZqJ5uJygek2A 
提取码：1517。
