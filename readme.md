changeBagFramerate.py
将Bag中的指定Topic转换到目标帧率并输出为新的Bag文件。
用法：Input Format: Input_bag_path topic_name1 target_rate1 [topic_name2 target_rate2 ... ] Output_bag_path

extractSelectedTopic.py
提取源Bag文件中的指定的Topic到新的Bag文件中。
用法：Input Format: Input_bag_path

parseBag.py
用于提取Bag文件中的Image类型Topic，并输出，包含多种模式：
	(1)Output preview video
	(2)Output general summary file
	(3)Output preview video and general summary file
	(4)Output frame images
	(5)Output frames of all image topics
	(6)Output preview video, summary file, frame images of all topics
	(7)Show frame images
Mode 1: 将选择的Image Topic生成一段预览视频
Mode 2: 输出一个包含包起止时间、时长、所有Topic基本信息、Image Topic的帧率、大小、数据类型的文本文件
Mode 3：同时生成预览视频和信息汇总文件
Mode 4：将选择的Image Topic拆解成帧影像
Mode 5：将所有的Image Topic都拆解成帧影像
Mode 6: 同时生成预览视频、信息汇总文件、以及将所有Image Topic拆解成帧影像
Mode 7: 只展示指定Topic
用法：Input Format: Input_bag_path

parseBagAuto.py
用于批量提取Bag文件中的Image类型Topic，并输出，包含多种模式：
	(1)Output preview video
	(2)Output general summary file
	(3)Output preview video and general summary file
	(4)Output frames of all image topics
	(5)Output preview video, summary file, frame images of all topics
Mode 1: 批量生成预览视频文件
Mode 2: 批量生成汇总信息文件
Mode 3: 同时批量生成预览视频文件和信息汇总文件
Mode 4: 批量将Bag中所有Image Topic拆解成帧影像
Mode 5: 同时批量生成预览视频文件、信息汇总文件，并将Bag中所有Image Topic拆解成帧影像
用法：Input Format: Input_bag_path