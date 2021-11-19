# coding=utf-8
import os
import sys
import time


def checkPythonVersion():
    return float(sys.version[:3])

if __name__ == '__main__':
    head_str = "gnome-terminal -x bash -c "

    # step 1 启动相机
    step1_com = head_str+r"'roslaunch realsense2_camera rs_camera.launch'"
    print("==> Launching Intel Realsense D435i camera ...")
    os.system(step1_com)

    # step2 关闭红外发射器
    # python2, python3 input()函数不一样,需要分别处理
    python_v = checkPythonVersion()
    if python_v == 3:
        user_input = input("Reconfigure Intel Realsense D435i camera? [y]/n")
    else:
        user_input = raw_input("Reconfigure Intel Realsense D435i camera? [y]/n")
    if user_input == '' or user_input == 'y' or user_input == 'Y':
        step2_com = head_str+r"'rosrun rqt_reconfigure rqt_reconfigure'"
        print("==> Reconfiguring Intel Realsense D435i camera ...")
        os.system(step2_com)
    
        python_v = checkPythonVersion()
        if python_v == 3:
            configure_rst = input("Finished configuration? [y]/n")
        else:
            configure_rst = raw_input("Finished configuration? [y]/n")
        if configure_rst == '' or configure_rst == 'y' or configure_rst == 'Y':
            print("==> Reconfiguration finished.")

    # step3 可视化数据
    step3_1_com = head_str+r"'rqt_image_view /camera/color/image_raw'"
    step3_2_com = head_str+r"'rqt_image_view /camera/infra1/image_rect_raw'"
    step3_3_com = head_str+r"'rqt_image_view /camera/infra2/image_rect_raw'"
    os.system(step3_1_com)
    os.system(step3_2_com)
    os.system(step3_3_com)
    python_v = checkPythonVersion()
    if python_v == 3:
        user_input = input("Visualize images OK? [y]/n")
    else:
        user_input = raw_input("Visualize images OK? [y]/n")
    if user_input == '' or user_input == 'y' or user_input == 'Y':
        print("==> Visualization OK")

    # step4 录制数据
    bag_str = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())+".bag"
    step4_com = r"rosbag record /camera/color/image_raw /camera/imu /camera/infra1/image_rect_raw /camera/infra2/image_rect_raw -O "+bag_str
    python_v = checkPythonVersion()
    if python_v == 3:
        user_input = input("Start recording color, infra(left), infra(red) and IMU stream? [y]/n")
    else:
        user_input = raw_input("Start recording color, infra(left), infra(red) and IMU stream? [y]/n")
    if user_input == '' or user_input == 'y' or user_input == 'Y':
        print("==> Start recording ...")
        os.system(step4_com)