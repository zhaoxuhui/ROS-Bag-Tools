# coding=utf-8
import os
import sys


def findAllFiles(root_dir, filter):
    """
    在指定目录查找指定类型文件

    :param root_dir: 查找目录
    :param filter: 文件类型
    :return: 路径、名称、文件全路径

    """

    print("Finding files ends with \'" + filter + "\' ...")
    separator = os.path.sep
    paths = []
    names = []
    files = []
    for parent, dirname, filenames in os.walk(root_dir):
        for filename in filenames:
            if filename.endswith(filter):
                paths.append(parent + separator)
                names.append(filename)
    for i in range(paths.__len__()):
        files.append(paths[i] + names[i])
    print(names.__len__().__str__() + " files have been found.")
    paths.sort()
    names.sort()
    files.sort()
    return paths, names, files


if __name__ == "__main__":
    img_dir = sys.argv[1]
    save_path = img_dir + os.path.sep + "timestamps.txt"

    if len(sys.argv) == 2:
        img_type = ".jpg"
    else:
        img_type = sys.argv[2]

    paths, names, files = findAllFiles(img_dir, img_type)

    fout = open(save_path, "w")
    for i in range(len(names)):
        out_name = names[i].split(img_type)[0]
        print(out_name)
        fout.write(out_name + "\n")
    fout.close()
    print("Timestamp file has been saved at:" + save_path)
