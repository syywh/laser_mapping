import os
import shutil

DATADIR_2 = '/media/dxq/D/data/bag/zjg/merge/frames'
DATADIR_merge = '/media/dxq/ZHUOYANG/zjg_frames'

num_submap_1 = 0
num_submap_2 = 266

for i in range(0, num_submap_2, 1):

    source_path = DATADIR_2+'/'+str(i);
    target_path = os.path.join(DATADIR_merge, str(i+num_submap_1))
    if not os.path.exists(target_path):
        os.mkdir(target_path)


    print(source_path)

    source = os.path.join(source_path, 'GlobalPose.txt')
    target = os.path.join(target_path, 'GlobalPose.txt')
    print(str(i))

    shutil.copy(source, target)

    #print(source_path)
    #print(target_path)
    #source = source_path + '/gps.xml'
    #target = target_path + '/gps.xml'
    #print(source)
    #print(target)
    #shutil.copy(source, target)


