from multiprocessing import Process

from make_120_bvh.optitrack_120_bvh import OptitrackBVH
from make_120_bvh.xsens_120_bvh import XsensBVH


# Process version
class BVH120frame:
    def __init__(self):
        self.test = 0
        self.i = 0

    def run(self, result, xsens_bvh_queue, optitrack_bvh_queue, click_timestamp):    
        if 'xsens' in result:
            xsens_process = Process(target=XsensBVH().make_bvh, args=(xsens_bvh_queue, click_timestamp))
            xsens_process.start()
        if 'optitrack' in result:
            optitrack_process = Process(target=OptitrackBVH().make_bvh, args=(optitrack_bvh_queue, click_timestamp))
            optitrack_process.start()
