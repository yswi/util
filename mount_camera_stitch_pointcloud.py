#!/usr/bin/env python
import rospy
import tf2_ros as tf
# import tf as tf
import copy
import threading

from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, Pose, PoseWithCovarianceStamped
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray


def transform_from_camera(camera):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = detection.pose.header.frame_id
    t.child_frame_id = 'tag_{}'.format(tag_id)
    t.transform.translation.x = camera.pose.x
    t.transform.translation.y = camera.pose.x
    t.transform.translation.z = camera.pose.x
    t.transform.rotation.x = camera.orientation.x
    t.transform.rotation.y = camera.orientation.y
    t.transform.rotation.z = camera.orientation.z
    t.transform.rotation.w = camera.orientation.w
    return t

def _broadcast_tf(tf_i):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = tf_i['frame_id']
    t.child_frame_id = tf_i['child_id']
    t.transform.translation.x = tf_i['x']
    t.transform.translation.y = tf_i['y']
    t.transform.translation.z = tf_i['z']
    t.transform.rotation.x = tf_i['qx']
    t.transform.rotation.y = tf_i['qy']
    t.transform.rotation.z = tf_i['qz']
    t.transform.rotation.w = tf_i['qw']
    .tf_broadcaster.sendTransform(t)

class MountedCameraTFFusion(object):
    def __init__(self, parent_frame_name=None):
        self.tf_buffer = tf.BufferCore()
        self.tf_brodacaster = tf.TransformBroadcaster()
        self.subscribers = self._get_subscribers()
        self.pointcloud = {}

    def transform_from_camera(self, panda_base_id, camera, pos_idx):
        '''
            panda_base = base Name
            camera = a class of e-e pos/orientation
            pos_idx = pose index
        '''
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = panda_base_id
        t.child_frame_id = 'tag_{}'.format(pos_idx)
        t.transform.translation.x = camera.pose.x
        t.transform.translation.y = camera.pose.x
        t.transform.translation.z = camera.pose.x
        t.transform.rotation.x = camera.orientation.x
        t.transform.rotation.y = camera.orientation.y
        t.transform.rotation.z = camera.orientation.z
        t.transform.rotation.w = camera.orientation.w
        self.tf_buffer.set_transform(t, 'default_authority')

        ctr = self.tf_buffer.lookup_transform_core(t.header.frame_id, t.child_frame_id , rospy.Time(0))
        calib_frame_tr = copy.deepcopy(ctr)



    def add_pointcloud(self):




class CameraApriltagTFFusion(object):
    """
    This class listens to topics /tag_detections_i contained the detected apriltags.
    Then, it broadcasts the position of the cameras with respect to the common tag, which we assume fixed.
    """
    def __init__(self, continuous=False, tag_id=None, parent_frame_name=None):
        """
        :param continuous: <bool>
            * If True, it will keep updating the position of the cameras with respect to the detected tag.
            * If False, the reading is only done once, and it keeps broadcasting the same tf.
        :param tag_id: <int> id of the tag used to calibrate
        :param parent_frame_name: <str> Name of the frame to set as parent for the detected tags. If None, it is the same as calibration_frame
        """
        self.tag_id = tag_id
        self.continuous = continuous
        self.calibration_frame_name = 'calibration_frame'
        if parent_frame_name is None:
            self.broadcast_parent_frame_name = self.calibration_frame_name
        else:
            self.broadcast_parent_frame_name = parent_frame_name
        self.lock = threading.Lock()
        rospy.init_node('camera_tf_fusion')
        self.tf_buffer = tf.BufferCore() # provides coordinate transforms between any two frames in a system
        self.transformer = tf.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.detections = {} # stores the detections -- (shared result)
        self.tfs = {}
        self.subscribers = self._get_subscribers()
        self.broadcast_tfs()
        rospy.spin()

    def broadcast_tfs(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            try:
                self._update_tfs()
                for camera_id, tf_i in self.tfs.items():
                    self._broadcast_tf(tf_i)
            except:
                continue
            rate.sleep()

    def _broadcast_tf(self, tf_i):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = tf_i['frame_id']
        t.child_frame_id = tf_i['child_id']
        t.transform.translation.x = tf_i['x']
        t.transform.translation.y = tf_i['y']
        t.transform.translation.z = tf_i['z']
        t.transform.rotation.x = tf_i['qx']
        t.transform.rotation.y = tf_i['qy']
        t.transform.rotation.z = tf_i['qz']
        t.transform.rotation.w = tf_i['qw']
        self.tf_broadcaster.sendTransform(t)

    def _get_transform_from_detection(self, detection, tag_id=None):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = detection.pose.header.frame_id
        if tag_id is None:
            t.child_frame_id = 'tag_{}'.format(detection.id[0])
        else:
            t.child_frame_id = 'tag_{}'.format(tag_id)
        t.transform.translation.x = detection.pose.pose.pose.position.x
        t.transform.translation.y = detection.pose.pose.pose.position.y
        t.transform.translation.z = detection.pose.pose.pose.position.z
        t.transform.rotation.x = detection.pose.pose.pose.orientation.x
        t.transform.rotation.y = detection.pose.pose.pose.orientation.y
        t.transform.rotation.z = detection.pose.pose.pose.orientation.z
        t.transform.rotation.w = detection.pose.pose.pose.orientation.w
        return t

    def _get_subscribers(self):
        # this listents to all topic and returns a list of subscribers for the topics /tag_detections_i
        # - Get all topics:
        all_topics = rospy.get_published_topics()
        # - Filter out all topics not /tag_detections_i
        detected_topics = [t[0] for t in all_topics if 'tag_detections' in t[0]]
        detected_topics = [t for t in detected_topics if 'image' not in t]# remove image topics
        # - Create a Subscriber for each topic
        subscribers = []
        for i, topic in enumerate(detected_topics):
            camera_id = topic.split('tag_detections_')[-1]
            subscriber_i = rospy.Subscriber(topic, AprilTagDetectionArray, self._update_detections, camera_id)
            subscribers.append(subscriber_i)
        return subscribers

    def _update_detections(self, msg, indx):
        self.lock.acquire()
        try:
            self.detections[indx] = msg.detections
        finally:
            self.lock.release()

    def _update_tfs(self):
        self.lock.acquire()

        for camera_id, detections in self.detections.items():
            for detection in detections:
                if self.tag_id in detection.id:
                    pose = detection.pose # PoseWithCovarianceStampted
                    # get the transformation from tag_id to camera_i_link
                    t = self._get_transform_from_detection(detection, tag_id=self.tag_id) # transform stamped
                    self.tf_buffer.set_transform(t, 'default_authority')
                    ctr = self.tf_buffer.lookup_transform_core('tag_{}'.format(self.tag_id), 'camera_{}_link'.format(camera_id), rospy.Time(0))
                    # pos, rot = b[0], b[1]
                    # pack the transform
                    # Calibration Frame Transform:
                    calib_frame_tr = copy.deepcopy(ctr)
                    calib_frame_tr.header.frame_id = self.calibration_frame_name
                    self.tf_buffer.set_transform(calib_frame_tr, 'default_authority')

                    ltr = self.tf_buffer.lookup_transform_core(self.broadcast_parent_frame_name, 'camera_{}_link'.format(camera_id), rospy.Time(0))
                    tf_i = {}
                    tf_i['frame_id'] = self.broadcast_parent_frame_name
                    tf_i['child_id'] = 'camera_{}_link'.format(camera_id)
                    tf_i['x'] = ltr.transform.translation.x
                    tf_i['y'] = ltr.transform.translation.y
                    tf_i['z'] = ltr.transform.translation.z
                    tf_i['qx'] = ltr.transform.rotation.x
                    tf_i['qy'] = ltr.transform.rotation.y
                    tf_i['qz'] = ltr.transform.rotation.z
                    tf_i['qw'] = ltr.transform.rotation.w
                    self.tfs[camera_id] = tf_i



if __name__ == '__main__':

    # camera_apriltag_fusion = CameraApriltagTFFusion(continuous=True, tag_id=0)
    camera_apriltag_fusion = CameraApriltagTFFusion(continuous=False, tag_id=0, parent_frame_name='med_base') # Camera Calibration
