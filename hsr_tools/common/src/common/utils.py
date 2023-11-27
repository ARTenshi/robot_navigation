import rospy
import smach
import smach_ros
import os
#from common import SaveableObject

class TemporarySubscriber:
    def __init__(self, name, msg, cb, *args):

        self.name = name
        self.msg = msg
        self.cb = cb
        self.args = args

    def __enter__(self):
        self.sub = rospy.Subscriber(self.name, self.msg, self.cb, *self.args)
        return self.sub

    def __exit__(self, exctype, excval, traceback):
        self.sub.unregister()

class NearDoorContext(TemporarySubscriber):
    def __init__(self, topic="/laser_2d_pose",
                    door_poses=[],
                    ref_frame='map',
                    near_threshold = 1.5,
                    value_when_near = 0.15,
                    value_when_far = 0.35,
                    tf_buffer=None):
        import dynamic_reconfigure.client as reconf_client
        from geometry_msgs.msg import PoseWithCovarianceStamped
        TemporarySubscriber.__init__(self, topic, PoseWithCovarianceStamped,
                                        self.cb)
        self.obstacle_reconf = reconf_client.Client('obstacle_grid_mapper')
        self.ref_frame = ref_frame
        self.tf_buffer = tf_buffer
        self.door_poses = door_poses
        self.warned = False
        self.near_threshold = near_threshold
        self.value_when_near = value_when_near
        self.value_when_far = value_when_far

    def cb(self, msg):
        if self.ref_frame != msg.header.frame_id:
            if self.tf_buffer is None:
                if not self.warned:
                    rospy.logerr('When the frame of door positions and robot position are different, an instance of tf2_ros.Buffer is required.')
                    self.warned = True
                return
            import tf2_geometry_msgs
            msg = self.tf_buffer.transform(msg, self.ref_frame, rospy.Duration(1.))
        pos = msg.pose.pose.position
        rospy.loginfo("I'm at ({}, {})".format(pos.x, pos.y))
        value = self.value_when_far
        for (x,y), theta in self.door_poses:
            if (x-pos.x)**2+(y-pos.y)**2 < self.near_threshold**2:
                rospy.loginfo("Close to the door at ({}, {})".format(x, y))
                value = self.value_when_near
                break
        self.obstacle_reconf.update_configuration({'exception_potential_distance': value})

# class StateMachineWrapper:
#     def __init__(self, name, sm, resume=False, save_dir=None):
#         self._name = name
#         self._sm = sm
#         self._iss = smach_ros.IntrospectionServer(name, sm, '/SM_ROOT')
#         if save_dir is None:
#             save_dir = os.path.join('/tmp', '__state_machine_'+name+'_data__')
#         if not os.path.exists(save_dir):
#             os.makedirs(save_dir)
#         self._save_dir = save_dir
#         self._register_cb()
#         self._current_state_path = []
#         self._current_state = SaveableObject('current_state', save_dir=save_dir)
#         self._userdata = SaveableObject('userdata', {}, save_dir=save_dir)
#         self._saveable_topics = []
#         if resume:
#             if self._current_state.value is not None:
#                 self.set_initial_state(self._current_state.value)
#             self.update_userdata(self._userdata.value)
        
#     def _register_cb(self):
#         def cb(ud, states, append):
#             if append:
#                 self._current_state_path = self._current_state_path + [states[0]]
#             else:
#                 self._current_state_path = self._current_state_path[:-1] + [states[0]]
#             self._current_state.value = '/'.join(self._current_state_path)
#             ud = self.get_userdata()
#             print(ud)
#             self._userdata.value = ud
#         def do_reccurent(sm):
#             if isinstance(sm, smach.Container):
#                 sm.register_start_cb(cb, [True])
#                 sm.register_transition_cb(cb, [False])
#                 for s in sm.get_children():
#                     do_reccurent(sm[s])
#         do_reccurent(self._sm)
        
#     def get_userdata(self):
#         def do_reccurent(sm, name, ud_dict):
#             if isinstance(sm, smach.Container):
#                 ud_dict[name] = dict(sm.userdata)
#                 for s in sm.get_children():
#                     do_reccurent(sm[s], name+'/'+s, ud_dict)
#         ud_dict = {}
#         do_reccurent(self._sm, self._name, ud_dict)
#         return ud_dict

#     def update_userdata(self, ud_dict):
#         def do_reccurent(sm, name, ud_dict):
#             if isinstance(sm, smach.Container):
#                 if name in ud_dict:
#                     for k, v in ud_dict[name].items():
#                         sm.userdata[k] = v
#                 for s in sm.get_children():
#                     do_reccurent(sm[s], name+'/'+s, ud_dict)
#         do_reccurent(self._sm, self._name, ud_dict)

#     def add_saveable_topic(self, save_topic, msg_type, restore_topic):
#         v = (save_topic, msg_type, restore_topic,
#              SaveableObject(save_topic.replace('/', '__'), save_dir=self._save_dir))
#         self._saveable_topics.append(v)

#     def set_initial_state(self, state):
#         if not state:
#             return
#         def do_reccurent(sm, path):
#             sm.set_initial_state([path[0]])
#             if len(path) > 1:
#                 child = sm.get_children()[path[0]]
#                 do_reccurent(child, path[1:])
#         path = state.split(os.path.sep)
#         do_reccurent(self._sm, path)

#     @property
#     def current_state(self):
#         return self._current_state.value

#     def execute(self):
#         def cb(msg, obj):
#             obj.value = msg
#         self._iss.start()
#         subs = []
#         pubs = []
#         for save_topic, msg_type, restore_topic, obj in self._saveable_topics:
#             if obj.value is not None:
#                 pub = rospy.Publisher(restore_topic, msg_type, queue_size=1, latch=True)
#                 pub.publish(obj.value)
#                 pubs.append(pub)
#             subs.append(rospy.Subscriber(save_topic, msg_type, cb, obj))
#         self._sm.execute()
#         for sub in subs:
#             sub.unregister()

#         self._current_state_path = []
#         self._current_state.value = '/'.join(self._current_state_path)
#         ud = self.get_userdata()
#         print(ud)
#         self._userdata.value = ud

#         self._iss.stop()

def default_accept_object_fn(userdata, obj):
    depth = obj.center.point.z
    return depth > .3 and depth < 1.5

from geometry_msgs.msg import PoseStamped, PointStamped
_DEFAULT_TIMEOUT_TRANSFORMATION = 1.0
def transform_geometry_pose_stamped(tf2_ros_buffer, geometry_pose_stamped_from, frame_id_to, timeout=rospy.Duration(_DEFAULT_TIMEOUT_TRANSFORMATION)):
    __geometry_pose_stamped_to = PoseStamped()
    try:
        __geometry_pose_stamped_to = tf2_ros_buffer.transform(geometry_pose_stamped_from, 'base_footprint', timeout)
        __geometry_pose_stamped_to = tf2_ros_buffer.transform(__geometry_pose_stamped_to, frame_id_to, timeout)
    except Exception as e:
        rospy.logerr( 'Exception! transform_geometry_pose_stamped(): tf2_ros_buffer.transform()[%s]' % (str(e)))
    return __geometry_pose_stamped_to

def transform_geometry_point_stamped(tf2_ros_buffer, geometry_point_stamped_from, frame_id_to, timeout=rospy.Duration(_DEFAULT_TIMEOUT_TRANSFORMATION)):
    __geometry_point_stamped_to = PointStamped()
    try:
        __geometry_point_stamped_to = tf2_ros_buffer.transform(geometry_point_stamped_from, 'base_footprint', timeout)
        __geometry_point_stamped_to = tf2_ros_buffer.transform(__geometry_point_stamped_to, frame_id_to, timeout)
    except Exception as e:
        rospy.logerr( 'Exception! transform_geometry_point_stamped(): tf2_ros_buffer.transform()[%s]' % (str(e)))
    return __geometry_point_stamped_to
