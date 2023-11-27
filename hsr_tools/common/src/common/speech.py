import rospy
import json
import urllib
from datetime import datetime
import time

SERVER_URL = 'http://192.168.100.2:8000/data-en.json'
TIME_FORMAT = '%Y-%m-%dT%H:%M:%S.%f'

class RospeexClient:
    def __init__(self):
        pass

    def next_speech(self, timeout=None, t0=None):
        start = datetime.now()
        if t0 is None:
            t0 = start
        while not rospy.is_shutdown():
            req = urllib.Request(SERVER_URL)
            response = urllib.urlopen(req).read()
            contents = json.loads(response)
            t = datetime.strptime(contents[0]['time'], TIME_FORMAT)
            if t > t0:
                return contents[0]['transcription']
            if timeout and (datetime.now() - start).total_seconds() > timeout:
                break
            time.sleep(.5)
        raise Exception('Timeout while waiting for the speech recognition.')

class Console:
    def __init__(self):
        pass

    def next_speech(self, timeout=None):
        return raw_input('Type command here: ')

    def say(self, text):
        print(text)

class DefaultTTS:
    def __init__(self):
        import actionlib
        from tmc_msgs.msg import TalkRequestAction
        self.client = actionlib.SimpleActionClient('/talk_request_action', TalkRequestAction)
        self.client.wait_for_server(rospy.Duration(1.))
        #from tmc_msgs.msg import Voice
        #self.talk_pub = rospy.Publisher('/talk_request', Voice, queue_size=10)

    def say(self, text, wait_result=True):
        from tmc_msgs.msg import TalkRequestGoal
        goal = TalkRequestGoal()
        goal.data.language = 1   # 0: Japanese, 1: English
        goal.data.sentence = text
        self.client.send_goal(goal)
        if wait_result:
            self.client.wait_for_result()
        #from tmc_msgs.msg import Voice
        #voice = Voice()
        #voice.language = 1
        #voice.sentence = text
        #self.talk_pub.publish(voice)
