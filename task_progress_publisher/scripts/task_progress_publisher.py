import rospy

import requests
from std_msgs.msg import Empty, Bool


class TaskProgressPublisher:
    
    def __init__(self):
        self.task_id = rospy.get_param("task_id")
        self.app_token = rospy.get_param("app_token")
        self.app_token_key = rospy.get_param("app_token_key")
        self.task_finished_url = rospy.get_param("task_finished_url")

        rospy.Subscriber("scenario_finished", Bool, self.finished_task_callback)
        rospy.Subscriber("training_finished", Empty, self.finished_task_callback)

    def finished_task_callback(self, _):
        requests.post(
            self.task_finished_url, 
            json={ "taskId": self.task_id },
            headers={ self.app_token_key: self.app_token }
        )


if __name__ == "__main__":
    rospy.init_node("task_progress_publisher")

    task_progress_publihser = TaskProgressPublisher()

    while not rospy.is_shutdown():
        rospy.spin()