import rclpy
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped

import paho.mqtt.client as mqtt

# 전역 변수: 현재 goal 핸들을 저장
current_goal_handle = None

# ROS2 초기화 및 노드 생성
rclpy.init()
action_node = rclpy.create_node('follow_waypoints_client')
action_client = ActionClient(action_node, FollowWaypoints, '/follow_waypoints')

# ---------------------- ROS2 액션 콜백 함수들 ----------------------

def feedback_callback(feedback_msg):
    current_waypoint = feedback_msg.feedback.current_waypoint
    print('Feedback - current_waypoint: {0}'.format(current_waypoint))

def get_result_callback(future):
    result = future.result()
    print('Result: {0}'.format(result))
    rclpy.shutdown()

def goal_response_callback(future):
    global current_goal_handle
    current_goal_handle = future.result()
    if not current_goal_handle.accepted:
        print('Goal rejected')
        return

    print('Goal accepted')
    get_result_future = current_goal_handle.get_result_async()
    get_result_future.add_done_callback(get_result_callback)

def cancel_done_callback(future):
    cancel_response = future.result()
    print("Cancel request completed: ", cancel_response)

# ---------------------- MQTT 콜백 함수들 ----------------------

def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT broker with result code " + str(rc))
    # "robot/stop" 토픽 구독
    client.subscribe("robot/stop")

def on_message(client, userdata, msg):
    global current_goal_handle
    payload = msg.payload.decode().strip().upper()
    print("Received MQTT message:", payload)
    if payload == "STOP":
        if current_goal_handle is not None:
            print("Cancelling goal due to MQTT STOP message")
            cancel_future = current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(cancel_done_callback)
        else:
            print("No active goal to cancel")

# ---------------------- MQTT 클라이언트 설정 ----------------------

mqtt_client = mqtt.Client()
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.connect("localhost", 1883, 60)  # 브로커 주소/포트는 필요에 따라 수정
mqtt_client.loop_start()  # MQTT 네트워크 루프 시작 (백그라운드 스레드)

# ---------------------- 액션 서버 연결 및 goal 전송 ----------------------

if not action_client.wait_for_server(timeout_sec=10.0):
    print('Action server not available!')
else:
    print("Action server available!")

# goal로 보낼 PoseStamped 리스트 구성
goal_pose_list = []

goal_pose1 = PoseStamped()
goal_pose1.header.frame_id = 'map'
goal_pose1.pose.position.x = 1.51
goal_pose1.pose.position.y = 0.36
goal_pose1.pose.orientation.w = 0.707
goal_pose1.pose.orientation.z = 0.707
goal_pose_list.append(goal_pose1)

goal_pose2 = PoseStamped()
goal_pose2.header.frame_id = 'map'
goal_pose2.pose.position.x = 0.98
goal_pose2.pose.position.y = 6.05
goal_pose2.pose.orientation.w = 0.707
goal_pose2.pose.orientation.z = 0.707
goal_pose_list.append(goal_pose2)

goal_pose3 = PoseStamped()
goal_pose3.header.frame_id = 'map'
goal_pose3.pose.position.x = 3.95
goal_pose3.pose.position.y = 5.48
goal_pose3.pose.orientation.w = 0.707
goal_pose3.pose.orientation.z = 0.707
goal_pose_list.append(goal_pose3)

goal_pose3 = PoseStamped()
goal_pose3.header.frame_id = 'map'
goal_pose3.pose.position.x = 3.95
goal_pose3.pose.position.y = 1.06
goal_pose3.pose.orientation.w = 0.707
goal_pose3.pose.orientation.z = 0.707
goal_pose_list.append(goal_pose3)

goal_msg = FollowWaypoints.Goal()
goal_msg.poses = goal_pose_list

send_goal_future = action_client.send_goal_async(goal_msg, feedback_callback=feedback_callback)
send_goal_future.add_done_callback(goal_response_callback)

# ROS2 노드 spin: 액션 클라이언트 노드와 MQTT 클라이언트는 동시에 동작함
rclpy.spin(action_node)
