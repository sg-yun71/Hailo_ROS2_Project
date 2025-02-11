import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import os
import argparse
import cv2
import time
import hailo
import setproctitle
from hailo_rpi_common import (
    get_default_parser,
    QUEUE,
    get_caps_from_pad,
    get_numpy_from_buffer,
    GStreamerApp,
    app_callback_class,
)

# MQTT 라이브러리 임포트
import paho.mqtt.client as mqtt

# MQTT 설정
MQTT_BROKER = "localhost"    # MQTT 브로커 주소 (필요에 따라 수정)
MQTT_PORT = 1883             # MQTT 브로커 포트
MQTT_TOPIC = "robot/stop"    # 발행할 토픽

# MQTT 클라이언트 초기화 (한번만 생성하여 사용)
mqtt_client = mqtt.Client()
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
mqtt_client.loop_start()  # background에서 MQTT 네트워크 루프 실행

# -----------------------------------------------------------------------------------------------
# 사용자 정의 클래스 (콜백에서 사용할 사용자 데이터)
# -----------------------------------------------------------------------------------------------
class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()
        self.new_variable = 42  # 예제용 변수
        self.person_detected = False  # 직전 프레임에서 사람 감지 여부

    def new_function(self):
        # 예제용 함수
        return "The meaning of life is:"

# -----------------------------------------------------------------------------------------------
# 사용자 정의 콜백 함수
# -----------------------------------------------------------------------------------------------
def app_callback(pad, info, user_data):
    # GstBuffer 가져오기
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    # 프레임 카운팅 (예제)
    user_data.increment()
    string_to_print = f"Frame count: {user_data.get_count()}\n"

    # pad로부터 caps, width, height 얻기
    format, width, height = get_caps_from_pad(pad)

    # 영상 프레임 추출 (필요 시)
    frame = None
    if user_data.use_frame and format is not None and width is not None and height is not None:
        frame = get_numpy_from_buffer(buffer, format, width, height)

    # ROI에서 객체 검출
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

    # 사람 감지 여부 및 메시지 처리
    person_found = False
    detection_count = 0
    for detection in detections:
        label = detection.get_label()
        confidence = detection.get_confidence()
        if label == "person":
            string_to_print += f"Detection: {label} {confidence:.2f}\n"
            detection_count += 1
            person_found = True

    # MQTT 메시지 발행: 이전 프레임에서는 사람이 없었으나, 이번에 감지되면 발행
    if person_found and not user_data.person_detected:
        print("[MQTT] Person detected, sending STOP message")
        mqtt_client.publish(MQTT_TOPIC, "STOP")
    user_data.person_detected = person_found

    # 영상에 detection 정보 추가 (화면 디버깅용)
    if user_data.use_frame and frame is not None:
        cv2.putText(frame, f"Detections: {detection_count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"{user_data.new_function()} {user_data.new_variable}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        user_data.set_frame(frame)

    print(string_to_print)
    return Gst.PadProbeReturn.OK

# -----------------------------------------------------------------------------------------------
# Gstreamer Detection Application 클래스
# -----------------------------------------------------------------------------------------------
class GStreamerDetectionApp(GStreamerApp):
    def __init__(self, args, user_data):
        # 부모 클래스 초기화
        super().__init__(args, user_data)

        # Hailo 파라미터 설정 (모델에 맞게 수정)
        self.batch_size = 2
        self.network_width = 640
        self.network_height = 640
        self.network_format = "RGB"
        nms_score_threshold = 0.3
        nms_iou_threshold = 0.45

        # 새로운 postprocess so 파일의 경로 확인
        new_postprocess_path = os.path.join(self.current_path, '../resources/libyolo_hailortpp_post.so')
        if os.path.exists(new_postprocess_path):
            self.default_postprocess_so = new_postprocess_path
        else:
            self.default_postprocess_so = os.path.join(self.postprocess_dir, 'libyolo_hailortpp_post.so')

        # HEF 파일 경로 설정 (명령행 인자 또는 네트워크 타입에 따라)
        if args.hef_path is not None:
            self.hef_path = args.hef_path
        elif args.network == "yolov6n":
            self.hef_path = os.path.join(self.current_path, '../resources/yolov6n.hef')
        elif args.network == "yolov8s":
            self.hef_path = os.path.join(self.current_path, '../resources/yolov8s_h8l.hef')
        elif args.network == "yolox_s_leaky":
            self.hef_path = os.path.join(self.current_path, '../resources/yolox_s_leaky_h8l_mz.hef')
        else:
            assert False, "Invalid network type"

        # 사용자 정의 라벨 JSON 파일 (존재할 경우)
        if args.labels_json is not None:
            self.labels_config = f' config-path={args.labels_json} '
        else:
            self.labels_config = ''

        # 필수 postprocess so 파일 존재 확인
        if not os.path.exists(new_postprocess_path):
            print("New postprocess so file is missing. It is required to support custom labels. Check documentation for more information.")
            exit(1)

        # 사용자 정의 콜백 연결
        self.app_callback = app_callback
        self.thresholds_str = (f"nms-score-threshold={nms_score_threshold} "
                               f"nms-iou-threshold={nms_iou_threshold} "
                               f"output-format-type=HAILO_FORMAT_TYPE_FLOAT32")

        # 프로세스 이름 설정
        setproctitle.setproctitle("Hailo Detection App")
        self.create_pipeline()

    def get_pipeline_string(self):
        # 소스에 따라 GStreamer 파이프라인 문자열 생성 (여기서는 예제 코드 그대로 사용)
        if self.source_type == "rpi":
            source_element = (
                "libcamerasrc name=src_0 auto-focus-mode=2 ! "
                f"video/x-raw, format={self.network_format}, width=1536, height=864 ! "
                + QUEUE("queue_src_scale") +
                "videoscale ! "
                f"video/x-raw, format={self.network_format}, width={self.network_width}, height={self.network_height}, framerate=30/1 ! "
            )
        elif self.source_type == "usb":
            source_element = (
                f"v4l2src device={self.video_source} name=src_0 ! "
                "video/x-raw, width=640, height=480, framerate=30/1 ! "
            )
        else:
            source_element = (
                f"filesrc location={self.video_source} name=src_0 ! "
                + QUEUE("queue_dec264") +
                " qtdemux ! h264parse ! avdec_h264 max-threads=2 ! "
                " video/x-raw, format=I420 ! "
            )
        source_element += QUEUE("queue_scale")
        source_element += "videoscale n-threads=2 ! "
        source_element += QUEUE("queue_src_convert")
        source_element += "videoconvert n-threads=3 name=src_convert qos=false ! "
        source_element += f"video/x-raw, format={self.network_format}, width={self.network_width}, height={self.network_height}, pixel-aspect-ratio=1/1 ! "

        pipeline_string = (
            "hailomuxer name=hmux " +
            source_element +
            "tee name=t ! " +
            QUEUE("bypass_queue", max_size_buffers=20) +
            "hmux.sink_0 " +
            "t. ! " +
            QUEUE("queue_hailonet") +
            "videoconvert n-threads=3 ! " +
            f"hailonet hef-path={self.hef_path} batch-size={self.batch_size} {self.thresholds_str} force-writable=true ! " +
            QUEUE("queue_hailofilter") +
            f"hailofilter so-path={self.default_postprocess_so} {self.labels_config} qos=false ! " +
            QUEUE("queue_hmuc") +
            "hmux.sink_1 " +
            "hmux. ! " +
            QUEUE("queue_hailo_python") +
            QUEUE("queue_user_callback") +
            "identity name=identity_callback ! " +
            QUEUE("queue_hailooverlay") +
            "hailooverlay ! " +
            QUEUE("queue_videoconvert") +
            "videoconvert n-threads=3 qos=false ! " +
            QUEUE("queue_hailo_display") +
            f"fpsdisplaysink video-sink=fakesink name=hailo_display sync={self.sync} text-overlay={self.options_menu.show_fps} signal-fps-measurements=true "
        )
        print(pipeline_string)
        return pipeline_string

if __name__ == "__main__":
    user_data = user_app_callback_class()
    parser = get_default_parser()
    parser.add_argument("--network", default="yolov6n", choices=['yolov6n', 'yolov8s', 'yolox_s_leaky'], help="Which Network to use, default is yolov6n")
    parser.add_argument("--hef-path", default=None, help="Path to HEF file")
    parser.add_argument("--labels-json", default=None, help="Path to costume labels JSON file")
    args = parser.parse_args()
    app = GStreamerDetectionApp(args, user_data)
    app.run()
