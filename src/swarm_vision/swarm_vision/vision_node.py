# flake8: noqa

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2
import time
import os
import io
import zipfile
import threading
import numpy as np
from datetime import datetime
from flask import Flask, Response, render_template_string, request, jsonify, send_file

from swarm_vision.detector import (
    COCO_CLASSES,
    YoloDetector,
    resolve_model_path,
    resolve_output_directory,
)

app = Flask(__name__)

detector = None

# متغیرهای سیستمی
cap = None
stream_quality = 50
inference_skip = 1
selected_classes = [32]      # پیش‌فرض: Sports Ball (توپ)
detect_all = False
digital_zoom = 1.0

recording_test = False
enable_video_record = True
current_test_name = ""
current_test_dir = ""
test_start_time = None
last_save_time = 0
latest_notification = "سیستم آماده پایش سوارم است."
target_currently_detected = False

latest_annotated_frame = None
frame_lock = threading.Lock()

video_writer = None
BASE_TEST_DIR = ''

ros_node = None
stop_event = threading.Event()
web_host = '0.0.0.0'
web_port = 5000

class SwarmVisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.declare_parameter('uav_id', 'UAV_1')
        self.declare_parameter('model_path', '')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('input_size', 320)
        self.declare_parameter('confidence', 0.45)
        self.declare_parameter('nms_threshold', 0.45)
        self.declare_parameter('inference_threads', 1)
        self.declare_parameter('output_directory', '')
        self.declare_parameter('command_interval', 1.0)
        self.declare_parameter('web_host', '0.0.0.0')
        self.declare_parameter('web_port', 5000)
        self.uav_id = self.get_parameter('uav_id').value
        self.command_interval = max(
            0.0, float(self.get_parameter('command_interval').value)
        )
        self.last_command_time = 0.0
        self.detection_enabled = False
        self.cmd_pub = self.create_publisher(String, '/swarm/vision_command', 10)
        self.trigger_subscription = self.create_subscription(
            String,
            '/swarm/vision_trigger',
            self.trigger_callback,
            10,
        )
        self.get_logger().info(f'نود بینایی پرنده {self.uav_id} روی تاپیک سراسری /swarm/vision_command فعال شد.')

    def trigger_callback(self, msg):
        """Enable selected COCO classes with START[:ids], or stop detection."""
        global selected_classes, detect_all, target_currently_detected

        payload = msg.data.strip().upper()
        if payload.startswith('START'):
            if ':' in payload:
                requested = []
                for value in payload.split(':', 1)[1].split(','):
                    value = value.strip()
                    if not value:
                        continue
                    try:
                        class_id = int(value)
                    except ValueError:
                        self.get_logger().warning(
                            f'Ignoring invalid class ID: {value}'
                        )
                        continue
                    if class_id in COCO_CLASSES:
                        requested.append(class_id)
                    else:
                        self.get_logger().warning(
                            f'Ignoring unknown COCO class ID: {class_id}'
                        )
                if requested:
                    selected_classes = requested
                    detect_all = False
            self.detection_enabled = True
            names = [COCO_CLASSES[value] for value in selected_classes]
            self.get_logger().info(
                f'Detection active for {selected_classes} ({names}).'
            )
        elif payload in ('STOP', 'DISABLE'):
            self.detection_enabled = False
            target_currently_detected = False
            self.get_logger().info('Detection stopped.')
        else:
            self.get_logger().warning(
                'Unknown vision trigger. Use START, START:32,0,29, or STOP.'
            )

    def publish_command(self, command_str):
        now = time.monotonic()
        if now - self.last_command_time < self.command_interval:
            return
        self.last_command_time = now
        msg = String()
        msg.data = f"{self.uav_id}:{command_str}"
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'مخابره به سوارم: {msg.data}')

def get_cpu_temp():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            temp = float(f.read()) / 1000.0
            return f"{temp:.1f}°C"
    except Exception:
        return "N/A"

def apply_digital_zoom(frame, zoom_factor):
    if zoom_factor <= 1.0:
        return frame
    h, w = frame.shape[:2]
    new_h, new_w = int(h / zoom_factor), int(w / zoom_factor)
    y1, y2 = (h - new_h) // 2, ((h - new_h) // 2) + new_h
    x1, x2 = (w - new_w) // 2, ((w - new_w) // 2) + new_w
    cropped = frame[y1:y2, x1:x2]
    return cv2.resize(cropped, (w, h), interpolation=cv2.INTER_LINEAR)

def run_onnx_inference(frame):
    global detector, selected_classes, detect_all
    if detector is None:
        return False, frame
    return detector.detect(
        frame,
        selected_classes=selected_classes,
        detect_all=detect_all,
    )

def vision_loop():
    global cap, digital_zoom, ros_node, target_currently_detected
    global recording_test, enable_video_record, video_writer, last_save_time, latest_notification
    global inference_skip, latest_annotated_frame
    
    prev_time = 0
    frame_count = 0
    cached_detected = False
    
    while not stop_event.is_set():
        if cap is None or not cap.isOpened():
            stop_event.wait(0.05)
            continue
            
        success, raw_frame = cap.read()
        if not success:
            stop_event.wait(0.02)
            continue
            
        frame = apply_digital_zoom(raw_frame, digital_zoom)
        frame_count += 1
        current_time = time.time()
        
        detection_enabled = (
            ros_node is not None and ros_node.detection_enabled
        )
        if detection_enabled and frame_count % inference_skip == 0:
            try:
                detected, annotated_frame = run_onnx_inference(frame)
            except Exception as error:
                if ros_node is not None:
                    ros_node.get_logger().error(
                        f'Vision inference failed: {error}'
                    )
                stop_event.wait(0.1)
                continue
            if ros_node is None or not ros_node.detection_enabled:
                # STOP may arrive while inference is running.
                detected = False
            cached_detected = detected
        else:
            detected = cached_detected if detection_enabled else False
            annotated_frame = frame.copy()
            if not detection_enabled:
                cached_detected = False
            
        target_currently_detected = detected
        
        # Detection is report-only. Flight actions are decided elsewhere.
        if detected and ros_node is not None:
            ros_node.publish_command("TARGET_DETECTED")

        fps = 1.0 / (current_time - prev_time) if (current_time - prev_time) > 0 else 0
        prev_time = current_time
        
        sys_clock = datetime.now().strftime("%H:%M:%S")
        test_timer_str = "00:00"
        if recording_test and test_start_time:
            elapsed = int(current_time - test_start_time)
            mins, secs = divmod(elapsed, 60)
            test_timer_str = f"{mins:02d}:{secs:02d}"

        cpu_temp = get_cpu_temp()
        cv2.putText(annotated_frame, f"FPS: {fps:.1f} | Temp: {cpu_temp} | Zoom: {digital_zoom:.1f}x", 
                    (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        
        timer_display = f"Time: {sys_clock}"
        if recording_test:
            timer_display += f" | Test: {test_timer_str}"
            cv2.circle(annotated_frame, (frame.shape[1] - 25, 25), 8, (0, 0, 255), -1)
            if enable_video_record:
                cv2.putText(annotated_frame, "REC VID", (frame.shape[1] - 110, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        cv2.putText(annotated_frame, timer_display, 
                    (15, frame.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        if recording_test:
            if enable_video_record and video_writer is not None:
                video_writer.write(annotated_frame)

            if detected:
                latest_notification = "🎯 هدف در کادر است! ارسال سیگنال سوارم..."
                if current_time - last_save_time >= 1.0:
                    last_save_time = current_time
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    filename = os.path.join(current_test_dir, f"img_{timestamp}.jpg")
                    threading.Thread(target=cv2.imwrite, args=(filename, annotated_frame.copy())).start()
            else:
                latest_notification = "در حال پایش منطقه... هدفی یافت نشد."

        with frame_lock:
            latest_annotated_frame = annotated_frame.copy()

def generate_frames():
    global latest_annotated_frame, stream_quality
    while not stop_event.is_set():
        with frame_lock:
            if latest_annotated_frame is None:
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
            else:
                frame = latest_annotated_frame.copy()
                
        ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), stream_quality])
        if not ret:
            time.sleep(0.03)
            continue
            
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        time.sleep(0.03)

HTML_PAGE = """
<!DOCTYPE html>
<html lang="fa" dir="rtl">
<head>
    <meta charset="UTF-8">
    <title>پنل پایش سوارم - نود ROS 2</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Tahoma, Arial, sans-serif; background: #0f111a; color: #e1e1e1; margin: 0; padding: 15px; text-align: center; }
        .main-container { display: flex; flex-wrap: wrap; justify-content: center; gap: 20px; max-width: 1350px; margin: auto; }
        .stream-container { flex: 1 1 620px; max-width: 700px; background: #000; border-radius: 10px; overflow: hidden; border: 1px solid #222; }
        img { width: 100%; height: auto; display: block; }
        .panel { flex: 1 1 350px; background: #1a1d2e; padding: 18px; border-radius: 10px; border: 1px solid #2c304d; text-align: right; }
        .box { background: #23273e; padding: 12px; border-radius: 6px; margin-bottom: 12px; }
        .notif-box { background: #132438; border-right: 4px solid #00a8ff; color: #bde0fe; padding: 10px; font-weight: bold; }
        button { padding: 8px 14px; font-size: 13px; border: none; border-radius: 5px; cursor: pointer; color: #fff; font-family: inherit; font-weight: bold; }
        .btn-start { background: #27ae60; width: 100%; }
        .btn-stop { background: #c0392b; width: 100%; }
        .btn-dl { background: #2980b9; padding: 4px 8px; font-size: 12px; }
        input, select { padding: 7px; border-radius: 4px; border: 1px solid #444; background: #111; color: #fff; font-family: inherit; }
        label { font-size: 12px; color: #bbb; display: block; margin-top: 5px; }
        .test-list { max-height: 120px; overflow-y: auto; list-style: none; padding: 0; margin: 5px 0 0 0; }
        .test-item { display: flex; justify-content: space-between; align-items: center; padding: 5px; border-bottom: 1px solid #333; font-size: 12px; }
        .checkbox-container { max-height: 140px; overflow-y: auto; background: #111; padding: 8px; border-radius: 5px; border: 1px solid #444; margin-top: 5px; }
        .checkbox-item { display: flex; align-items: center; gap: 8px; font-size: 12px; margin-bottom: 4px; }
        .checkbox-item input { width: auto; margin: 0; }
        .toggle-box { background: #182333; border: 1px solid #254466; padding: 8px; border-radius: 5px; margin-top: 8px; }
    </style>
</head>
<body onclick="initAudio()">
    <h2>🚁 پنل مدیریت ویژن سوارم (تاپیک سراسری)</h2>
    
    <div class="main-container">
        <div class="stream-container">
            <img src="/video_feed">
        </div>

        <div class="panel">
            <div class="box notif-box" id="notif-text">سیستم آماده است.</div>

            <div class="box">
                <strong>🔍 کنترل زوم دیجیتال:</strong>
                <label>ضریب زوم: <span id="zval" style="color: #00d2d3; font-weight: bold;">1.0x</span></label>
                <input type="range" min="1.0" max="4.0" step="0.1" value="1.0" style="width:100%" onchange="setZoom(this.value)" oninput="document.getElementById('zval').innerText=parseFloat(this.value).toFixed(1)+'x'">
            </div>

            <div class="box">
                <strong>⚡ روانی استریم و مصرف پردازنده:</strong>
                <label>Skip Frame اینفرنس: <span id="skval">هر 1 فریم</span></label>
                <input type="range" min="1" max="4" value="1" style="width:100%" onchange="setSkip(this.value)" oninput="document.getElementById('skval').innerText='هر ' + this.value + ' فریم'">

                <label>کیفیت استریم شبکه: <span id="qv" style="color: #f39c12;">50%</span></label>
                <input type="range" min="15" max="95" value="50" style="width:100%" onchange="setQuality(this.value)" oninput="document.getElementById('qv').innerText=this.value+'%'">
            </div>

            <div class="box">
                <label>انتخاب کلاس‌های مورد نظر برای کشف هدف:</label>
                <div class="checkbox-item">
                    <input type="checkbox" id="chk-all" onchange="toggleAllClasses(this.checked)">
                    <label for="chk-all" style="color:#2ecc71; margin:0;">همه اشیاء (Detect All)</label>
                </div>
                <div class="checkbox-container" id="class-container">
                    {% for cid, cname in classes.items() %}
                    <div class="checkbox-item">
                        <input type="checkbox" class="cls-box" value="{{ cid }}" id="cls_{{ cid }}" {% if cid in active_cls %}checked{% endif %} onchange="updateSelectedClasses()">
                        <label for="cls_{{ cid }}" style="margin:0;">{{ cid }}: {{ cname }}</label>
                    </div>
                    {% endfor %}
                </div>
            </div>

            <div class="box">
                <label>مدیریت تست پرواز گروهی:</label>
                <div id="start-controls">
                    <input type="text" id="test-name" placeholder="نام تست پرواز (مثلا SWARM_T1)" style="width: calc(100% - 16px);">
                    <div class="toggle-box checkbox-item">
                        <input type="checkbox" id="chk-video-rec" checked onchange="toggleVideoRec(this.checked)">
                        <label for="chk-video-rec" style="color:#3498db; margin:0; cursor:pointer;">ضبط ویدیوی کل تست</label>
                    </div>
                    <button class="btn-start" style="margin-top: 10px;" onclick="startTest()">شروع تست و تایمر</button>
                </div>
                <div id="stop-controls" style="display: none;">
                    <p id="active-test-label" style="color: #2ecc71; margin: 4px 0;"></p>
                    <button class="btn-stop" onclick="stopTest()">پایان تست</button>
                </div>
            </div>

            <div class="box">
                <strong>📁 دانلود فایل‌های ماموریت:</strong>
                <ul class="test-list" id="test-list-ui">
                    <li>درحال بارگذاری...</li>
                </ul>
            </div>
        </div>
    </div>

    <script>
        let audioCtx = null, lastBeepTime = 0;
        function initAudio() { if (!audioCtx) audioCtx = new (window.AudioContext || window.webkitAudioContext)(); }
        function playBeep() {
            initAudio(); let now = Date.now();
            if (now - lastBeepTime < 1000) return; lastBeepTime = now;
            if (audioCtx && audioCtx.state === 'running') {
                let osc = audioCtx.createOscillator(), gain = audioCtx.createGain();
                osc.type = 'sine'; osc.frequency.value = 880;
                gain.gain.setValueAtTime(0.15, audioCtx.currentTime);
                gain.gain.exponentialRampToValueAtTime(0.001, audioCtx.currentTime + 0.25);
                osc.connect(gain); gain.connect(audioCtx.destination);
                osc.start(); osc.stop(audioCtx.currentTime + 0.25);
            }
        }
        function updateNotif() {
            fetch('/get_status').then(r => r.json()).then(d => {
                document.getElementById('notif-text').innerText = d.notification;
                if(d.recording) {
                    document.getElementById('start-controls').style.display = 'none';
                    document.getElementById('stop-controls').style.display = 'block';
                    document.getElementById('active-test-label').innerText = 'تست فعال: ' + d.test_name;
                } else {
                    document.getElementById('start-controls').style.display = 'block';
                    document.getElementById('stop-controls').style.display = 'none';
                }
                if (d.detected) playBeep();
            });
        }
        setInterval(updateNotif, 500);

        function loadTests() {
            fetch('/list_tests').then(r => r.json()).then(folders => {
                let html = '';
                if(folders.length === 0) html = '<li style="color:#777;">تستی ثبت نشده است.</li>';
                else folders.forEach(f => { html += `<li class="test-item"><span>${f}</span><a href="/download_test/${f}"><button class="btn-dl">دانلود ZIP</button></a></li>`; });
                document.getElementById('test-list-ui').innerHTML = html;
            });
        }
        setInterval(loadTests, 5000); loadTests();

        function setZoom(val) { fetch('/set_zoom?zoom=' + val); }
        function setSkip(val) { fetch('/set_skip?skip=' + val); }
        function setQuality(val) { fetch('/set_quality?quality=' + val); }
        function updateSelectedClasses() {
            let selected = [];
            document.querySelectorAll('.cls-box:checked').forEach(cb => selected.push(cb.value));
            fetch('/set_classes', { method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({classes: selected, all: false}) });
        }
        function toggleAllClasses(isAll) {
            fetch('/set_classes', { method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({classes: [], all: isAll}) });
            document.querySelectorAll('.cls-box').forEach(cb => cb.disabled = isAll);
        }
        function toggleVideoRec(enabled) { fetch('/set_video_rec?enable=' + (enabled ? 1 : 0)); }
        function startTest() {
            initAudio(); var name = document.getElementById('test-name').value.trim();
            if(!name) { alert('لطفاً نام تست را وارد کنید.'); return; }
            fetch('/start_test?name=' + encodeURIComponent(name)).then(() => loadTests());
        }
        function stopTest() { fetch('/stop_test').then(() => loadTests()); }
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_PAGE, classes=COCO_CLASSES, active_cls=selected_classes)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/get_status')
def get_status():
    return jsonify({
        "notification": latest_notification,
        "recording": recording_test,
        "test_name": current_test_name,
        "detected": target_currently_detected
    })

@app.route('/set_zoom')
def set_zoom():
    global digital_zoom
    digital_zoom = max(1.0, min(float(request.args.get('zoom', 1.0)), 4.0))
    return jsonify(status="ok", zoom=digital_zoom)

@app.route('/set_classes', methods=['POST'])
def set_classes():
    global selected_classes, detect_all
    data = request.get_json()
    detect_all = data.get('all', False)
    selected_classes = [int(c) for c in data.get('classes', [])]
    return jsonify(status="ok")

@app.route('/set_video_rec')
def set_video_rec():
    global enable_video_record
    enable_video_record = (request.args.get('enable', '1') == '1')
    return jsonify(status="ok", enable=enable_video_record)

@app.route('/set_skip')
def set_skip():
    global inference_skip
    inference_skip = max(1, min(int(request.args.get('skip', 1)), 5))
    return jsonify(status="ok", skip=inference_skip)

@app.route('/list_tests')
def list_tests():
    try:
        folders = [d for d in os.listdir(BASE_TEST_DIR) if os.path.isdir(os.path.join(BASE_TEST_DIR, d))]
        folders.sort(reverse=True)
        return jsonify(folders)
    except Exception:
        return jsonify([])

@app.route('/download_test/<folder_name>')
def download_test(folder_name):
    base_path = os.path.realpath(BASE_TEST_DIR)
    folder_path = os.path.realpath(os.path.join(base_path, folder_name))
    if (
        os.path.dirname(folder_path) != base_path
        or not os.path.isdir(folder_path)
    ):
        return "پوشه پیدا نشد", 404

    zip_buffer = io.BytesIO()
    with zipfile.ZipFile(zip_buffer, 'w', zipfile.ZIP_DEFLATED) as zip_file:
        for root, _, files in os.walk(folder_path):
            for file in files:
                file_path = os.path.join(root, file)
                zip_file.write(file_path, arcname=file)
    
    zip_buffer.seek(0)
    return send_file(zip_buffer, mimetype='application/zip', as_attachment=True, download_name=f"{folder_name}.zip")

@app.route('/set_quality')
def set_quality():
    global stream_quality
    stream_quality = max(10, min(request.args.get('quality', default=50, type=int), 95))
    return jsonify(status="ok", quality=stream_quality)

@app.route('/start_test')
def start_test():
    global recording_test, current_test_name, current_test_dir, test_start_time, latest_notification
    global video_writer, enable_video_record
    
    requested_name = request.args.get('name', default='TEST').strip()
    name = ''.join(
        character
        for character in requested_name
        if character.isalnum() or character in ('-', '_')
    ) or 'TEST'
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    folder_name = f"{name}_{timestamp}"
    
    current_test_name = name
    current_test_dir = os.path.join(BASE_TEST_DIR, folder_name)
    os.makedirs(current_test_dir, exist_ok=True)
    
    if enable_video_record:
        video_path = os.path.join(current_test_dir, f"full_test_{timestamp}.mp4")
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video_writer = cv2.VideoWriter(video_path, fourcc, 20.0, (640, 480))
    else:
        video_writer = None

    test_start_time = time.time()
    recording_test = True
    latest_notification = f"تست {name} آغاز شد. ضبط فعال است."
    return jsonify(status="ok", dir=current_test_dir)

@app.route('/stop_test')
def stop_test():
    global recording_test, test_start_time, latest_notification, video_writer
    recording_test = False
    test_start_time = None
    if video_writer is not None:
        video_writer.release()
        video_writer = None
    latest_notification = f"تست {current_test_name} پایان یافت."
    return jsonify(status="ok")

def run_flask():
    app.run(
        host=web_host,
        port=web_port,
        debug=False,
        use_reloader=False,
        threaded=True,
    )

def main(args=None):
    global ros_node, cap, detector, BASE_TEST_DIR, web_host, web_port
    rclpy.init(args=args)
    worker_thread = None
    failure = None
    try:
        ros_node = SwarmVisionNode()
        model_path = resolve_model_path(
            ros_node.get_parameter('model_path').value
        )
        detector = YoloDetector(
            model_path=model_path,
            input_size=int(ros_node.get_parameter('input_size').value),
            confidence=float(ros_node.get_parameter('confidence').value),
            nms_threshold=float(
                ros_node.get_parameter('nms_threshold').value
            ),
            inference_threads=int(
                ros_node.get_parameter('inference_threads').value
            ),
        )
        BASE_TEST_DIR = str(resolve_output_directory(
            ros_node.get_parameter('output_directory').value
        ))
        web_host = str(ros_node.get_parameter('web_host').value)
        web_port = int(ros_node.get_parameter('web_port').value)
        camera_index = int(ros_node.get_parameter('camera_index').value)
        cap = cv2.VideoCapture(camera_index)
        cap.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            int(ros_node.get_parameter('camera_width').value),
        )
        cap.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            int(ros_node.get_parameter('camera_height').value),
        )
        if not cap.isOpened():
            raise RuntimeError(
                f'Could not open camera index {camera_index}. Set '
                'camera_index to the correct /dev/video device.'
            )

        worker_thread = threading.Thread(
            target=vision_loop,
            name='swarm-vision-inference',
            daemon=True,
        )
        worker_thread.start()
        flask_thread = threading.Thread(
            target=run_flask,
            name='swarm-vision-web',
            daemon=True,
        )
        flask_thread.start()
        ros_node.get_logger().info(
            f'Web vision ready: model={model_path}, camera={camera_index}, '
            f'output={BASE_TEST_DIR}, URL=http://{web_host}:{web_port}'
        )
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        failure = error
        if ros_node is not None:
            ros_node.get_logger().fatal(str(error))
        else:
            print(f'Failed to start web vision: {error}')
    finally:
        stop_event.set()
        if worker_thread is not None and worker_thread.is_alive():
            worker_thread.join(timeout=2.0)
        if video_writer is not None:
            video_writer.release()
        if cap is not None:
            cap.release()
        if ros_node is not None:
            ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if failure is not None:
        raise SystemExit(1)

if __name__ == '__main__':
    main()
