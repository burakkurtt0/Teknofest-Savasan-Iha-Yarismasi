#yolov5_path = 'yolov7-main'
yolov5_path = 'yolov7'
model_source = '/home/burak/catkin_ws/src/offb/src/epoch_124.pt'
searching_for_uav = 'IHA ARANIYOR'
uav_detected = 'IHA TESPIT EDILDI'
chasing_uav = 'IHA TAKIP EDILIYOR'
box = []
label = "Target"
isPlane = False
tracked = False

NMS_threshold = 0.40
Conf_threshold = 0.25
kirmizi = 0, 0, 255
mavi = 255, 0, 0
yesil = 0, 255, 0
beyaz = 255, 255, 255
siyah = 0, 0, 0
