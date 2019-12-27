#! /usr/bin/env python

import time

import numpy as np
import tensorflow as tf
from keras.models import load_model
from tf import TransformListener

import cv2
import scipy.ndimage as ndimage
from skimage.draw import circle
from skimage.feature import peak_local_max

import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo, JointState
from std_msgs.msg import Float32MultiArray

import copy

bridge = CvBridge()


# Load the Network.
MODEL_FILE = '/home/caio/2_ROS/Doutorado/src/ggcnn/data/networks/ggcnn_rss/epoch_29_model.hdf5'
with tf.device('/device:GPU:0'):
    model = load_model(MODEL_FILE)

rospy.init_node('ggcnn_detection')
transf = TransformListener()

# Output publishers.
grasp_pub = rospy.Publisher('ggcnn/img/grasp', Image, queue_size=1)
grasp_plain_pub = rospy.Publisher('ggcnn/img/grasp_plain', Image, queue_size=1)
depth_pub = rospy.Publisher('ggcnn/img/depth', Image, queue_size=1)
ang_pub = rospy.Publisher('ggcnn/img/ang', Image, queue_size=1)
cmd_pub = rospy.Publisher('ggcnn/out/command', Float32MultiArray, queue_size=1)

camera_topic_info = '/kinect2/qhd/camera_info'
camera_topic = '/kinect2/qhd/image_depth_rect'

'FOR TEST'
depth_pub2 = rospy.Publisher('ggcnn/img/antes', Image, queue_size=1)
depth_pub3 = rospy.Publisher('ggcnn/img/depois', Image, queue_size=1)
cmd_test = rospy.Publisher('ggcnn/out/test', Image, queue_size=1)

# Initialise some globals.
prev_mp = np.array([150, 150])
ROBOT_Z = 0

# Tensorflow graph to allow use in callback.
graph = tf.get_default_graph()

# Get the camera parameters
camera_info_msg = rospy.wait_for_message(camera_topic_info, CameraInfo)
K = camera_info_msg.K
fx = K[0]
cx = K[2]
fy = K[4]
cy = K[5]


# Execution Timing
class TimeIt:
    def __init__(self, s):
        # it is called first
        self.s = s
        self.t0 = None
        self.t1 = None
        self.print_output = False

    def __enter__(self):
        # it is called secondly
        # after that, what is inside with TimeIT is called
        self.t0 = time.time()

    def __exit__(self, t, value, traceback):
        # after what is inside TimeIt function finishes, it is called
        self.t1 = time.time()
        if self.print_output:
            print('%s: %s' % (self.s, self.t1 - self.t0))

# not used anymore
def robot_pos_callback(data):
    global ROBOT_Z

    # ROBOT_Z = data.pose.position.z
    ROBOT_Z = 0.35

def depth_callback(depth_message):
    global model
    global graph
    global prev_mp
    global ROBOT_Z
    global fx, cx, fy, cy
    global transf

    # The EOF position should be tracked in real time by depth_callback
    link_pose, _ = transf.lookupTransform("base_link", "grasping_link", rospy.Time(0))
    ROBOT_Z = link_pose[2]

    # cada width eh utilizado para calcular o tempo de cada processamento
    with TimeIt('Crop'):
        depth = bridge.imgmsg_to_cv2(depth_message)

        # Crop a square out of the middle of the depth and resize it to 300*300
        # Resolution option: option 1 for 480x640, option 2 for 960x540
        crop_size = 400
        option = 2

        if option:
            height_res = 480
            width_res = 640
        elif option == 2:
            height_res = 540
            width_res = 960

        # // return the int value of quotient
        # camera size is 480x680 and the default operation crops the image to
        # height: 40 - 440 e width: 120 - 520
        # adicionando o space, chegamos a janela para cima, entao o height fica: 80 - 480
        # (height_res - crop_size)//2 is used to select the uppermost part of the image
        # if option 1 is chosen then (480 - 400)//2 = 40
        depth_crop = cv2.resize(depth[(height_res - crop_size)//2 + (height_res - crop_size)//2:(height_res - crop_size)//2 + crop_size + (height_res - crop_size)//2,
                                      (width_res - crop_size)//2:(width_res - crop_size)//2+crop_size],
                                      (300, 300))

        'Tratamento para retirar NaN'
        # Replace nan with 0 for inpainting.
        # Com shallow copy (.copy()). Todas as mudancas feitas no objeto copiado tambem sao feitas no objeto original
        depth_crop = depth_crop.copy()
        # Verificar quais sao os valores com nan e retorna True na posicao do valor nan - not a number
        depth_nan = np.isnan(depth_crop)
        depth_nan = depth_nan.copy()
        # substitui a posicao onde tem True por zero
        depth_crop[depth_nan] = 0
        'Fim'

    with TimeIt('Inpaint'):
        # open cv inpainting does weird things at the border.
        # default border is a constant color
        depth_crop = cv2.copyMakeBorder(depth_crop, 1, 1, 1, 1, cv2.BORDER_DEFAULT)

        # se o numero que esta no vetor acima for 0, retorna o numero 1 na mesma posicao (como se fosse True)
        # se depth_crop == 0, retorna 1 como inteiro.
        # Ou seja, copia os pixels pretos da imagem e a posicao deles
        mask = (depth_crop == 0).astype(np.uint8)

        # depth_crop = depth_crop / 1000

        # Scale to keep as float, but has to be in bounds -1:1 to keep opencv happy.
        # Copia o maior valor para depois realizar a escala
        depth_scale = np.abs(depth_crop).max()

        'TESTE - ANTES'
        # antes = copy.deepcopy(depth_crop)
        # depth_pub2.publish(bridge.cv2_to_imgmsg(antes))

        # Normalize
        depth_crop = depth_crop.astype(np.float32)/depth_scale  # Has to be float32, 64 not supported.

        # aplica a mascara no depth_crop
        # a mascara ja possui os valores dos pixels pretos (informacoes falhas do sensor kinect)
        depth_crop = cv2.inpaint(depth_crop, mask, 1, cv2.INPAINT_NS)

        # Back to original size and value range.
        # Retira as bordas da imagem, pegando os pixels de dentro
        depth_crop = depth_crop[1:-1, 1:-1]

        # depois da escala a imagem fica totalmente branca
        # perde alguma informacao?
        depth_crop = depth_crop * depth_scale

    # with TimeIt('Calculate Depth'):
        # Figure out roughly the depth in mm of the part between the grippers for collision avoidance.
        # depth_center = depth_crop[100:141, 130:171].flatten()
        # depth_center.sort()
        # depth_center = depth_center[:10].mean() * 1000.0

    with TimeIt('Inference'):
        # Run it through the network.
        # Peter - metros -> nunca vai ter valores maiores que 1
        # Kinect -> vamos ter valores maiores do que 1 de profundidade
        # print(depth_crop.mean())
        # The D435 publishes depth in "16-bit unsigned integers in millimeter resolution."

        # values smaller than -1 become -1, and values larger than 1 become 1.
        depth_crop = np.clip((depth_crop - depth_crop.mean()), -0.2, 0.2)

        'TESTE - DEPOIS'
        # depois = copy.deepcopy(depth_crop)
        # depth_pub3.publish(bridge.cv2_to_imgmsg(depois))

        with graph.as_default():
            pred_out = model.predict(depth_crop.reshape((1, 300, 300, 1)))

        # tambem retira os NaN presente no depth_crop
        # pred_out shape is (4, 1, 300, 300, 1)
        points_out = pred_out[0].squeeze()
        points_out[depth_nan] = 0

    with TimeIt('Trig'):
        # Calculate the angle map.
        cos_out = pred_out[1].squeeze()
        sin_out = pred_out[2].squeeze()
        ang_out = np.arctan2(sin_out, cos_out)/2.0

        width_out = pred_out[3].squeeze() * 150.0  # Scaled 0-150:0-1

    with TimeIt('Filter'):
        # Filter the outputs
        # Os filtros sao aplicados para aumentar os grasps identificados
        points_out = ndimage.filters.gaussian_filter(points_out, 5.0)  # 3.0
        ang_out = ndimage.filters.gaussian_filter(ang_out, 2.0)

    with TimeIt('Control'):
        # Calculate the best pose from the camera intrinsics.
        maxes = None

        ALWAYS_MAX = True  # Use ALWAYS_MAX = True for the open-loop solution.

        # ou seja, se a altura do tool for maior que 0.34 ou open_open loop, o codigo abaixo eh executado
        if ROBOT_Z > 0.34 or ALWAYS_MAX:  # > 0.34 initialises the max tracking when the robot is reset.
            # Track the global max.
            # argmax returns max pixel in points_out
            # unravel_index returns the index of that max pixel as coordinates as tuple
            max_pixel = np.array(np.unravel_index(np.argmax(points_out), points_out.shape))
            # deve ser a coordenada do max pixel identificado
            prev_mp = max_pixel.astype(np.int)
        else:
            # Calculate a set of local maxes.  Choose the one that is closes to the previous one.
            maxes = peak_local_max(points_out, min_distance=10, threshold_abs=0.1, num_peaks=3)
            if maxes.shape[0] == 0:
                return
            max_pixel = maxes[np.argmin(np.linalg.norm(maxes - prev_mp, axis=1))]

            # Keep a global copy for next iteration.
            prev_mp = (max_pixel * 0.25 + prev_mp * 0.75).astype(np.int)

        # obtem a coordenada x e y do max_pixel
        ang = ang_out[max_pixel[0], max_pixel[1]]
        width = width_out[max_pixel[0], max_pixel[1]]

        # Convert max_pixel back to uncropped/resized image coordinates in order to do the camera transform.
        # + [(480 - 400) // 2, (680 - 400) // 2]
        # + [40, 140]
        max_pixel = ((np.array(max_pixel) / 300.0 * crop_size) + np.array([(height_res - crop_size) // 2, (width_res - crop_size) // 2]))
        max_pixel = np.round(max_pixel).astype(np.int)

        # altura do ponto identificado
        point_depth = depth[max_pixel[0], max_pixel[1]]
        # print(point_depth)

        # These magic numbers are my camera intrinsic parameters.
        x = (max_pixel[1] - cx)/(fx) * point_depth
        y = (max_pixel[0] - cy)/(fy) * point_depth
        z = point_depth

        if np.isnan(z):
            return

    with TimeIt('Draw'):
        # Draw grasp markers on the points_out and publish it. (for visualisation)
        # points_out was used in gaussian_filter for last
        grasp_img = np.zeros((300, 300, 3), dtype=np.uint8)
        # Draw the red area in the image
        grasp_img[:,:,2] = (points_out * 255.0)

        # grasp plain does not have the green point
        grasp_img_plain = grasp_img.copy()

        # draw the circle at the green point
        rr, cc = circle(prev_mp[0], prev_mp[1], 5)
        grasp_img[rr, cc, 0] = 0
        grasp_img[rr, cc, 1] = 255
        grasp_img[rr, cc, 2] = 0

    with TimeIt('Publish'):
        # Publish the output images (not used for control, only visualisation)
        grasp_img = bridge.cv2_to_imgmsg(grasp_img, 'bgr8')
        grasp_img.header = depth_message.header
        grasp_pub.publish(grasp_img)

        grasp_img_plain = bridge.cv2_to_imgmsg(grasp_img_plain, 'bgr8')
        grasp_img_plain.header = depth_message.header
        grasp_plain_pub.publish(grasp_img_plain)

        depth_pub.publish(bridge.cv2_to_imgmsg(depth_crop))
        ang_pub.publish(bridge.cv2_to_imgmsg(ang_out))

        # Output the best grasp pose relative to camera.
        cmd_msg = Float32MultiArray()
        cmd_msg.data = [x, y, z, ang, width]#, depth_center]
        cmd_pub.publish(cmd_msg)

depth_sub = rospy.Subscriber(camera_topic, Image, depth_callback, queue_size=1)

# robot_pos_sub = rospy.Subscriber('/m1n6s200_driver/out/tool_pose', PoseStamped, robot_pos_callback, queue_size=1)

while not rospy.is_shutdown():
    rospy.spin()
