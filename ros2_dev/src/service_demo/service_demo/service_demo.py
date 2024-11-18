import sys
import rclpy
from rclpy.node import Node
from learning_interface.srv import AddTwoInts
from learning_interface.srv import GetObjectPosition
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
#客户端
class AddClient(Node):

    def __init__(self,name):
        super().__init__(name)
        self.client = self.create_client(AddTwoInts,"add_two_ints") #
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.request = AddTwoInts.Request()
    def send_request(self):
        self.request.a = int(sys.argv[1])
        self.request.b = int(sys.argv[2])
        self.future = self.client.call_async(self.request)

#客户端main函数

def client_main(args=None):
    rclpy.init(args=args)
    node = AddClient("service_adder_client")
    node.send_request()
    while rclpy.ok():         #ros2系统正常
        rclpy.spin_once(node) # 循环执行一次
        if node.future.done():
            try:
                response = node.future.result()
            except Exception as e:
                node.get_logger().info(
                    "Service all failed %r" %e
                )
            else:
                node.get_logger().info(
                    'Result of add_two_ints: for %d + %d = %d' %
                    (node.request.a, node.request.b, response.sum)
                )
            break
    node.destroy_node()
    rclpy.shutdown()


#服务端
class AddServer(Node):

    def __init__(self,name):
        super().__init__(name)
        self.srv = self.create_service(AddTwoInts,'add_two_ints',self.adder_callback)

    def adder_callback(self,request,response):
        response.sum = request.a+request.b
        self.get_logger().info("Incoming request\n a:%d b:%d"%(request.a,request.b))
        return response

#服务main函数
def srv_main(args=None):
    rclpy.init(args=args)
    node = AddServer("service_adder_server")
    rclpy.spin(node)  #循环等待ros2退出
    node.destroy_node() #销毁
    rclpy.shutdown()  #关闭




lower_red = np.array([0, 90, 128])     # 红色的HSV阈值下限
upper_red = np.array([180, 255, 255])  # 红色的HSV阈值上限

class ImageSubscriber(Node):
    def __init__(self,name):
        super().__init__(name)
        self.sub = self.create_subscription(
            Image,"image_raw",self.listener_callback,10
        )
        self.srv = self.create_service(
            GetObjectPosition,'get_target_position',
            self.object_position_callback
        )
        self.cv_bridge = CvBridge()
        self.objectX = 0
        self.objectY = 0

    def object_detect(self,image):
        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  # 图像从BGR颜色模型转换为HSV模型
        mask_red = cv2.inRange(hsv_img, lower_red, upper_red)  # 图像二值化
        contours, hierarchy = cv2.findContours(
            mask_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)  # 图像中轮廓检测

        for cnt in contours:  # 去除一些轮廓面积太小的噪声
            if cnt.shape[0] < 150:
                continue

            (x, y, w, h) = cv2.boundingRect(cnt)  # 得到苹果所在轮廓的左上角xy像素坐标及轮廓范围的宽和高
            cv2.drawContours(image, [cnt], -1, (0, 255, 0), 2)  # 将苹果的轮廓勾勒出来
            cv2.circle(image, (int(x + w / 2), int(y + h / 2)), 5,
                       (0, 255, 0), -1)  # 将苹果的图像中心点画出来

            self.objectX = int(x + w / 2)
            self.objectY = int(y + h / 2)

        cv2.imshow("object", image)  # 使用OpenCV显示处理后的图像效果
        cv2.waitKey(50)

    def listener_callback(self,data):
        self.get_logger().info("Receiving video frame")
        image = self.cv_bridge.imgmsg_to_cv2(data,'bgr8')
        self.object_detect(image)

    def object_position_callback(self,request,response):

        if request.get==True:
            response.x = self.objectX
            response.y = self.objectY
            self.get_logger().info("Object position\nx:%d y:%d"%(
                response.x,response.y
            ))
        else:
            response.x = 0
            response.y = 0
            self.get_logger().info("Invalid command")
        return response




def object_server_main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber("service_object_server")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



class ObjectClient(Node):
    def __init__(self,name):
        super().__init__(name)
        self.client = self.create_client(GetObjectPosition,"get_target_position") #
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.request = GetObjectPosition.Request()
    def send_request(self):
        self.request.get = True
        self.future = self.client.call_async(self.request)

def object_client_main(args=None):
    rclpy.init(args=args)

    node = ObjectClient("service_object_client")
    node.send_request()

    while rclpy.ok():
        rclpy.spin_once(node)

        if node.future.done():
            try:
                response = node.future.result()
            except Exception as e:
                node.get_logger().info(
                    'Service call failed %r'%e
                )
            else:
                node.get_logger().info(
                    'Result of object position:\n x: %d y: %d' %
                    (response.x, response.y)
                )
            break
    node.destroy_node()
    rclpy.shutdown()

